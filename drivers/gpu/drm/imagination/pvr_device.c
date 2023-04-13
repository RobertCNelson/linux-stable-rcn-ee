// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#include "pvr_device.h"
#include "pvr_device_info.h"

#include "pvr_fw.h"
#include "pvr_params.h"
#include "pvr_power.h"
#include "pvr_rogue_cr_defs.h"
#include "pvr_stream.h"

#include <drm/drm_print.h>

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/compiler_attributes.h>
#include <linux/compiler_types.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/gfp.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/types.h>
#include <linux/workqueue.h>

/* Major number for the supported version of the firmware. */
#define PVR_FW_VERSION_MAJOR 1

/**
 * pvr_device_reg_init() - Initialize kernel access to a PowerVR device's
 * control registers.
 * @pvr_dev: Target PowerVR device.
 *
 * Sets struct pvr_device->regs.
 *
 * This method of mapping the device control registers into memory ensures that
 * they are unmapped when the driver is detached (i.e. no explicit cleanup is
 * required).
 *
 * Return:
 *  * 0 on success, or
 *  * Any error returned by devm_platform_ioremap_resource().
 */
static int
pvr_device_reg_init(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct platform_device *plat_dev = to_platform_device(drm_dev->dev);
	struct resource *regs_resource;
	void __iomem *regs;
	int err;

	pvr_dev->regs_resource = NULL;
	pvr_dev->regs = NULL;

	regs = devm_platform_get_and_ioremap_resource(plat_dev, 0, &regs_resource);
	if (IS_ERR(regs)) {
		err = PTR_ERR(regs);
		drm_err(drm_dev, "failed to ioremap gpu registers (err=%d)\n",
			err);
		return err;
	}

	pvr_dev->regs = regs;
	pvr_dev->regs_resource = regs_resource;

	return 0;
}

/**
 * pvr_device_reg_fini() - Deinitialize kernel access to a PowerVR device's
 * control registers.
 * @pvr_dev: Target PowerVR device.
 *
 * This is essentially a no-op, since pvr_device_reg_init() already ensures that
 * struct pvr_device->regs is unmapped when the device is detached. This
 * function just sets struct pvr_device->regs to %NULL.
 */
static __always_inline void
pvr_device_reg_fini(struct pvr_device *pvr_dev)
{
	pvr_dev->regs = NULL;
}

/**
 * pvr_device_clk_init() - Initialize clocks required by a PowerVR device
 * @pvr_dev: Target PowerVR device.
 *
 * Sets struct pvr_device->core_clk, struct pvr_device->sys_clk and
 * struct pvr_device->mem_clk.
 *
 * Three clocks are required by the PowerVR device: core, sys and mem. On
 * return, this function guarantees that the clocks are in one of the following
 * states:
 *
 *  * All successfully initialized,
 *  * Core errored, sys and mem uninitialized,
 *  * Core deinitialized, sys errored, mem uninitialized, or
 *  * Core and sys deinitialized, mem errored.
 *
 * Return:
 *  * 0 on success,
 *  * Any error returned by devm_clk_get(), or
 *  * Any error returned by clk_prepare_enable().
 */
static int pvr_device_clk_init(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct clk *core_clk;
	struct clk *sys_clk;
	struct clk *mem_clk;
	int err;

	pvr_dev->core_clk = NULL;
	pvr_dev->sys_clk = NULL;
	pvr_dev->mem_clk = NULL;

	core_clk = devm_clk_get(drm_dev->dev, "core_clk");
	if (IS_ERR(core_clk)) {
		err = PTR_ERR(core_clk);
		drm_err(drm_dev, "failed to get core_clk (err=%d)\n", err);
		goto err_out;
	}

	sys_clk = devm_clk_get(drm_dev->dev, "sys_clk");
	if (IS_ERR(sys_clk))
		sys_clk = NULL;

	mem_clk = devm_clk_get(drm_dev->dev, "mem_clk");
	if (IS_ERR(mem_clk))
		mem_clk = NULL;

	err = clk_prepare(core_clk);
	if (err)
		goto err_out;

	if (sys_clk) {
		err = clk_prepare(sys_clk);
		if (err)
			goto err_deinit_core_clk;
	}

	if (mem_clk) {
		err = clk_prepare(mem_clk);
		if (err)
			goto err_deinit_sys_clk;
	}

	pvr_dev->core_clk = core_clk;
	pvr_dev->sys_clk = sys_clk;
	pvr_dev->mem_clk = mem_clk;

	return 0;

err_deinit_sys_clk:
	if (sys_clk)
		clk_disable_unprepare(sys_clk);
err_deinit_core_clk:
	clk_disable_unprepare(core_clk);
err_out:
	return err;
}

/**
 * pvr_device_clk_fini() - Deinitialize clocks required by a PowerVR device
 * @pvr_dev: Target PowerVR device.
 */
static void
pvr_device_clk_fini(struct pvr_device *pvr_dev)
{
	if (pvr_dev->mem_clk)
		clk_unprepare(pvr_dev->mem_clk);
	if (pvr_dev->sys_clk)
		clk_unprepare(pvr_dev->sys_clk);
	clk_unprepare(pvr_dev->core_clk);

	pvr_dev->core_clk = NULL;
	pvr_dev->sys_clk = NULL;
	pvr_dev->mem_clk = NULL;
}

/**
 * pvr_device_regulator_init() - Initialise regulator required by a PowerVR device
 * @pvr_dev: Target PowerVR device.
 *
 * The regulator is not a required devicetree property. If it is not present then this function will
 * succeed, but &pvr_device->regulator will be %NULL.
 *
 * Returns:
 *  * 0 on success, or
 *  * Any error returned by devm_regulator_get().
 */
static int
pvr_device_regulator_init(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct regulator *regulator;
	int err;

	regulator = devm_regulator_get(drm_dev->dev, "power");
	if (IS_ERR(regulator)) {
		err = PTR_ERR(regulator);
		/* Regulator is not required, so ENODEV is allowed here. */
		if (err != -ENODEV)
			goto err_out;
		regulator = NULL;
	}

	pvr_dev->regulator = regulator;

	return 0;

err_out:
	return err;
}

/**
 * pvr_device_clk_core_get_freq - Get current PowerVR device core clock frequency
 * @pvr_dev: Target PowerVR device.
 * @freq_out: Pointer to location to store core clock frequency in Hz.
 *
 * Returns:
 *  * 0 on success, or
 *  * -%EINVAL if frequency can not be determined.
 */
int
pvr_device_clk_core_get_freq(struct pvr_device *pvr_dev, u32 *freq_out)
{
	u32 freq = clk_get_rate(pvr_dev->core_clk);

	if (!freq)
		return -EINVAL;

	*freq_out = freq;
	return 0;
}

static irqreturn_t pvr_meta_irq_handler(int irq, void *data)
{
	struct pvr_device *pvr_dev = data;

	if (!pvr_dev->fw_dev.funcs->check_and_ack_irq(pvr_dev))
		return IRQ_NONE; /* Spurious IRQ - ignore. */

	/* Only process IRQ work if FW is currently running. */
	if (pvr_dev->fw_dev.booted) {
		queue_work(pvr_dev->irq_wq, &pvr_dev->fwccb_work);
		wake_up(&pvr_dev->kccb_rtn_q);
		queue_work(pvr_dev->irq_wq, &pvr_dev->context_work);
		pvr_power_check_idle(pvr_dev);
	}

	return IRQ_HANDLED;
}

/**
 * pvr_device_irq_init() - Initialise IRQ required by a PowerVR device
 * @pvr_dev: Target PowerVR device.
 *
 * Returns:
 *  * 0 on success,
 *  * Any error returned by platform_get_irq_byname(), or
 *  * Any error returned by request_irq().
 */
static int
pvr_device_irq_init(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct platform_device *plat_dev = to_platform_device(drm_dev->dev);
	int err;

	init_waitqueue_head(&pvr_dev->kccb_rtn_q);

	pvr_dev->irq_wq = alloc_workqueue("powervr-irq", WQ_UNBOUND, 0);
	if (!pvr_dev->irq_wq) {
		err = -ENOMEM;
		goto err_out;
	}

	pvr_dev->irq = platform_get_irq_byname(plat_dev, "gpu");
	if (pvr_dev->irq < 0) {
		err = pvr_dev->irq;
		goto err_destroy_wq;
	}

	err = request_irq(pvr_dev->irq, pvr_meta_irq_handler, IRQF_SHARED, NULL, pvr_dev);
	if (err)
		goto err_destroy_wq;

	return 0;

err_destroy_wq:
	destroy_workqueue(pvr_dev->irq_wq);

err_out:
	return err;
}

/**
 * pvr_device_irq_fini() - Deinitialise IRQ required by a PowerVR device
 * @pvr_dev: Target PowerVR device.
 */
static void
pvr_device_irq_fini(struct pvr_device *pvr_dev)
{
	free_irq(pvr_dev->irq, pvr_dev);
	destroy_workqueue(pvr_dev->irq_wq);
}

/**
 * pvr_build_firmware_filename() - Construct a PowerVR firmware filename
 * @pvr_dev: Target PowerVR device.
 * @base: First part of the filename.
 * @major: Major version number.
 *
 * A PowerVR firmware filename consists of three parts separated by underscores
 * (``'_'``) along with a '.fw' file suffix. The first part is the exact value
 * of @base, the second part is the hardware version string derived from @pvr_fw
 * and the final part is the firmware version number constructed from @major with
 * a 'v' prefix, e.g. powervr/rogue_4.40.2.51_v1.fw.
 *
 * The returned string will have been slab allocated and must be freed with
 * kfree().
 *
 * Return:
 *  * The constructed filename on success, or
 *  * Any error returned by kasprintf().
 */
static char *
pvr_build_firmware_filename(struct pvr_device *pvr_dev, const char *base,
			    u8 major)
{
	struct pvr_gpu_id *gpu_id = &pvr_dev->gpu_id;

	return kasprintf(GFP_KERNEL, "%s_%d.%d.%d.%d_v%d.fw", base, gpu_id->b,
			 gpu_id->v, gpu_id->n, gpu_id->c, major);
}

/**
 * pvr_request_firmware() - Load firmware for a PowerVR device
 * @pvr_dev: Target PowerVR device.
 *
 * See pvr_build_firmware_filename() for details on firmware file naming.
 *
 * Return:
 *  * 0 on success,
 *  * Any error returned by pvr_build_firmware_filename(), or
 *  * Any error returned by request_firmware().
 */
static int
pvr_request_firmware(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = &pvr_dev->base;
	char *filename;
	const struct firmware *fw;
	int err;

	filename = pvr_build_firmware_filename(pvr_dev, "powervr/rogue",
					       PVR_FW_VERSION_MAJOR);
	if (IS_ERR(filename))
		return PTR_ERR(filename);

	/*
	 * This function takes a copy of &filename, meaning we can free our
	 * instance before returning.
	 */
	err = request_firmware(&fw, filename, pvr_dev->base.dev);
	if (err) {
		drm_err(drm_dev, "failed to load firmware %s (err=%d)\n",
			filename, err);
		goto err_free_filename;
	}

	drm_info(drm_dev, "loaded firmware %s\n", filename);
	kfree(filename);

	pvr_dev->fw_dev.firmware = fw;

	return 0;

err_free_filename:
	kfree(filename);

	return err;
}

/**
 * pvr_load_gpu_id() - Load a PowerVR device's GPU ID (BVNC) from control registers.
 *
 * Sets struct pvr_dev.gpu_id.
 *
 * @pvr_dev: Target PowerVR device.
 */
static void
pvr_load_gpu_id(struct pvr_device *pvr_dev)
{
	struct pvr_gpu_id *gpu_id = &pvr_dev->gpu_id;
	u64 bvnc;

	/*
	 * Try reading the BVNC using the newer (cleaner) method first. If the
	 * B value is zero, fall back to the older method.
	 */
	bvnc = PVR_CR_READ64(pvr_dev, CORE_ID__PBVNC);

	gpu_id->b = PVR_CR_FIELD_GET(bvnc, CORE_ID__PBVNC__BRANCH_ID);
	if (gpu_id->b != 0) {
		gpu_id->v = PVR_CR_FIELD_GET(bvnc, CORE_ID__PBVNC__VERSION_ID);
		gpu_id->n = PVR_CR_FIELD_GET(bvnc, CORE_ID__PBVNC__NUMBER_OF_SCALABLE_UNITS);
		gpu_id->c = PVR_CR_FIELD_GET(bvnc, CORE_ID__PBVNC__CONFIG_ID);
	} else {
		u32 core_rev = PVR_CR_READ32(pvr_dev, CORE_REVISION);
		u32 core_id = PVR_CR_READ32(pvr_dev, CORE_ID);
		u16 core_id_config = PVR_CR_FIELD_GET(core_id, CORE_ID_CONFIG);

		gpu_id->b = PVR_CR_FIELD_GET(core_rev, CORE_REVISION_MAJOR);
		gpu_id->v = PVR_CR_FIELD_GET(core_rev, CORE_REVISION_MINOR);
		gpu_id->n = FIELD_GET(0xFF00, core_id_config);
		gpu_id->c = FIELD_GET(0x00FF, core_id_config);
	}
}

/**
 * pvr_set_dma_info() - Set PowerVR device DMA information
 * @pvr_dev: Target PowerVR device.
 *
 * Sets the DMA mask and max segment size for the PowerVR device.
 *
 * Return:
 *  * 0 on success,
 *  * Any error returned by PVR_FEATURE_VALUE(), or
 *  * Any error returned by dma_set_mask().
 */

static int
pvr_set_dma_info(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	u16 phys_bus_width;
	int err;

	err = PVR_FEATURE_VALUE(pvr_dev, phys_bus_width, &phys_bus_width);
	if (err) {
		drm_err(drm_dev, "Failed to get device physical bus width\n");
		return err;
	}

	/*
	 * See the comment on &pvr_drm_driver.prime_fd_to_handle for an
	 * explanation of the dma_set_mask function and dma_set_max_seg_size
	 * calls below.
	 */
	err = dma_set_mask(drm_dev->dev, DMA_BIT_MASK(phys_bus_width));
	if (err) {
		drm_err(drm_dev, "Failed to set DMA mask (err=%d)\n", err);
		return err;
	}

	dma_set_max_seg_size(drm_dev->dev, UINT_MAX);

	return 0;
}

/**
 * pvr_device_gpu_init() - GPU-specific initialization for a PowerVR device
 * @pvr_dev: Target PowerVR device.
 *
 * The following steps are taken to ensure the device is ready:
 *
 *  1. Read the hardware version information from control registers,
 *  2. Initialise the hardware feature information,
 *  3. Setup the device DMA information,
 *  4. Setup the device-scoped memory context, and
 *  5. Load firmware into the device.
 *
 * Return:
 *  * 0 on success,
 *  * -%ENODEV if the GPU is not supported,
 *  * Any error returned by pvr_set_dma_info(),
 *  * Any error returned by pvr_memory_context_init(), or
 *  * Any error returned by pvr_request_firmware().
 */
static int
pvr_device_gpu_init(struct pvr_device *pvr_dev)
{
	int err;

	pvr_load_gpu_id(pvr_dev);

	err = pvr_device_info_init(pvr_dev);
	if (err)
		goto err_out;

	if (PVR_HAS_FEATURE(pvr_dev, meta)) {
		pvr_dev->fw_dev.processor_type = PVR_FW_PROCESSOR_TYPE_META;
	} else if (PVR_HAS_FEATURE(pvr_dev, mips)) {
		pvr_dev->fw_dev.processor_type = PVR_FW_PROCESSOR_TYPE_MIPS;
	} else if (PVR_HAS_FEATURE(pvr_dev, riscv_fw_processor)) {
		pvr_dev->fw_dev.processor_type = PVR_FW_PROCESSOR_TYPE_RISCV;
	} else {
		err = -EINVAL;
		goto err_out;
	}

	pvr_stream_create_musthave_masks(pvr_dev);

	err = pvr_set_dma_info(pvr_dev);
	if (err)
		goto err_out;

	pvr_dev->kernel_vm_ctx = pvr_vm_create_context(pvr_dev, false);
	if (IS_ERR(pvr_dev->kernel_vm_ctx)) {
		err = PTR_ERR(pvr_dev->kernel_vm_ctx);
		goto err_out;
	}

	err = pvr_request_firmware(pvr_dev);
	if (err)
		goto err_vm_ctx_put;

	err = pvr_fw_init(pvr_dev);
	if (err)
		goto err_release_firmware;

	return 0;

err_release_firmware:
	release_firmware(pvr_dev->fw_dev.firmware);

err_vm_ctx_put:
	pvr_vm_context_put(pvr_dev->kernel_vm_ctx);
	pvr_dev->kernel_vm_ctx = NULL;

err_out:
	return err;
}

/**
 * pvr_device_gpu_fini() - GPU-specific deinitialization for a PowerVR device
 * @pvr_dev: Target PowerVR device.
 */
static void
pvr_device_gpu_fini(struct pvr_device *pvr_dev)
{
	pvr_fw_fini(pvr_dev);
	release_firmware(pvr_dev->fw_dev.firmware);
	WARN_ON(!pvr_vm_context_put(pvr_dev->kernel_vm_ctx));
	pvr_dev->kernel_vm_ctx = NULL;
}

/**
 * pvr_device_init() - Initialize a PowerVR device
 * @pvr_dev: Target PowerVR device.
 *
 * If this function returns successfully, the device will have been fully
 * initialized. Otherwise, any parts of the device initialized before an error
 * occurs will be de-initialized before returning.
 *
 * NOTE: The initialization steps currently taken are the bare minimum required
 *       to read from the control registers. The device is unlikely to function
 *       until further initialization steps are added. [This note should be
 *       removed when that happens.]
 *
 * Return:
 *  * 0 on success,
 *  * Any error returned by pvr_device_reg_init(),
 *  * Any error returned by pvr_device_clk_init(), or
 *  * Any error returned by pvr_device_gpu_init().
 */
int
pvr_device_init(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct device *dev = drm_dev->dev;
	int err;

	/*
	 * Setup device parameters. We do this first in case other steps
	 * depend on them.
	 */
	err = pvr_device_params_init(&pvr_dev->params);
	if (err)
		return err;

	/* Enable and initialize clocks required for the device to operate. */
	err = pvr_device_clk_init(pvr_dev);
	if (err)
		goto err_out;

	err = pvr_device_regulator_init(pvr_dev);
	if (err)
		goto err_device_clk_fini;

	/* Explicitly power the GPU so we can access control registers before the FW is booted. */
	err = pm_runtime_get_sync(dev);
	if (err)
		goto err_device_clk_fini;

	/* Map the control registers into memory. */
	err = pvr_device_reg_init(pvr_dev);
	if (err)
		goto err_pm_runtime_put;

	err = pvr_device_irq_init(pvr_dev);
	if (err)
		goto err_device_reg_fini;

	/* Perform GPU-specific initialization steps. */
	err = pvr_device_gpu_init(pvr_dev);
	if (err)
		goto err_device_irq_fini;

	pm_runtime_put_autosuspend(dev);

	return 0;

err_device_irq_fini:
	pvr_device_irq_fini(pvr_dev);

err_device_reg_fini:
	pvr_device_reg_fini(pvr_dev);

err_pm_runtime_put:
	pm_runtime_put_sync_suspend(dev);

err_device_clk_fini:
	pvr_device_clk_fini(pvr_dev);

err_out:
	return err;
}

/**
 * pvr_device_fini() - Deinitialize a PowerVR device
 * @pvr_dev: Target PowerVR device.
 */
void
pvr_device_fini(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct device *dev = drm_dev->dev;

	/*
	 * Deinitialization stages are performed in reverse order compared to
	 * the initialization stages in pvr_device_init().
	 */
	pm_runtime_get_sync(dev);
	pvr_device_gpu_fini(pvr_dev);
	pvr_device_irq_fini(pvr_dev);
	pvr_device_reg_fini(pvr_dev);
	pm_runtime_put_sync_suspend(dev);
	pvr_device_clk_fini(pvr_dev);

	/* TODO: Remaining deinitialization steps */
}

bool
pvr_device_has_uapi_quirk(struct pvr_device *pvr_dev, u32 quirk)
{
	switch (quirk) {
	case 47217:
		return PVR_HAS_QUIRK(pvr_dev, 47217);
	case 48545:
		return PVR_HAS_QUIRK(pvr_dev, 48545);
	case 49927:
		return PVR_HAS_QUIRK(pvr_dev, 49927);
	case 51764:
		return PVR_HAS_QUIRK(pvr_dev, 51764);
	case 62269:
		return PVR_HAS_QUIRK(pvr_dev, 62269);
	default:
		return false;
	};
}

bool
pvr_device_has_uapi_enhancement(struct pvr_device *pvr_dev, u32 enhancement)
{
	switch (enhancement) {
	case 35421:
		return PVR_HAS_ENHANCEMENT(pvr_dev, 35421);
	case 42064:
		return PVR_HAS_ENHANCEMENT(pvr_dev, 42064);
	default:
		return false;
	};
}

/**
 * pvr_device_has_feature() - Look up device feature based on feature definition
 * @pvr_dev: Device pointer.
 * @feature: Feature to look up. Should be one of %PVR_FEATURE_*.
 *
 * Returns:
 *  * %true if feature is present on device, or
 *  * %false if feature is not present on device.
 */
bool
pvr_device_has_feature(struct pvr_device *pvr_dev, u32 feature)
{
	switch (feature) {
	case PVR_FEATURE_CLUSTER_GROUPING:
		return PVR_HAS_FEATURE(pvr_dev, cluster_grouping);

	case PVR_FEATURE_COMPUTE_MORTON_CAPABLE:
		return PVR_HAS_FEATURE(pvr_dev, compute_morton_capable);

	case PVR_FEATURE_FB_CDC_V4:
		return PVR_HAS_FEATURE(pvr_dev, fb_cdc_v4);

	case PVR_FEATURE_GPU_MULTICORE_SUPPORT:
		return PVR_HAS_FEATURE(pvr_dev, gpu_multicore_support);

	case PVR_FEATURE_ISP_ZLS_D24_S8_PACKING_OGL_MODE:
		return PVR_HAS_FEATURE(pvr_dev, isp_zls_d24_s8_packing_ogl_mode);

	case PVR_FEATURE_S7_TOP_INFRASTRUCTURE:
		return PVR_HAS_FEATURE(pvr_dev, s7_top_infrastructure);

	case PVR_FEATURE_TESSELLATION:
		return PVR_HAS_FEATURE(pvr_dev, tessellation);

	case PVR_FEATURE_TPU_DM_GLOBAL_REGISTERS:
		return PVR_HAS_FEATURE(pvr_dev, tpu_dm_global_registers);

	case PVR_FEATURE_VDM_DRAWINDIRECT:
		return PVR_HAS_FEATURE(pvr_dev, vdm_drawindirect);

	case PVR_FEATURE_VDM_OBJECT_LEVEL_LLS:
		return PVR_HAS_FEATURE(pvr_dev, vdm_object_level_lls);

	case PVR_FEATURE_ZLS_SUBTILE:
		return PVR_HAS_FEATURE(pvr_dev, zls_subtile);

	/* Derived features. */
	case PVR_FEATURE_CDM_USER_MODE_QUEUE: {
		u8 cdm_control_stream_format = 0;

		PVR_FEATURE_VALUE(pvr_dev, cdm_control_stream_format, &cdm_control_stream_format);
		return (cdm_control_stream_format >= 2 && cdm_control_stream_format <= 4);
	}

	case PVR_FEATURE_REQUIRES_FB_CDC_ZLS_SETUP:
		if (PVR_HAS_FEATURE(pvr_dev, fbcdc_algorithm)) {
			u8 fbcdc_algorithm = 0;

			PVR_FEATURE_VALUE(pvr_dev, fbcdc_algorithm, &fbcdc_algorithm);
			return (fbcdc_algorithm < 3 || PVR_HAS_FEATURE(pvr_dev, fb_cdc_v4));
		}
		return false;

	default:
		WARN(true, "Looking up undefined feature %u\n", feature);
		return false;
	}
}
