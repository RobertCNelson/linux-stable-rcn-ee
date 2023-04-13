// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#include "pvr_device.h"
#include "pvr_fw.h"
#include "pvr_fw_info.h"
#include "pvr_gem.h"
#include "pvr_rogue_cr_defs.h"
#include "pvr_rogue_meta.h"

#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/ktime.h>
#include <linux/types.h>

#define ROGUE_FW_HEAP_META_SHIFT 25 /* 32 MB */

#define POLL_TIMEOUT_USEC 1000000

/**
 * pvr_meta_cr_read32() - Read a META register via the Slave Port
 * @pvr_dev: Device pointer.
 * @reg_addr: Address of register to read.
 * @reg_value_out: Pointer to location to store register value.
 *
 * Returns:
 *  * 0 on success, or
 *  * Any error returned by pvr_cr_poll_reg32().
 */
int
pvr_meta_cr_read32(struct pvr_device *pvr_dev, u32 reg_addr, u32 *reg_value_out)
{
	int err;

	/* Wait for Slave Port to be Ready. */
	err = pvr_cr_poll_reg32(pvr_dev, ROGUE_CR_META_SP_MSLVCTRL1,
				ROGUE_CR_META_SP_MSLVCTRL1_READY_EN |
					ROGUE_CR_META_SP_MSLVCTRL1_GBLPORT_IDLE_EN,
				ROGUE_CR_META_SP_MSLVCTRL1_READY_EN |
					ROGUE_CR_META_SP_MSLVCTRL1_GBLPORT_IDLE_EN,
				POLL_TIMEOUT_USEC);
	if (err)
		goto err_out;

	/* Issue a Read. */
	PVR_CR_WRITE32(pvr_dev, META_SP_MSLVCTRL0,
		       reg_addr | ROGUE_CR_META_SP_MSLVCTRL0_RD_EN);
	(void)PVR_CR_READ32(pvr_dev, META_SP_MSLVCTRL0); /* Fence write. */

	/* Wait for Slave Port to be Ready. */
	err = pvr_cr_poll_reg32(pvr_dev, ROGUE_CR_META_SP_MSLVCTRL1,
				ROGUE_CR_META_SP_MSLVCTRL1_READY_EN |
					ROGUE_CR_META_SP_MSLVCTRL1_GBLPORT_IDLE_EN,
				ROGUE_CR_META_SP_MSLVCTRL1_READY_EN |
					ROGUE_CR_META_SP_MSLVCTRL1_GBLPORT_IDLE_EN,
				POLL_TIMEOUT_USEC);
	if (err)
		goto err_out;

	*reg_value_out = PVR_CR_READ32(pvr_dev, META_SP_MSLVDATAX);

	return 0;

err_out:
	return err;
}

static int
pvr_meta_wrapper_init(struct pvr_device *pvr_dev)
{
	u64 garten_config;

	/* Configure META to Master boot. */
	PVR_CR_WRITE64(pvr_dev, META_BOOT, ROGUE_CR_META_BOOT_MODE_EN);

	/* Set Garten IDLE to META idle and Set the Garten Wrapper BIF Fence address. */

	/* Garten IDLE bit controlled by META. */
	garten_config = ROGUE_CR_MTS_GARTEN_WRAPPER_CONFIG_IDLE_CTRL_META;

	/* The fence addr is set during the fw init sequence. */

	/* Set PC = 0 for fences. */
	garten_config &=
		ROGUE_CR_MTS_GARTEN_WRAPPER_CONFIG_FENCE_PC_BASE_CLRMSK;
	garten_config |=
		(u64)MMU_CONTEXT_MAPPING_FWPRIV
		<< ROGUE_CR_MTS_GARTEN_WRAPPER_CONFIG_FENCE_PC_BASE_SHIFT;

	/* Set SLC DM=META. */
	garten_config |= ((u64)ROGUE_FW_SEGMMU_META_BIFDM_ID)
			 << ROGUE_CR_MTS_GARTEN_WRAPPER_CONFIG_FENCE_DM_SHIFT;

	PVR_CR_WRITE64(pvr_dev, MTS_GARTEN_WRAPPER_CONFIG, garten_config);

	return 0;
}

static __always_inline void
add_boot_arg(u32 **boot_conf, u32 param, u32 data)
{
	*(*boot_conf)++ = param;
	*(*boot_conf)++ = data;
}

static int
meta_ldr_cmd_loadmem(struct drm_device *drm_dev, const u8 *fw,
		     struct rogue_meta_ldr_l1_data_blk *l1_data,
		     u32 coremem_size,
		     const struct pvr_fw_layout_entry *layout_entries,
		     u32 num_layout_entries, u8 *fw_code_ptr, u8 *fw_data_ptr,
		     u8 *fw_core_code_ptr, u8 *fw_core_data_ptr,
		     const u32 fw_size)
{
	struct rogue_meta_ldr_l2_data_blk *l2_block =
		(struct rogue_meta_ldr_l2_data_blk *)(fw +
						      l1_data->cmd_data[1]);
	u32 offset = l1_data->cmd_data[0];
	u32 data_size;
	void *write_addr;
	int err;

	/* Verify header is within bounds. */
	if (((u8 *)l2_block - fw) >= fw_size || ((u8 *)(l2_block + 1) - fw) >= fw_size) {
		err = -EINVAL;
		goto err_out;
	}

	data_size = l2_block->length - 6 /* L2 Tag length and checksum */;

	/* Verify data is within bounds. */
	if (((u8 *)l2_block->block_data - fw) >= fw_size ||
	    ((((u8 *)l2_block->block_data) + data_size) - fw) >= fw_size) {
		err = -EINVAL;
		goto err_out;
	}

	if (!ROGUE_META_IS_COREMEM_CODE(offset, coremem_size) &&
	    !ROGUE_META_IS_COREMEM_DATA(offset, coremem_size)) {
		/* Global range is aliased to local range */
		offset &= ~META_MEM_GLOBAL_RANGE_BIT;
	}

	err = pvr_fw_find_mmu_segment(offset, data_size, layout_entries,
				      num_layout_entries, fw_code_ptr, fw_data_ptr,
				      fw_core_code_ptr, fw_core_data_ptr, &write_addr);
	if (err) {
		drm_err(drm_dev,
			"Addr 0x%x (size: %d) not found in any firmware segment",
			offset, data_size);
		goto err_out;
	}

	memcpy(write_addr, l2_block->block_data, data_size);

	return 0;

err_out:
	return err;
}

static int
meta_ldr_cmd_zeromem(struct drm_device *drm_dev,
		     struct rogue_meta_ldr_l1_data_blk *l1_data,
		     u32 coremem_size,
		     const struct pvr_fw_layout_entry *layout_entries,
		     u32 num_layout_entries, u8 *fw_code_ptr, u8 *fw_data_ptr,
		     u8 *fw_core_code_ptr, u8 *fw_core_data_ptr)
{
	u32 offset = l1_data->cmd_data[0];
	u32 byte_count = l1_data->cmd_data[1];
	void *write_addr;
	int err;

	if (ROGUE_META_IS_COREMEM_DATA(offset, coremem_size)) {
		/* cannot zero coremem directly */
		return 0;
	}

	/* Global range is aliased to local range */
	offset &= ~META_MEM_GLOBAL_RANGE_BIT;

	err = pvr_fw_find_mmu_segment(offset, byte_count, layout_entries,
				      num_layout_entries, fw_code_ptr, fw_data_ptr,
				      fw_core_code_ptr, fw_core_data_ptr, &write_addr);
	if (err) {
		drm_err(drm_dev,
			"Addr 0x%x (size: %d) not found in any firmware segment",
			offset, byte_count);
		goto err_out;
	}

	memset(write_addr, 0, byte_count);

	return 0;

err_out:
	return err;
}

static int
meta_ldr_cmd_config(struct drm_device *drm_dev, const u8 *fw,
		    struct rogue_meta_ldr_l1_data_blk *l1_data,
		    const u32 fw_size, u32 **boot_conf_ptr)
{
	struct rogue_meta_ldr_l2_data_blk *l2_block =
		(struct rogue_meta_ldr_l2_data_blk *)(fw +
						      l1_data->cmd_data[0]);
	struct rogue_meta_ldr_cfg_blk *config_command;
	u32 l2_block_size;
	u32 curr_block_size = 0;
	u32 *boot_conf = boot_conf_ptr ? *boot_conf_ptr : NULL;
	int err;

	/* Verify block header is within bounds. */
	if (((u8 *)l2_block - fw) >= fw_size || ((u8 *)(l2_block + 1) - fw) >= fw_size) {
		err = -EINVAL;
		goto err_out;
	}

	l2_block_size = l2_block->length - 6 /* L2 Tag length and checksum */;
	config_command = (struct rogue_meta_ldr_cfg_blk *)l2_block->block_data;

	if (((u8 *)config_command - fw) >= fw_size ||
	    ((((u8 *)config_command) + l2_block_size) - fw) >= fw_size) {
		err = -EINVAL;
		goto err_out;
	}

	while (l2_block_size >= 12) {
		if (config_command->type != ROGUE_META_LDR_CFG_WRITE) {
			err = -EINVAL;
			goto err_out;
		}

		/*
		 * Only write to bootloader if we got a valid pointer to the FW
		 * code allocation.
		 */
		if (boot_conf) {
			u32 register_offset = config_command->block_data[0];
			u32 register_value = config_command->block_data[1];

			/* Do register write */
			add_boot_arg(&boot_conf, register_offset,
				     register_value);
		}

		curr_block_size = 12;
		l2_block_size -= curr_block_size;
		config_command = (struct rogue_meta_ldr_cfg_blk
					  *)((uintptr_t)config_command +
					     curr_block_size);
	}

	if (boot_conf_ptr)
		*boot_conf_ptr = boot_conf;

	return 0;

err_out:
	return err;
}

/**
 * process_ldr_command_stream() - Process LDR firmware image and populate
 *                                firmware sections
 * @pvr_dev: Device pointer.
 * @fw: Pointer to firmware image.
 * @layout_entries: Pointer to layout table.
 * @num_layout_entries: Number of entries in layout table.
 * @fw_code_ptr: Pointer to FW code section.
 * @fw_data_ptr: Pointer to FW data section.
 * @fw_core_code_ptr: Pointer to FW coremem code section.
 * @fw_core_data_ptr: Pointer to FW coremem data section.
 * @boot_conf_ptr: Pointer to boot config argument pointer.
 *
 * Returns :
 *  * 0 on success, or
 *  * -EINVAL on any error in LDR command stream.
 */
static int
process_ldr_command_stream(struct pvr_device *pvr_dev, const u8 *fw,
			   const struct pvr_fw_layout_entry *layout_entries,
			   u32 num_layout_entries, u8 *fw_code_ptr,
			   u8 *fw_data_ptr, u8 *fw_core_code_ptr,
			   u8 *fw_core_data_ptr, u32 **boot_conf_ptr)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct rogue_meta_ldr_block_hdr *ldr_header =
		(struct rogue_meta_ldr_block_hdr *)fw;
	struct rogue_meta_ldr_l1_data_blk *l1_data =
		(struct rogue_meta_ldr_l1_data_blk *)(fw + ldr_header->sl_data);
	const u32 fw_size = pvr_dev->fw_dev.firmware->size;
	int err;

	u32 *boot_conf = boot_conf_ptr ? *boot_conf_ptr : NULL;
	u32 coremem_size;

	err = PVR_FEATURE_VALUE(pvr_dev, meta_coremem_size, &coremem_size);
	if (err)
		goto err_out;

	coremem_size *= SZ_1K;

	while (l1_data) {
		/* Verify block header is within bounds. */
		if (((u8 *)l1_data - fw) >= fw_size || ((u8 *)(l1_data + 1) - fw) >= fw_size) {
			err = -EINVAL;
			goto err_out;
		}

		if (ROGUE_META_LDR_BLK_IS_COMMENT(l1_data->cmd)) {
			/* Don't process comment blocks */
			goto next_block;
		}

		switch (l1_data->cmd & ROGUE_META_LDR_CMD_MASK)
		case ROGUE_META_LDR_CMD_LOADMEM: {
			err = meta_ldr_cmd_loadmem(drm_dev, fw, l1_data,
						   coremem_size, layout_entries,
						   num_layout_entries,
						   fw_code_ptr, fw_data_ptr,
						   fw_core_code_ptr,
						   fw_core_data_ptr, fw_size);
			if (err)
				goto err_out;
			break;

		case ROGUE_META_LDR_CMD_START_THREADS:
			/* Don't process this block */
			break;

		case ROGUE_META_LDR_CMD_ZEROMEM:
			err = meta_ldr_cmd_zeromem(drm_dev, l1_data,
						   coremem_size, layout_entries,
						   num_layout_entries,
						   fw_code_ptr, fw_data_ptr,
						   fw_core_code_ptr,
						   fw_core_data_ptr);
			break;

		case ROGUE_META_LDR_CMD_CONFIG:
			err = meta_ldr_cmd_config(drm_dev, fw, l1_data, fw_size,
						  &boot_conf);
			break;

		default:
			err = -EINVAL;
			goto err_out;
		}

next_block:
		if (l1_data->next == 0xFFFFFFFF)
			break;

		l1_data = (struct rogue_meta_ldr_l1_data_blk *)(fw +
								l1_data->next);
	}

	if (boot_conf_ptr)
		*boot_conf_ptr = boot_conf;

	return 0;

err_out:
	return err;
}

static void
configure_seg_id(u64 seg_out_addr, u32 seg_base, u32 seg_limit, u32 seg_id,
		 u32 **boot_conf_ptr)
{
	u32 seg_out_addr0 = seg_out_addr & 0x00000000FFFFFFFFUL;
	u32 seg_out_addr1 = (seg_out_addr >> 32) & 0x00000000FFFFFFFFUL;
	u32 *boot_conf = *boot_conf_ptr;

	/* META segments have a minimum size. */
	u32 limit_off = max(seg_limit, ROGUE_FW_SEGMMU_ALIGN);

	/* The limit is an offset, therefore off = size - 1. */
	limit_off -= 1;

	seg_base |= ROGUE_FW_SEGMMU_ALLTHRS_WRITEABLE;

	add_boot_arg(&boot_conf, META_CR_MMCU_SEGMENT_N_BASE(seg_id), seg_base);
	add_boot_arg(&boot_conf, META_CR_MMCU_SEGMENT_N_LIMIT(seg_id), limit_off);
	add_boot_arg(&boot_conf, META_CR_MMCU_SEGMENT_N_OUTA0(seg_id), seg_out_addr0);
	add_boot_arg(&boot_conf, META_CR_MMCU_SEGMENT_N_OUTA1(seg_id), seg_out_addr1);

	*boot_conf_ptr = boot_conf;
}

static void
configure_seg_mmu(struct pvr_device *pvr_dev,
		  const struct pvr_fw_layout_entry *layout_entries,
		  u32 num_layout_entries, u32 **boot_conf_ptr)
{
	u64 seg_out_addr_top;
	u32 i;

	seg_out_addr_top =
		ROGUE_FW_SEGMMU_OUTADDR_TOP_SLC(MMU_CONTEXT_MAPPING_FWPRIV,
						ROGUE_FW_SEGMMU_META_BIFDM_ID);

	for (i = 0; i < num_layout_entries; i++) {
		/*
		 * FW code is using the bootloader segment which is already
		 * configured on boot. FW coremem code and data don't use the
		 * segment MMU. Only the FW data segment needs to be configured.
		 */
		if (layout_entries[i].type == FW_DATA) {
			u32 seg_id = ROGUE_FW_SEGMMU_DATA_ID;
			u64 seg_out_addr;

			WARN_ON(!pvr_gem_get_fw_gpu_addr(pvr_dev->fw_dev.mem.data_obj,
							 &seg_out_addr));
			seg_out_addr += layout_entries[i].alloc_offset;
			seg_out_addr |= seg_out_addr_top;

			/* Write the sequence to the bootldr. */
			configure_seg_id(seg_out_addr,
					 layout_entries[i].base_addr,
					 layout_entries[i].alloc_size, seg_id,
					 boot_conf_ptr);

			break;
		}
	}
}

static void
configure_meta_caches(u32 **boot_conf_ptr)
{
	u32 *boot_conf = *boot_conf_ptr;
	u32 d_cache_t0, i_cache_t0;
	u32 d_cache_t1, i_cache_t1;
	u32 d_cache_t2, i_cache_t2;
	u32 d_cache_t3, i_cache_t3;

	/* Initialise I/Dcache settings */
	d_cache_t0 = META_CR_SYSC_DCPARTX_CACHED_WRITE_ENABLE;
	d_cache_t1 = META_CR_SYSC_DCPARTX_CACHED_WRITE_ENABLE;
	d_cache_t2 = META_CR_SYSC_DCPARTX_CACHED_WRITE_ENABLE;
	d_cache_t3 = META_CR_SYSC_DCPARTX_CACHED_WRITE_ENABLE;
	i_cache_t0 = 0;
	i_cache_t1 = 0;
	i_cache_t2 = 0;
	i_cache_t3 = 0;

	d_cache_t0 |= META_CR_SYSC_XCPARTX_LOCAL_ADDR_FULL_CACHE;
	i_cache_t0 |= META_CR_SYSC_XCPARTX_LOCAL_ADDR_FULL_CACHE;

	/* Local region MMU enhanced bypass: WIN-3 mode for code and data caches */
	add_boot_arg(&boot_conf, META_CR_MMCU_LOCAL_EBCTRL,
		     META_CR_MMCU_LOCAL_EBCTRL_ICWIN |
			     META_CR_MMCU_LOCAL_EBCTRL_DCWIN);

	/* Data cache partitioning thread 0 to 3 */
	add_boot_arg(&boot_conf, META_CR_SYSC_DCPART(0), d_cache_t0);
	add_boot_arg(&boot_conf, META_CR_SYSC_DCPART(1), d_cache_t1);
	add_boot_arg(&boot_conf, META_CR_SYSC_DCPART(2), d_cache_t2);
	add_boot_arg(&boot_conf, META_CR_SYSC_DCPART(3), d_cache_t3);

	/* Enable data cache hits */
	add_boot_arg(&boot_conf, META_CR_MMCU_DCACHE_CTRL,
		     META_CR_MMCU_XCACHE_CTRL_CACHE_HITS_EN);

	/* Instruction cache partitioning thread 0 to 3 */
	add_boot_arg(&boot_conf, META_CR_SYSC_ICPART(0), i_cache_t0);
	add_boot_arg(&boot_conf, META_CR_SYSC_ICPART(1), i_cache_t1);
	add_boot_arg(&boot_conf, META_CR_SYSC_ICPART(2), i_cache_t2);
	add_boot_arg(&boot_conf, META_CR_SYSC_ICPART(3), i_cache_t3);

	/* Enable instruction cache hits */
	add_boot_arg(&boot_conf, META_CR_MMCU_ICACHE_CTRL,
		     META_CR_MMCU_XCACHE_CTRL_CACHE_HITS_EN);

	add_boot_arg(&boot_conf, 0x040000C0, 0);

	*boot_conf_ptr = boot_conf;
}

static int
pvr_meta_fw_process(struct pvr_device *pvr_dev, const u8 *fw,
		    const struct pvr_fw_layout_entry *layout_entries, u32 num_layout_entries,
		    u8 *fw_code_ptr, u8 *fw_data_ptr, u8 *fw_core_code_ptr, u8 *fw_core_data_ptr,
		    u32 core_code_alloc_size)
{
	struct pvr_fw_device *fw_dev = &pvr_dev->fw_dev;
	u32 *boot_conf;
	int err;

	boot_conf = ((u32 *)fw_code_ptr) + ROGUE_FW_BOOTLDR_CONF_OFFSET;

	/* Slave port and JTAG accesses are privileged. */
	add_boot_arg(&boot_conf, META_CR_SYSC_JTAG_THREAD,
		     META_CR_SYSC_JTAG_THREAD_PRIV_EN);

	configure_seg_mmu(pvr_dev, layout_entries, num_layout_entries, &boot_conf);

	/* Populate FW sections from LDR image. */
	err = process_ldr_command_stream(pvr_dev, fw, layout_entries, num_layout_entries,
					 fw_code_ptr, fw_data_ptr, fw_core_code_ptr,
					 fw_core_data_ptr, &boot_conf);
	if (err)
		goto err_out;

	configure_meta_caches(&boot_conf);

	/* End argument list. */
	add_boot_arg(&boot_conf, 0, 0);

	if (fw_dev->mem.core_code_obj) {
		u32 core_code_fw_addr;

		pvr_gem_get_fw_addr(fw_dev->mem.core_code_obj, &core_code_fw_addr);
		add_boot_arg(&boot_conf, core_code_fw_addr, core_code_alloc_size);
	} else {
		add_boot_arg(&boot_conf, 0, 0);
	}
	/* None of the cores supported by this driver have META DMA. */
	add_boot_arg(&boot_conf, 0, 0);

	return 0;

err_out:
	return err;
}

static int
pvr_meta_init(struct pvr_device *pvr_dev)
{
	pvr_fw_heap_info_init(pvr_dev, ROGUE_FW_HEAP_META_SHIFT, 0);

	return 0;
}

static u32
pvr_meta_get_fw_addr_with_offset(struct pvr_fw_object *fw_obj, u32 offset)
{
	u32 fw_addr = fw_obj->fw_addr_offset + offset + ROGUE_FW_SEGMMU_DATA_BASE_ADDRESS;

	/* META cacheability is determined by address. */
	if (fw_obj->base.flags & PVR_BO_FW_FLAGS_DEVICE_UNCACHED)
		fw_addr |= ROGUE_FW_SEGMMU_DATA_META_UNCACHED |
			   ROGUE_FW_SEGMMU_DATA_VIVT_SLC_UNCACHED;

	return fw_addr;
}

static int
pvr_meta_vm_map(struct pvr_device *pvr_dev, struct pvr_fw_object *fw_obj)
{
	struct pvr_gem_object *pvr_obj = from_pvr_fw_object(fw_obj);

	return pvr_vm_map(pvr_dev->kernel_vm_ctx, pvr_obj, 0, fw_obj->fw_mm_node.start,
			  pvr_gem_object_size(pvr_obj));
}

static void
pvr_meta_vm_unmap(struct pvr_device *pvr_dev, struct pvr_fw_object *fw_obj)
{
	pvr_vm_unmap(pvr_dev->kernel_vm_ctx, fw_obj->fw_mm_node.start);
}

static bool
pvr_meta_check_and_ack_irq(struct pvr_device *pvr_dev)
{
	u32 irq_status = PVR_CR_READ32(pvr_dev, META_SP_MSLVIRQSTATUS);

	if (!(irq_status & ROGUE_CR_META_SP_MSLVIRQSTATUS_TRIGVECT2_EN))
		return false; /* Spurious IRQ - ignore. */

	/* Acknowledge IRQ. */
	PVR_CR_WRITE32(pvr_dev, META_SP_MSLVIRQSTATUS,
		       ROGUE_CR_META_SP_MSLVIRQSTATUS_TRIGVECT2_CLRMSK);

	return true;
}

static bool
pvr_meta_has_fixed_data_addr(void)
{
	return false;
}

const struct pvr_fw_funcs pvr_fw_funcs_meta = {
	.init = pvr_meta_init,
	.fw_process = pvr_meta_fw_process,
	.vm_map = pvr_meta_vm_map,
	.vm_unmap = pvr_meta_vm_unmap,
	.get_fw_addr_with_offset = pvr_meta_get_fw_addr_with_offset,
	.wrapper_init = pvr_meta_wrapper_init,
	.check_and_ack_irq = pvr_meta_check_and_ack_irq,
	.has_fixed_data_addr = pvr_meta_has_fixed_data_addr,
};
