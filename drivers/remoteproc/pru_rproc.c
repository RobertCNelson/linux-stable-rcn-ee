// SPDX-License-Identifier: GPL-2.0
/*
 * PRU-ICSS remoteproc driver for various TI SoCs
 *
 * Copyright (C) 2014-2019 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 *	Andrew F. Davis <afd@ti.com>
 */

#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/irqchip/irq-pruss-intc.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/remoteproc/pru_rproc.h>
#include <linux/pruss.h>

#include "remoteproc_internal.h"
#include "pru_rproc.h"

/* ELF PAGE ID */
#define PRU_PAGE_IRAM	0
#define PRU_PAGE_DRAM	1

/* IRAM offsets */
#define PRU0_IRAM_START		0x34000
#define PRU1_IRAM_START		0x38000
#define PRU_IRAM_MASK		0x3ffff

/* PRU_ICSS_PRU_CTRL registers */
#define PRU_CTRL_CTRL		0x0000
#define PRU_CTRL_STS		0x0004
#define PRU_CTRL_WAKEUP_EN	0x0008
#define PRU_CTRL_CYCLE		0x000C
#define PRU_CTRL_STALL		0x0010
#define PRU_CTRL_CTBIR0		0x0020
#define PRU_CTRL_CTBIR1		0x0024
#define PRU_CTRL_CTPPR0		0x0028
#define PRU_CTRL_CTPPR1		0x002C

/* CTRL register bit-fields */
#define CTRL_CTRL_SOFT_RST_N	BIT(0)
#define CTRL_CTRL_EN		BIT(1)
#define CTRL_CTRL_SLEEPING	BIT(2)
#define CTRL_CTRL_CTR_EN	BIT(3)
#define CTRL_CTRL_SINGLE_STEP	BIT(8)
#define CTRL_CTRL_RUNSTATE	BIT(15)

/* PRU_ICSS_PRU_DEBUG registers */
#define PRU_DEBUG_GPREG(x)	(0x0000 + (x) * 4)
#define PRU_DEBUG_CT_REG(x)	(0x0080 + (x) * 4)

/**
 * struct pru_rproc - PRU remoteproc structure
 * @id: id of the PRU core within the PRUSS
 * @pruss: back-reference to parent PRUSS structure
 * @rproc: remoteproc pointer for this PRU core
 * @iram_region: PRU IRAM IOMEM
 * @ctrl_regmap: regmap to PRU CTRL IOMEM
 * @debug_regmap: regmap to PRU DEBUG IOMEM
 * @client_np: client device node
 * @intc_config: PRU INTC configuration data
 * @dram0: PRUSS DRAM0 region
 * @dram1: PRUSS DRAM1 region
 * @shrdram: PRUSS SHARED RAM region
 * @iram_da: device address of Instruction RAM for this PRU
 * @pdram_da: device address of primary Data RAM for this PRU
 * @sdram_da: device address of secondary Data RAM for this PRU
 * @shrdram_da: device address of shared Data RAM
 * @cfg: regmap to CFG module
 * @gpcfg_reg: offset to gpcfg register of this PRU
 * @fw_name: name of firmware image used during loading
 * @gpmux_save: saved value for gpmux config
 * @dt_irqs: number of irqs configured from DT
 * @fw_irqs: number of irqs configured from FW
 * @lock: mutex to protect client usage
 * @dbg_single_step: debug state variable to set PRU into single step mode
 * @ctrl_saved_state: saved CTRL state to return to normal mode
 */
struct pru_rproc {
	int id;
	struct pruss *pruss;
	struct rproc *rproc;
	struct pruss_mem_region iram_region;
	struct regmap *ctrl_regmap;
	struct regmap *debug_regmap;
	struct device_node *client_np;
	struct pruss_intc_config intc_config;
	struct pruss_mem_region dram0;
	struct pruss_mem_region dram1;
	struct pruss_mem_region shrdram;
	u32 iram_da;
	u32 pdram_da;
	u32 sdram_da;
	u32 shrdram_da;
	struct regmap *cfg;
	int gpcfg_reg;
	const char *fw_name;
	u8 gpmux_save;
	int dt_irqs;
	int fw_irqs;
	struct mutex lock; /* client access lock */
	bool dbg_single_step;
	u32 ctrl_saved_state;
};

/**
 * pru_rproc_set_firmware() - set firmware for a pru core
 * @rproc: the rproc instance of the PRU
 * @fw_name: the new firmware name, or NULL if default is desired
 */
static int pru_rproc_set_firmware(struct rproc *rproc, const char *fw_name)
{
	struct pru_rproc *pru = rproc->priv;

	if (!fw_name)
		fw_name = pru->fw_name;

	return rproc_set_firmware(rproc, fw_name);
}

/*
 * pru_get_gpmux() - get the current GPMUX value for a PRU device
 * @pru: pru_rproc instance
 * @mux: pointer to store the current mux value into
 *
 * returns 0 on success, negative error value otherwise.
 */
static int pru_get_gpmux(struct pru_rproc *pru, u8 *mux)
{
	int ret;
	unsigned int val;

	ret = regmap_read(pru->cfg, pru->gpcfg_reg, &val);
	if (ret)
		return ret;

	*mux = (u8)((val & PRUSS_GPCFG_PRU_MUX_SEL_MASK) >>
		    PRUSS_GPCFG_PRU_MUX_SEL_SHIFT);

	return 0;
}

/**
 * pru_set_gpmux() - set the GPMUX value for a PRU device
 * @pru: pru_rproc instance
 * @mux: new mux value for PRU
 *
 * returns 0 on success, negative error value otherwise.
 */
static int pru_set_gpmux(struct pru_rproc *pru, u8 mux)
{
	if (mux >= PRUSS_GP_MUX_SEL_MAX)
		return -EINVAL;

	return regmap_update_bits(pru->cfg, pru->gpcfg_reg,
				  PRUSS_GPCFG_PRU_MUX_SEL_MASK,
				  (u32)mux << PRUSS_GPCFG_PRU_MUX_SEL_SHIFT);
}

static int pru_get_intc_dt_config(struct pru_rproc *pru,
				  const char *propname, int index,
				  struct pruss_intc_config *intc_config)
{
	struct device *dev = &pru->rproc->dev;
	struct device_node *np = pru->client_np;
	struct property *prop;
	int ret = 0, entries, i;
	int dt_irqs = 0;
	u32 *arr;
	int max_system_events, max_pru_channels, max_pru_host_ints;

	max_system_events = MAX_PRU_SYS_EVENTS;
	max_pru_channels = MAX_PRU_CHANNELS;
	max_pru_host_ints = MAX_PRU_CHANNELS;

	prop = of_find_property(np, propname, NULL);
	if (!prop)
		return 0;

	entries = of_property_count_u32_elems(np, propname);
	if (entries <= 0 || entries % 4)
		return -EINVAL;

	arr = kmalloc_array(entries, sizeof(u32), GFP_KERNEL);
	if (!arr)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, propname, arr, entries);
	if (ret)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(intc_config->sysev_to_ch); i++)
		intc_config->sysev_to_ch[i] = -1;

	for (i = 0; i < ARRAY_SIZE(intc_config->ch_to_host); i++)
		intc_config->ch_to_host[i] = -1;

	for (i = 0; i < entries; i += 4) {
		if (arr[i] != index)
			continue;

		if (arr[i + 1] < 0 ||
		    arr[i + 1] >= max_system_events) {
			dev_err(dev, "bad sys event %d\n", arr[i + 1]);
			ret = -EINVAL;
			goto err;
		}

		if (arr[i + 2] < 0 ||
		    arr[i + 2] >= max_pru_channels) {
			dev_err(dev, "bad channel %d\n", arr[i + 2]);
			ret = -EINVAL;
			goto err;
		}

		if (arr[i + 3] < 0 ||
		    arr[i + 3] >= max_pru_host_ints) {
			dev_err(dev, "bad irq %d\n", arr[i + 3]);
			ret = -EINVAL;
			goto err;
		}

		intc_config->sysev_to_ch[arr[i + 1]] = arr[i + 2];
		dev_dbg(dev, "sysevt-to-ch[%d] -> %d\n", arr[i + 1],
			arr[i + 2]);

		intc_config->ch_to_host[arr[i + 2]] = arr[i + 3];
		dev_dbg(dev, "chnl-to-host[%d] -> %d\n", arr[i + 2],
			arr[i + 3]);

		dt_irqs++;
	}

	kfree(arr);
	return dt_irqs;

err:
	kfree(arr);
	return ret;
}

static struct rproc *__pru_rproc_get(struct device_node *np, int index)
{
	struct device_node *rproc_np = NULL;
	struct platform_device *pdev;
	struct rproc *rproc;

	rproc_np = of_parse_phandle(np, "prus", index);
	if (!rproc_np || !of_device_is_available(rproc_np))
		return ERR_PTR(-ENODEV);

	pdev = of_find_device_by_node(rproc_np);
	of_node_put(rproc_np);

	if (!pdev)
		/* probably PRU not yet probed */
		return ERR_PTR(-EPROBE_DEFER);

	/* TODO: replace the crude string based check to make sure it is PRU */
	if (!strstr(dev_name(&pdev->dev), "pru")) {
		put_device(&pdev->dev);
		return ERR_PTR(-ENODEV);
	}

	rproc = platform_get_drvdata(pdev);
	put_device(&pdev->dev);
	if (!rproc)
		return ERR_PTR(-EPROBE_DEFER);

	get_device(&rproc->dev);

	return rproc;
}

/**
 * pru_rproc_get() - get the PRU rproc instance from a device node
 * @np: the user/client device node
 * @index: index to use for the prus property
 *
 * This function looks through a client device node's "prus" property at index
 * @index and returns the rproc handle for a valid PRU remote processor if
 * found. The function allows only one user to own the PRU rproc resource at
 * a time. Caller must call pru_rproc_put() when done with using the rproc,
 * not required if the function returns a failure.
 *
 * Returns the rproc handle on success, and an ERR_PTR on failure using one
 * of the following error values
 *    -ENODEV if device is not found
 *    -EBUSY if PRU is already acquired by anyone
 *    -EPROBE_DEFER is PRU device is not probed yet
 */
struct rproc *pru_rproc_get(struct device_node *np, int index)
{
	struct rproc *rproc;
	struct pru_rproc *pru;
	struct device *dev;
	int ret;
	u32 mux;
	const char *fw_name;

	rproc = __pru_rproc_get(np, index);
	if (IS_ERR(rproc))
		return rproc;

	pru = rproc->priv;
	dev = &rproc->dev;

	mutex_lock(&pru->lock);

	if (pru->client_np) {
		mutex_unlock(&pru->lock);
		put_device(&rproc->dev);
		return ERR_PTR(-EBUSY);
	}

	pru->client_np = np;

	mutex_unlock(&pru->lock);

	ret = pru_get_gpmux(pru, &pru->gpmux_save);
	if (ret) {
		dev_err(dev, "failed to get cfg gpmux: %d\n", ret);
		goto err;
	}

	ret = of_property_read_u32_index(np, "ti,pruss-gp-mux-sel", index,
					 &mux);
	if (!ret) {
		ret = pru_set_gpmux(pru, mux);
		if (ret) {
			dev_err(dev, "failed to set cfg gpmux: %d\n", ret);
			goto err;
		}
	}

	ret = of_property_read_string_index(np, "firmware-name", index,
					    &fw_name);
	if (!ret) {
		ret = pru_rproc_set_firmware(rproc, fw_name);
		if (ret) {
			dev_err(dev, "failed to set firmware: %d\n", ret);
			goto err;
		}
	}

	ret = pru_get_intc_dt_config(pru, "ti,pru-interrupt-map",
				     index, &pru->intc_config);
	if (ret < 0) {
		dev_err(dev, "error getting DT interrupt map: %d\n", ret);
		goto err;
	}

	pru->dt_irqs = ret;

	return rproc;

err:
	pru_rproc_put(rproc);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(pru_rproc_get);

/**
 * pru_rproc_put() - release the PRU rproc resource
 * @rproc: the rproc resource to release
 *
 * Releases the PRU rproc resource and makes it available to other
 * users.
 */
void pru_rproc_put(struct rproc *rproc)
{
	struct pru_rproc *pru;

	if (IS_ERR_OR_NULL(rproc))
		return;

	/* TODO: replace the crude string based check to make sure it is PRU */
	if (!strstr(dev_name(rproc->dev.parent), "pru"))
		return;

	pru = rproc->priv;
	if (!pru->client_np)
		return;

	pru_rproc_set_firmware(rproc, NULL);
	pru_set_gpmux(pru, pru->gpmux_save);

	mutex_lock(&pru->lock);
	pru->client_np = NULL;
	mutex_unlock(&pru->lock);

	put_device(&rproc->dev);
}
EXPORT_SYMBOL_GPL(pru_rproc_put);

/*
 * Control PRU single-step mode
 *
 * This is a debug helper function used for controlling the single-step
 * mode of the PRU. The PRU Debug registers are not accessible when the
 * PRU is in RUNNING state.
 *
 * Writing a non-zero value sets the PRU into single-step mode irrespective
 * of its previous state. The PRU mode is saved only on the first set into
 * a single-step mode. Writing a zero value will restore the PRU into its
 * original mode.
 */
static int pru_rproc_debug_ss_set(void *data, u64 val)
{
	struct rproc *rproc = data;
	struct pru_rproc *pru = rproc->priv;
	u32 reg_val;
	bool debug_ss;

	debug_ss = !!val;
	if (!debug_ss && !pru->dbg_single_step)
		return 0;

	regmap_read(pru->ctrl_regmap, PRU_CTRL_CTRL, &reg_val);

	if (debug_ss && !pru->dbg_single_step)
		pru->ctrl_saved_state = reg_val;

	if (debug_ss)
		reg_val |= CTRL_CTRL_SINGLE_STEP | CTRL_CTRL_EN;
	else
		reg_val = pru->ctrl_saved_state;

	pru->dbg_single_step = debug_ss;
	regmap_write(pru->ctrl_regmap, PRU_CTRL_CTRL, reg_val);

	return 0;
}

static int pru_rproc_debug_ss_get(void *data, u64 *val)
{
	struct rproc *rproc = data;
	struct pru_rproc *pru = rproc->priv;

	*val = pru->dbg_single_step;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(pru_rproc_debug_ss_fops, pru_rproc_debug_ss_get,
			pru_rproc_debug_ss_set, "%llu\n");

/*
 * Create PRU-specific debugfs entries
 *
 * The entries are created only if the parent remoteproc debugfs directory
 * exists, and will be cleaned up by the remoteproc core.
 */
static void pru_rproc_create_debug_entries(struct rproc *rproc)
{
	if (!rproc->dbg_dir)
		return;

	debugfs_create_file("single_step", 0600, rproc->dbg_dir,
			    rproc, &pru_rproc_debug_ss_fops);
}

static void *pru_d_da_to_va(struct pru_rproc *pru, u32 da, int len);

/*
 * parse the custom interrupt map resource and save the intc_config
 * for use when booting the processor.
 */
static int pru_handle_vendor_intrmap(struct rproc *rproc,
				     struct fw_rsc_pruss_intrmap *rsc)
{
	int fw_irqs = 0, i, ret = 0;
	u8 *arr;
	struct device *dev = &rproc->dev;
	struct pru_rproc *pru = rproc->priv;

	dev_dbg(dev, "vendor rsc intc: version %d\n", rsc->version);

	/*
	 * 0 was prototyping version. Not supported.
	 * 1 is currently supported version.
	 */
	if (rsc->version == 0 || rsc->version > 1) {
		dev_err(dev, "Unsupported version %d\n", rsc->version);
		return -EINVAL;
	}

	/* DT provided INTC config takes precedence */
	if (pru->dt_irqs) {
		dev_info(dev, "INTC config in DT and FW. Using DT config.\n");
		return 0;
	}

	arr = rsc->data;

	for (i = 0; i < ARRAY_SIZE(pru->intc_config.sysev_to_ch); i++)
		pru->intc_config.sysev_to_ch[i] = -1;

	for (i = 0; i < ARRAY_SIZE(pru->intc_config.ch_to_host); i++)
		pru->intc_config.ch_to_host[i] = -1;

	for (i = 0; i < rsc->num_maps * 3; i += 3) {
		if (arr[i] < 0 ||
		    arr[i] >= MAX_PRU_SYS_EVENTS) {
			dev_err(dev, "bad sys event %d\n", arr[i]);
			ret = -EINVAL;
			goto err;
		}

		if (arr[i + 1] < 0 ||
		    arr[i + 1] >= MAX_PRU_CHANNELS) {
			dev_err(dev, "bad channel %d\n", arr[i + 1]);
			ret = -EINVAL;
			goto err;
		}

		if (arr[i + 2] < 0 ||
		    arr[i + 2] >= MAX_PRU_CHANNELS) {
			dev_err(dev, "bad host irq %d\n", arr[i + 2]);
				ret = -EINVAL;
			goto err;
		}

		pru->intc_config.sysev_to_ch[arr[i]] = arr[i + 1];
		dev_dbg(dev, "sysevt-to-ch[%d] -> %d\n", arr[i],
			arr[i + 1]);

		pru->intc_config.ch_to_host[arr[i + 1]] = arr[i + 2];
		dev_dbg(dev, "chnl-to-host[%d] -> %d\n", arr[i + 1],
			arr[i + 2]);

		fw_irqs++;
	}

	pru->fw_irqs = fw_irqs;
	return 0;

err:
	pru->fw_irqs = 0;
	return ret;
}

/* PRU-specific vendor resource handler */
static int pru_rproc_handle_vendor_rsc(struct rproc *rproc,
				       struct fw_rsc_vendor *ven_rsc)
{
	struct device *dev = rproc->dev.parent;
	int ret = -EINVAL;

	struct fw_rsc_pruss_intrmap *rsc;

	rsc = (struct fw_rsc_pruss_intrmap *)ven_rsc->data;

	switch (rsc->type) {
	case PRUSS_RSC_INTRS:
		ret = pru_handle_vendor_intrmap(rproc, rsc);
		break;
	default:
		dev_err(dev, "%s: cannot handle unknown type %d\n", __func__,
			rsc->type);
	}

	return ret;
}

/* start a PRU core */
static int pru_rproc_start(struct rproc *rproc)
{
	struct device *dev = &rproc->dev;
	struct pru_rproc *pru = rproc->priv;
	u32 val;
	int ret;

	dev_dbg(dev, "starting PRU%d: entry-point = 0x%x\n",
		pru->id, (rproc->bootaddr >> 2));

	if (pru->dt_irqs || pru->fw_irqs) {
		ret = pruss_intc_configure(rproc->dev.parent,
					   &pru->intc_config);
		if (ret) {
			dev_err(dev, "failed to configure intc %d\n", ret);
			return ret;
		}
	}

	val = CTRL_CTRL_EN | ((rproc->bootaddr >> 2) << 16);
	regmap_write(pru->ctrl_regmap, PRU_CTRL_CTRL, val);

	return 0;
}

/* stop/disable a PRU core */
static int pru_rproc_stop(struct rproc *rproc)
{
	struct device *dev = &rproc->dev;
	struct pru_rproc *pru = rproc->priv;

	dev_dbg(dev, "stopping PRU%d\n", pru->id);
	regmap_update_bits(pru->ctrl_regmap, PRU_CTRL_CTRL, CTRL_CTRL_EN, 0);

	/* undo INTC config */
	if (pru->dt_irqs || pru->fw_irqs)
		pruss_intc_unconfigure(rproc->dev.parent, &pru->intc_config);

	return 0;
}

/*
 * Convert PRU device address (data spaces only) to kernel virtual address
 *
 * Each PRU has access to all data memories within the PRUSS, accessible at
 * different ranges. So, look through both its primary and secondary Data
 * RAMs as well as any shared Data RAM to convert a PRU device address to
 * kernel virtual address. Data RAM0 is primary Data RAM for PRU0 and Data
 * RAM1 is primary Data RAM for PRU1.
 */
static void *pru_d_da_to_va(struct pru_rproc *pru, u32 da, int len)
{
	struct pruss_mem_region dram0, dram1, shrd_ram;
	u32 offset;
	void *va = NULL;

	if (len <= 0)
		return NULL;

	dram0 = pru->dram0;
	dram1 = pru->dram1;
	/* PRU1 has its local RAM addresses reversed */
	if (pru->id == 1)
		swap(dram0, dram1);
	shrd_ram = pru->shrdram;

	if (da >= pru->pdram_da && da + len <= pru->pdram_da + dram0.size) {
		offset = da - pru->pdram_da;
		va = (__force void *)(dram0.va + offset);
	} else if (da >= pru->sdram_da &&
		   da + len <= pru->sdram_da + dram1.size) {
		offset = da - pru->sdram_da;
		va = (__force void *)(dram1.va + offset);
	} else if (da >= pru->shrdram_da &&
		   da + len <= pru->shrdram_da + shrd_ram.size) {
		offset = da - pru->shrdram_da;
		va = (__force void *)(shrd_ram.va + offset);
	}

	return va;
}

/*
 * Convert PRU device address (instruction space) to kernel virtual address
 *
 * A PRU does not have an unified address space. Each PRU has its very own
 * private Instruction RAM, and its device address is identical to that of
 * its primary Data RAM device address.
 */
static void *pru_i_da_to_va(struct pru_rproc *pru, u32 da, int len)
{
	u32 offset;
	void *va = NULL;

	if (len <= 0)
		return NULL;

	if (da >= pru->iram_da &&
	    da + len <= pru->iram_da + pru->iram_region.size) {
		offset = da - pru->iram_da;
		va = (__force void *)(pru->iram_region.va + offset);
	}

	return va;
}

/* PRU-specific address translator */
static void *pru_da_to_va(struct rproc *rproc, u64 da, int len, int map)
{
	struct pru_rproc *pru = rproc->priv;
	void *va;

	switch (map) {
	case PRU_PAGE_IRAM:
		va = pru_i_da_to_va(pru, da, len);
		break;
	case PRU_PAGE_DRAM:
		va = pru_d_da_to_va(pru, da, len);
		break;
	default:
		dev_info(&rproc->dev, "%s: invalid page %d\n", __func__, map);
		return 0;
	};

	return va;
}

static struct rproc_ops pru_rproc_ops = {
	.start			= pru_rproc_start,
	.stop			= pru_rproc_stop,
	.da_to_va		= pru_da_to_va,
	.handle_vendor_rsc	= pru_rproc_handle_vendor_rsc,
};

static int pru_rproc_set_id(struct pru_rproc *pru)
{
	int ret = 0;
	u32 iram_start = pru->iram_region.pa & PRU_IRAM_MASK;

	if (iram_start == PRU0_IRAM_START)
		pru->id = 0;
	else if (iram_start == PRU1_IRAM_START)
		pru->id = 1;
	else
		ret = -EINVAL;

	return ret;
}

static struct regmap_config pru_ctrl_regmap_config = {
	.name = "ctrl",
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = PRU_CTRL_CTPPR1,
};

static struct regmap_config pru_debug_regmap_config = {
	.name = "debug",
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = PRU_DEBUG_CT_REG(31),
};

static int pru_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct platform_device *ppdev = to_platform_device(dev->parent);
	struct pru_rproc *pru;
	const char *fw_name;
	struct rproc *rproc = NULL;
	struct resource *res;
	int ret;
	void __iomem *va;

	if (!np) {
		dev_err(dev, "Non-DT platform device not supported\n");
		return -ENODEV;
	}

	ret = of_property_read_string(np, "firmware-name", &fw_name);
	if (ret) {
		dev_err(dev, "unable to retrieve firmware-name %d\n", ret);
		return ret;
	}

	rproc = rproc_alloc(dev, pdev->name, &pru_rproc_ops, fw_name,
			    sizeof(*pru));
	if (!rproc) {
		dev_err(dev, "rproc_alloc failed\n");
		return -ENOMEM;
	}

	pru = rproc->priv;
	pru->cfg = syscon_regmap_lookup_by_phandle(np, "gpcfg");
	if (IS_ERR(pru->cfg)) {
		if (PTR_ERR(pru->cfg) != -EPROBE_DEFER)
			dev_err(dev, "can't get gpcfg syscon regmap\n");

		return PTR_ERR(pru->cfg);
	}

	if (of_property_read_u32_index(np, "gpcfg", 1, &pru->gpcfg_reg)) {
		dev_err(dev, "couldn't get gpcfg reg. offset\n");
		return -EINVAL;
	}

	/* error recovery is not supported for PRUs */
	rproc->recovery_disabled = true;

	/*
	 * rproc_add will auto-boot the processor normally, but this is
	 * not desired with PRU client driven boot-flow methodology. A PRU
	 * application/client driver will boot the corresponding PRU
	 * remote-processor as part of its state machine either through
	 * the remoteproc sysfs interface or through the equivalent kernel API
	 */
	rproc->auto_boot = false;

	pru->pruss = platform_get_drvdata(ppdev);
	pru->rproc = rproc;
	pru->fw_name = fw_name;
	mutex_init(&pru->lock);

	ret = pruss_request_mem_region(pru->pruss, PRUSS_MEM_DRAM0,
				       &pru->dram0);
	if (ret) {
		dev_err(dev, "couldn't get PRUSS DRAM0: %d\n", ret);
		return ret;
	}
	pruss_release_mem_region(pru->pruss, &pru->dram0);

	ret = pruss_request_mem_region(pru->pruss, PRUSS_MEM_DRAM1,
				       &pru->dram1);
	if (ret) {
		dev_err(dev, "couldn't get PRUSS DRAM1: %d\n", ret);
		return ret;
	}
	pruss_release_mem_region(pru->pruss, &pru->dram1);

	ret = pruss_request_mem_region(pru->pruss, PRUSS_MEM_SHRD_RAM2,
				       &pru->shrdram);
	if (ret) {
		dev_err(dev, "couldn't get PRUSS Shared RAM: %d\n", ret);
		return ret;
	}
	pruss_release_mem_region(pru->pruss, &pru->shrdram);

	/* XXX: get this from match data if different in the future */
	pru->iram_da = 0;
	pru->pdram_da = 0;
	pru->sdram_da = 0x2000;
	pru->shrdram_da = 0x10000;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "iram");
	pru->iram_region.va = devm_ioremap_resource(dev, res);
	if (IS_ERR(pru->iram_region.va)) {
		ret = PTR_ERR(pru->iram_region.va);
		dev_err(dev, "failed to get iram resource: %d\n", ret);
		goto free_rproc;
	}

	pru->iram_region.pa = res->start;
	pru->iram_region.size = resource_size(res);

	dev_dbg(dev, "iram: pa %pa size 0x%zx va %p\n",
		&pru->iram_region.pa, pru->iram_region.size,
		pru->iram_region.va);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "control");
	va = devm_ioremap_resource(dev, res);
	if (IS_ERR(va)) {
		ret = PTR_ERR(va);
		dev_err(dev, "failed to get control resource: %d\n", ret);
		goto free_rproc;
	}

	pru->ctrl_regmap = devm_regmap_init_mmio(dev, va,
						 &pru_ctrl_regmap_config);
	if (IS_ERR(pru->ctrl_regmap)) {
		ret = PTR_ERR(pru->ctrl_regmap);
		dev_err(dev, "CTRL regmap init failed: %d\n", ret);
		goto free_rproc;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "debug");
	va = devm_ioremap_resource(dev, res);
	if (IS_ERR(va)) {
		ret = PTR_ERR(va);
		dev_err(dev, "failed to get debug resource: %d\n", ret);
		goto free_rproc;
	}
	pru->debug_regmap = devm_regmap_init_mmio(dev, va,
						  &pru_debug_regmap_config);
	if (IS_ERR(pru->debug_regmap)) {
		ret = PTR_ERR(pru->debug_regmap);
		dev_err(dev, "DEBUG regmap init failed: %d\n", ret);
		goto free_rproc;
	}

	ret = pru_rproc_set_id(pru);
	if (ret < 0)
		goto free_rproc;

	platform_set_drvdata(pdev, rproc);

	ret = rproc_add(pru->rproc);
	if (ret) {
		dev_err(dev, "rproc_add failed: %d\n", ret);
		goto free_rproc;
	}

	pru_rproc_create_debug_entries(rproc);

	dev_info(dev, "PRU rproc node %s probed successfully\n", np->full_name);

	return 0;

free_rproc:
	rproc_free(rproc);
	return ret;
}

static int pru_rproc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rproc *rproc = platform_get_drvdata(pdev);

	dev_info(dev, "%s: removing rproc %s\n", __func__, rproc->name);

	rproc_del(rproc);
	rproc_free(rproc);

	return 0;
}

static const struct of_device_id pru_rproc_match[] = {
	{ .compatible = "ti,am3356-pru", },
	{ .compatible = "ti,am4376-pru", },
	{ .compatible = "ti,am5728-pru", },
	{},
};
MODULE_DEVICE_TABLE(of, pru_rproc_match);

static struct platform_driver pru_rproc_driver = {
	.driver = {
		.name   = "pru-rproc",
		.of_match_table = pru_rproc_match,
		.suppress_bind_attrs = true,
	},
	.probe  = pru_rproc_probe,
	.remove = pru_rproc_remove,
};
module_platform_driver(pru_rproc_driver);

MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
MODULE_DESCRIPTION("PRU-ICSS Remote Processor Driver");
MODULE_LICENSE("GPL v2");
