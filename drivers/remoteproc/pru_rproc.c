// SPDX-License-Identifier: GPL-2.0
/*
 * PRU-ICSS remoteproc driver for various TI SoCs
 *
 * Copyright (C) 2014-2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 *	Andrew F. Davis <afd@ti.com>
 */

#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/remoteproc.h>
#include <linux/pruss.h>

#include "remoteproc_internal.h"
#include "pru_rproc.h"

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
 * enum pru_mem - PRU core memory range identifiers
 */
enum pru_mem {
	PRU_MEM_IRAM = 0,
	PRU_MEM_CTRL,
	PRU_MEM_DEBUG,
	PRU_MEM_MAX,
};

/**
 * struct pru_rproc - PRU remoteproc structure
 * @id: id of the PRU core within the PRUSS
 * @pruss: back-reference to parent PRUSS structure
 * @rproc: remoteproc pointer for this PRU core
 * @client_np: client device node
 * @mbox: mailbox channel handle used for vring signalling with MPU
 * @client: mailbox client to request the mailbox channel
 * @irq_ring: IRQ number to use for processing vring buffers
 * @irq_kick: IRQ number to use to perform virtio kick
 * @mem_regions: data for each of the PRU memory regions
 * @intc_config: PRU INTC configuration data
 * @dram0: PRUSS DRAM0 region
 * @dram1: PRUSS DRAM1 region
 * @shrdram: PRUSS SHARED RAM region
 * @rmw_lock: lock for read, modify, write operations on registers
 * @iram_da: device address of Instruction RAM for this PRU
 * @pdram_da: device address of primary Data RAM for this PRU
 * @sdram_da: device address of secondary Data RAM for this PRU
 * @shrdram_da: device address of shared Data RAM
 * @fw_name: name of firmware image used during loading
 * @gpmux_save: saved value for gpmux config
 * @dt_irqs: number of irqs configured from DT
 * @fw_irqs: number of irqs configured from FW
 * @lock: mutex to protect client usage
 * @dbg_single_step: debug state variable to set PRU into single step mode
 * @dbg_continuous: debug state variable to restore PRU execution mode
 */
struct pru_rproc {
	int id;
	struct pruss *pruss;
	struct rproc *rproc;
	struct device_node *client_np;
	struct mbox_chan *mbox;
	struct mbox_client client;
	int irq_vring;
	int irq_kick;
	struct pruss_mem_region mem_regions[PRU_MEM_MAX];
	struct pruss_intc_config intc_config;
	struct pruss_mem_region dram0;
	struct pruss_mem_region dram1;
	struct pruss_mem_region shrdram;
	spinlock_t rmw_lock; /* register access lock */
	u32 iram_da;
	u32 pdram_da;
	u32 sdram_da;
	u32 shrdram_da;
	const char *fw_name;
	u8 gpmux_save;
	int dt_irqs;
	int fw_irqs;
	struct mutex lock; /* client access lock */
	u32 dbg_single_step;
	u32 dbg_continuous;
};

static void *pru_d_da_to_va(struct pru_rproc *pru, u32 da, int len);

static inline u32 pru_control_read_reg(struct pru_rproc *pru, unsigned int reg)
{
	return readl_relaxed(pru->mem_regions[PRU_MEM_CTRL].va + reg);
}

static inline
void pru_control_write_reg(struct pru_rproc *pru, unsigned int reg, u32 val)
{
	writel_relaxed(val, pru->mem_regions[PRU_MEM_CTRL].va + reg);
}

static inline
void pru_control_set_reg(struct pru_rproc *pru, unsigned int reg,
			 u32 mask, u32 set)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&pru->rmw_lock, flags);

	val = pru_control_read_reg(pru, reg);
	val &= ~mask;
	val |= (set & mask);
	pru_control_write_reg(pru, reg, val);

	spin_unlock_irqrestore(&pru->rmw_lock, flags);
}

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

static int pru_get_intc_dt_config(struct device *dev, const char *propname,
				  int index,
				  struct pruss_intc_config *intc_config)
{
	struct device_node *np = dev->of_node;
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
			dev_dbg(dev, "bad sys event %d\n", arr[i + 1]);
			ret = -EINVAL;
			goto err;
		}

		if (arr[i + 2] < 0 ||
		    arr[i + 2] >= max_pru_channels) {
			dev_dbg(dev, "bad channel %d\n", arr[i + 2]);
			ret = -EINVAL;
			goto err;
		}

		if (arr[i + 3] < 0 ||
		    arr[i + 3] >= max_pru_host_ints) {
			dev_dbg(dev, "bad irq %d\n", arr[i + 3]);
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

	ret = pruss_cfg_get_gpmux(pru->pruss, pru->id, &pru->gpmux_save);
	if (ret) {
		dev_err(dev, "failed to get cfg gpmux: %d\n", ret);
		goto err;
	}

	ret = of_property_read_u32_index(np, "ti,pruss-gp-mux-sel", index,
					 &mux);
	if (!ret) {
		ret = pruss_cfg_set_gpmux(pru->pruss, pru->id, mux);
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

	ret = pru_get_intc_dt_config(dev, "ti,pru-interrupt-map",
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
	pruss_cfg_set_gpmux(pru->pruss, pru->id, pru->gpmux_save);

	mutex_lock(&pru->lock);
	pru->client_np = NULL;
	mutex_unlock(&pru->lock);

	put_device(&rproc->dev);
}
EXPORT_SYMBOL_GPL(pru_rproc_put);

/**
 * pru_rproc_get_id() - get PRU id from a previously acquired PRU remoteproc
 * @rproc: the rproc instance of the PRU
 *
 * Returns the PRU id of the PRU remote processor that has been acquired through
 * a pru_rproc_get(), or a negative value on error
 */
enum pruss_pru_id pru_rproc_get_id(struct rproc *rproc)
{
	struct pru_rproc *pru;

	if (IS_ERR_OR_NULL(rproc) || !rproc->dev.parent)
		return -EINVAL;

	/* TODO: replace the crude string based check to make sure it is PRU */
	if (!strstr(dev_name(rproc->dev.parent), "pru"))
		return -EINVAL;

	pru = rproc->priv;
	return pru->id;
}
EXPORT_SYMBOL_GPL(pru_rproc_get_id);

/**
 * pru_rproc_set_ctable() - set the constant table index for the PRU
 * @rproc: the rproc instance of the PRU
 * @c: constant table index to set
 * @addr: physical address to set it to
 */
int pru_rproc_set_ctable(struct rproc *rproc, enum pru_ctable_idx c, u32 addr)
{
	struct pru_rproc *pru = rproc->priv;
	unsigned int reg;
	u32 mask, set;
	u16 idx;
	u16 idx_mask;

	/* pointer is 16 bit and index is 8-bit so mask out the rest */
	idx_mask = (c >= PRU_C28) ? 0xFFFF : 0xFF;

	/* ctable uses bit 8 and upwards only */
	idx = (addr >> 8) & idx_mask;

	/* configurable ctable (i.e. C24) starts at PRU_CTRL_CTBIR0 */
	reg = PRU_CTRL_CTBIR0 + 4 * (c >> 1);
	mask = idx_mask << (16 * (c & 1));
	set = idx << (16 * (c & 1));

	pru_control_set_reg(pru, reg, mask, set);

	return 0;
}
EXPORT_SYMBOL_GPL(pru_rproc_set_ctable);

static inline u32 pru_debug_read_reg(struct pru_rproc *pru, unsigned int reg)
{
	return readl_relaxed(pru->mem_regions[PRU_MEM_DEBUG].va + reg);
}

static inline
void pru_debug_write_reg(struct pru_rproc *pru, unsigned int reg, u32 val)
{
	writel_relaxed(val, pru->mem_regions[PRU_MEM_DEBUG].va + reg);
}

static int pru_rproc_debug_read_regs(struct seq_file *s, void *data)
{
	struct rproc *rproc = s->private;
	struct pru_rproc *pru = rproc->priv;
	int i, nregs = 32;
	u32 pru_sts;
	int pru_is_running;

	seq_puts(s, "============== Control Registers ==============\n");
	seq_printf(s, "CTRL      := 0x%08x\n",
		   pru_control_read_reg(pru, PRU_CTRL_CTRL));
	pru_sts = pru_control_read_reg(pru, PRU_CTRL_STS);
	seq_printf(s, "STS (PC)  := 0x%08x (0x%08x)\n", pru_sts, pru_sts << 2);
	seq_printf(s, "WAKEUP_EN := 0x%08x\n",
		   pru_control_read_reg(pru, PRU_CTRL_WAKEUP_EN));
	seq_printf(s, "CYCLE     := 0x%08x\n",
		   pru_control_read_reg(pru, PRU_CTRL_CYCLE));
	seq_printf(s, "STALL     := 0x%08x\n",
		   pru_control_read_reg(pru, PRU_CTRL_STALL));
	seq_printf(s, "CTBIR0    := 0x%08x\n",
		   pru_control_read_reg(pru, PRU_CTRL_CTBIR0));
	seq_printf(s, "CTBIR1    := 0x%08x\n",
		   pru_control_read_reg(pru, PRU_CTRL_CTBIR1));
	seq_printf(s, "CTPPR0    := 0x%08x\n",
		   pru_control_read_reg(pru, PRU_CTRL_CTPPR0));
	seq_printf(s, "CTPPR1    := 0x%08x\n",
		   pru_control_read_reg(pru, PRU_CTRL_CTPPR1));

	seq_puts(s, "=============== Debug Registers ===============\n");
	pru_is_running = pru_control_read_reg(pru, PRU_CTRL_CTRL) &
				CTRL_CTRL_RUNSTATE;
	if (pru_is_running) {
		seq_puts(s, "PRU is executing, cannot print/access debug registers.\n");
		return 0;
	}

	for (i = 0; i < nregs; i++) {
		seq_printf(s, "GPREG%-2d := 0x%08x\tCT_REG%-2d := 0x%08x\n",
			   i, pru_debug_read_reg(pru, PRU_DEBUG_GPREG(i)),
			   i, pru_debug_read_reg(pru, PRU_DEBUG_CT_REG(i)));
	}

	return 0;
}

static int pru_rproc_debug_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, pru_rproc_debug_read_regs, inode->i_private);
}

static const struct file_operations pru_rproc_debug_regs_ops = {
	.open = pru_rproc_debug_regs_open,
	.read = seq_read,
	.llseek	= seq_lseek,
	.release = single_release,
};

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

	val = val ? 1 : 0;
	if (!val && !pru->dbg_single_step)
		return 0;

	reg_val = pru_control_read_reg(pru, PRU_CTRL_CTRL);

	if (val && !pru->dbg_single_step)
		pru->dbg_continuous = reg_val;

	if (val)
		reg_val |= CTRL_CTRL_SINGLE_STEP | CTRL_CTRL_EN;
	else
		reg_val = pru->dbg_continuous;

	pru->dbg_single_step = val;
	pru_control_write_reg(pru, PRU_CTRL_CTRL, reg_val);

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

	debugfs_create_file("regs", 0400, rproc->dbg_dir,
			    rproc, &pru_rproc_debug_regs_ops);
	debugfs_create_file("single_step", 0600, rproc->dbg_dir,
			    rproc, &pru_rproc_debug_ss_fops);
}

/**
 * pru_rproc_mbox_callback() - inbound mailbox message handler
 * @client: mailbox client pointer used for requesting the mailbox channel
 * @data: mailbox payload
 *
 * This handler is invoked by omap's mailbox driver whenever a mailbox
 * message is received. Usually, the mailbox payload simply contains
 * the index of the virtqueue that is kicked by the PRU remote processor,
 * and we let remoteproc core handle it.
 *
 * In addition to virtqueue indices, we might also have some out-of-band
 * values that indicates different events. Those values are deliberately
 * very big so they don't coincide with virtqueue indices.
 */
static void pru_rproc_mbox_callback(struct mbox_client *client, void *data)
{
	struct pru_rproc *pru = container_of(client, struct pru_rproc, client);
	struct device *dev = &pru->rproc->dev;
	u32 msg = (u32)data;

	dev_dbg(dev, "mbox msg: 0x%x\n", msg);

	/* msg contains the index of the triggered vring */
	if (rproc_vq_interrupt(pru->rproc, msg) == IRQ_NONE)
		dev_dbg(dev, "no message was found in vqid %d\n", msg);
}

/**
 * pru_rproc_vring_interrupt() - interrupt handler for processing vrings
 * @irq: irq number associated with the PRU event MPU is listening on
 * @data: interrupt handler data, will be a PRU rproc structure
 *
 * This handler is used by the PRU remoteproc driver when using PRU system
 * events for processing the virtqueues. Unlike the mailbox IP, there is
 * no payload associated with an interrupt, so either a unique event is
 * used for each virtqueue kick, or a both virtqueues are processed on
 * a single event. The latter is chosen to conserve the usable PRU system
 * events.
 */
static irqreturn_t pru_rproc_vring_interrupt(int irq, void *data)
{
	struct pru_rproc *pru = data;

	dev_dbg(&pru->rproc->dev, "got vring irq\n");

	/* process incoming buffers on both the Rx and Tx vrings */
	rproc_vq_interrupt(pru->rproc, 0);
	rproc_vq_interrupt(pru->rproc, 1);

	return IRQ_HANDLED;
}

/* kick a virtqueue */
static void pru_rproc_kick(struct rproc *rproc, int vq_id)
{
	struct device *dev = &rproc->dev;
	struct pru_rproc *pru = rproc->priv;
	int ret;

	dev_dbg(dev, "kicking vqid %d on PRU%d\n", vq_id, pru->id);

	if (pru->irq_kick > 0) {
		ret = pruss_intc_trigger(pru->irq_kick);
		if (ret < 0)
			dev_err(dev, "pruss_intc_trigger failed: %d\n", ret);
	} else if (pru->mbox) {
		/*
		 * send the index of the triggered virtqueue in the mailbox
		 * payload
		 */
		ret = mbox_send_message(pru->mbox, (void *)vq_id);
		if (ret < 0)
			dev_err(dev, "mbox_send_message failed: %d\n", ret);
	}
}

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
		ret = pruss_intc_configure(pru->pruss, &pru->intc_config);
		if (ret) {
			dev_err(dev, "failed to configure intc %d\n", ret);
			return ret;
		}
	}

	if (!list_empty(&pru->rproc->rvdevs)) {
		if (!pru->mbox && (pru->irq_vring <= 0 || pru->irq_kick <= 0)) {
			dev_err(dev, "virtio vring interrupt mechanisms are not provided\n");
			ret = -EINVAL;
			goto fail;
		}

		if (!pru->mbox && pru->irq_vring > 0) {
			ret = request_threaded_irq(pru->irq_vring, NULL,
						   pru_rproc_vring_interrupt,
						   IRQF_ONESHOT, dev_name(dev),
						   pru);
			if (ret) {
				dev_err(dev, "failed to enable vring interrupt, ret = %d\n",
					ret);
				goto fail;
			}
		}
	}

	val = CTRL_CTRL_EN | ((rproc->bootaddr >> 2) << 16);
	pru_control_write_reg(pru, PRU_CTRL_CTRL, val);

	return 0;

fail:
	if (pru->dt_irqs || pru->fw_irqs)
		pruss_intc_unconfigure(pru->pruss, &pru->intc_config);

	return ret;
}

/* stop/disable a PRU core */
static int pru_rproc_stop(struct rproc *rproc)
{
	struct device *dev = &rproc->dev;
	struct pru_rproc *pru = rproc->priv;
	u32 val;

	dev_dbg(dev, "stopping PRU%d\n", pru->id);

	val = pru_control_read_reg(pru, PRU_CTRL_CTRL);
	val &= ~CTRL_CTRL_EN;
	pru_control_write_reg(pru, PRU_CTRL_CTRL, val);

	if (!list_empty(&pru->rproc->rvdevs) &&
	    !pru->mbox && pru->irq_vring > 0)
		free_irq(pru->irq_vring, pru);

	/* undo INTC config */
	if (pru->dt_irqs || pru->fw_irqs)
		pruss_intc_unconfigure(pru->pruss, &pru->intc_config);

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
	    da + len <= pru->iram_da + pru->mem_regions[PRU_MEM_IRAM].size) {
		offset = da - pru->iram_da;
		va = (__force void *)(pru->mem_regions[PRU_MEM_IRAM].va +
				      offset);
	}

	return va;
}

/* PRU-specific address translator */
static void *pru_da_to_va(struct rproc *rproc, u64 da, int len, u32 flags)
{
	struct pru_rproc *pru = rproc->priv;
	void *va;
	u32 exec_flag;

	exec_flag = ((flags & RPROC_FLAGS_ELF_SHDR) ? flags & SHF_EXECINSTR :
		     ((flags & RPROC_FLAGS_ELF_PHDR) ? flags & PF_X : 0));

	if (exec_flag)
		va = pru_i_da_to_va(pru, da, len);
	else
		va = pru_d_da_to_va(pru, da, len);

	return va;
}

static struct rproc_ops pru_rproc_ops = {
	.start			= pru_rproc_start,
	.stop			= pru_rproc_stop,
	.kick			= pru_rproc_kick,
	.da_to_va		= pru_da_to_va,
	.handle_vendor_rsc	= pru_rproc_handle_vendor_rsc,
};

static int pru_rproc_set_id(struct pru_rproc *pru)
{
	int ret = 0;
	u32 mask1 = 0x34000;
	u32 mask2 = 0x38000;

	if ((pru->mem_regions[0].pa & mask1) == mask1)
		pru->id = PRUSS_PRU0;
	else if ((pru->mem_regions[0].pa & mask2) == mask2)
		pru->id = PRUSS_PRU1;
	else
		ret = -EINVAL;

	return ret;
}

static int pru_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct platform_device *ppdev = to_platform_device(dev->parent);
	struct pru_rproc *pru;
	const char *fw_name;
	struct rproc *rproc = NULL;
	struct mbox_client *client;
	struct resource *res;
	int i, ret;
	const char *mem_names[PRU_MEM_MAX] = { "iram", "control", "debug" };

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

	pru = rproc->priv;
	pru->pruss = platform_get_drvdata(ppdev);
	pru->rproc = rproc;
	pru->fw_name = fw_name;
	spin_lock_init(&pru->rmw_lock);
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

	for (i = 0; i < ARRAY_SIZE(mem_names); i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   mem_names[i]);
		pru->mem_regions[i].va = devm_ioremap_resource(dev, res);
		if (IS_ERR(pru->mem_regions[i].va)) {
			dev_err(dev, "failed to parse and map memory resource %d %s\n",
				i, mem_names[i]);
			ret = PTR_ERR(pru->mem_regions[i].va);
			goto free_rproc;
		}
		pru->mem_regions[i].pa = res->start;
		pru->mem_regions[i].size = resource_size(res);

		dev_dbg(dev, "memory %8s: pa %pa size 0x%zx va %p\n",
			mem_names[i], &pru->mem_regions[i].pa,
			pru->mem_regions[i].size, pru->mem_regions[i].va);
	}

	ret = pru_rproc_set_id(pru);
	if (ret < 0)
		goto free_rproc;

	platform_set_drvdata(pdev, rproc);

	/* get optional vring and kick interrupts for supporting virtio rpmsg */
	pru->irq_vring = platform_get_irq_byname(pdev, "vring");
	if (pru->irq_vring <= 0) {
		ret = pru->irq_vring;
		if (ret == -EPROBE_DEFER)
			goto free_rproc;
		dev_dbg(dev, "unable to get vring interrupt, status = %d\n",
			ret);
	}

	pru->irq_kick = platform_get_irq_byname(pdev, "kick");
	if (pru->irq_kick <= 0) {
		ret = pru->irq_kick;
		if (ret == -EPROBE_DEFER)
			goto free_rproc;
		dev_dbg(dev, "unable to get kick interrupt, status = %d\n",
			ret);
	}

	/*
	 * get optional mailbox for virtio rpmsg signalling if vring and kick
	 * interrupts are not specified for OMAP architecture based SoCs
	 */
	if (pru->irq_vring <= 0 && pru->irq_kick <= 0 &&
	    !of_device_is_compatible(np, "ti,k2g-pru") &&
	    !of_device_is_compatible(np, "ti,da850-pru")) {
		client = &pru->client;
		client->dev = dev;
		client->tx_done = NULL;
		client->rx_callback = pru_rproc_mbox_callback;
		client->tx_block = false;
		client->knows_txdone = false;
		pru->mbox = mbox_request_channel(client, 0);
		if (IS_ERR(pru->mbox)) {
			ret = PTR_ERR(pru->mbox);
			pru->mbox = NULL;
			dev_dbg(dev, "unable to get mailbox channel, status = %d\n",
				ret);
		}
	}

	ret = rproc_add(pru->rproc);
	if (ret) {
		dev_err(dev, "rproc_add failed: %d\n", ret);
		goto put_mbox;
	}

	pru_rproc_create_debug_entries(rproc);

	dev_info(dev, "PRU rproc node %s probed successfully\n", np->full_name);

	return 0;

put_mbox:
	mbox_free_channel(pru->mbox);
free_rproc:
	rproc_free(rproc);
	return ret;
}

static int pru_rproc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct pru_rproc *pru = rproc->priv;

	dev_info(dev, "%s: removing rproc %s\n", __func__, rproc->name);

	mbox_free_channel(pru->mbox);

	rproc_del(rproc);
	rproc_free(rproc);

	return 0;
}

static const struct of_device_id pru_rproc_match[] = {
	{ .compatible = "ti,am3356-pru", },
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
