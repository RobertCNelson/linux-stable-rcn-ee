/* SPDX-License-Identifier: GPL-2.0 */
/**
 * PRU-ICSS Subsystem user interfaces
 *
 * Copyright (C) 2015-2018 Texas Instruments Incorporated - http://www.ti.com
 *	Suman Anna <s-anna@ti.com>
 *	Tero Kristo <t-kristo@ti.com>
 */

#ifndef __LINUX_PRUSS_H
#define __LINUX_PRUSS_H

/**
 * enum pruss_pru_id - PRU core identifiers
 */
enum pruss_pru_id {
	PRUSS_PRU0 = 0,
	PRUSS_PRU1,
	PRUSS_NUM_PRUS,
};

/*
 * PRU_ICSS_CFG registers
 * SYSCFG, ISRP, ISP, IESP, IECP, SCRP applicable on AMxxxx devices only
 */
#define PRUSS_CFG_REVID		0x00
#define PRUSS_CFG_SYSCFG	0x04
#define PRUSS_CFG_GPCFG(x)	(0x08 + (x) * 4)
#define PRUSS_CFG_CGR		0x10
#define PRUSS_CFG_ISRP		0x14
#define PRUSS_CFG_ISP		0x18
#define PRUSS_CFG_IESP		0x1C
#define PRUSS_CFG_IECP		0x20
#define PRUSS_CFG_SCRP		0x24
#define PRUSS_CFG_PMAO		0x28
#define PRUSS_CFG_MII_RT	0x2C
#define PRUSS_CFG_IEPCLK	0x30
#define PRUSS_CFG_SPP		0x34
#define PRUSS_CFG_PIN_MX	0x40

/* PRUSS_GPCFG register bits */
#define PRUSS_GPCFG_PRU_GPO_SH_SEL		BIT(25)

#define PRUSS_GPCFG_PRU_DIV1_SHIFT		20
#define PRUSS_GPCFG_PRU_DIV1_MASK		GENMASK(24, 20)

#define PRUSS_GPCFG_PRU_DIV0_SHIFT		15
#define PRUSS_GPCFG_PRU_DIV0_MASK		GENMASK(15, 19)

#define PRUSS_GPCFG_PRU_GPO_MODE		BIT(14)
#define PRUSS_GPCFG_PRU_GPO_MODE_DIRECT		0
#define PRUSS_GPCFG_PRU_GPO_MODE_SERIAL		BIT(14)

#define PRUSS_GPCFG_PRU_GPI_SB			BIT(13)

#define PRUSS_GPCFG_PRU_GPI_DIV1_SHIFT		8
#define PRUSS_GPCFG_PRU_GPI_DIV1_MASK		GENMASK(12, 8)

#define PRUSS_GPCFG_PRU_GPI_DIV0_SHIFT		3
#define PRUSS_GPCFG_PRU_GPI_DIV0_MASK		GENMASK(7, 3)

#define PRUSS_GPCFG_PRU_GPI_CLK_MODE_POSITIVE	0
#define PRUSS_GPCFG_PRU_GPI_CLK_MODE_NEGATIVE	BIT(2)
#define PRUSS_GPCFG_PRU_GPI_CLK_MODE		BIT(2)

#define PRUSS_GPCFG_PRU_GPI_MODE_MASK		GENMASK(1, 0)
#define PRUSS_GPCFG_PRU_GPI_MODE_SHIFT		0

#define PRUSS_GPCFG_PRU_MUX_SEL_SHIFT		26
#define PRUSS_GPCFG_PRU_MUX_SEL_MASK		GENMASK(29, 26)

/* PRUSS_MII_RT register bits */
#define PRUSS_MII_RT_EVENT_EN			BIT(0)

/* PRUSS_SPP register bits */
#define PRUSS_SPP_XFER_SHIFT_EN			BIT(1)
#define PRUSS_SPP_PRU1_PAD_HP_EN		BIT(0)

/**
 * enum pruss_gp_mux_sel - PRUSS GPI/O Mux modes for the
 * PRUSS_GPCFG0/1 registers
 *
 * NOTE: The below defines are the most common values, but there
 * are some exceptions like on 66AK2G, where the RESERVED and MII2
 * values are interchanged. Also, this bit-field does not exist on
 * AM335x SoCs
 */
enum pruss_gp_mux_sel {
	PRUSS_GP_MUX_SEL_GP = 0,
	PRUSS_GP_MUX_SEL_ENDAT,
	PRUSS_GP_MUX_SEL_RESERVED,
	PRUSS_GP_MUX_SEL_SD,
	PRUSS_GP_MUX_SEL_MII2,
	PRUSS_GP_MUX_SEL_MAX,
};

/**
 * enum pruss_gpi_mode - PRUSS GPI configuration modes, used
 *			 to program the PRUSS_GPCFG0/1 registers
 */
enum pruss_gpi_mode {
	PRUSS_GPI_MODE_DIRECT = 0,
	PRUSS_GPI_MODE_PARALLEL,
	PRUSS_GPI_MODE_28BIT_SHIFT,
	PRUSS_GPI_MODE_MII,
};

/**
 * enum pruss_mem - PRUSS memory range identifiers
 */
enum pruss_mem {
	PRUSS_MEM_DRAM0 = 0,
	PRUSS_MEM_DRAM1,
	PRUSS_MEM_SHRD_RAM2,
	PRUSS_MEM_MAX,
};

/**
 * struct pruss_mem_region - PRUSS memory region structure
 * @va: kernel virtual address of the PRUSS memory region
 * @pa: physical (bus) address of the PRUSS memory region
 * @size: size of the PRUSS memory region
 */
struct pruss_mem_region {
	void __iomem *va;
	phys_addr_t pa;
	size_t size;
};

/* maximum number of system events */
#define MAX_PRU_SYS_EVENTS	64

/* maximum number of interrupt channels */
#define MAX_PRU_CHANNELS	10

/**
 * struct pruss_intc_config - INTC configuration info
 * @sysev_to_ch: system events to channel mapping information
 * @ch_to_host: interrupt channel to host interrupt information
 */
struct pruss_intc_config {
	s8 sysev_to_ch[MAX_PRU_SYS_EVENTS];
	s8 ch_to_host[MAX_PRU_CHANNELS];
};

/**
 * enum pru_ctable_idx - Configurable Constant table index identifiers
 */
enum pru_ctable_idx {
	PRU_C24 = 0,
	PRU_C25,
	PRU_C26,
	PRU_C27,
	PRU_C28,
	PRU_C29,
	PRU_C30,
	PRU_C31,
};

struct pruss;
struct rproc;

#if IS_ENABLED(CONFIG_TI_PRUSS)

struct pruss *pruss_get(struct rproc *rproc);
void pruss_put(struct pruss *pruss);

int pruss_request_mem_region(struct pruss *pruss, enum pruss_mem mem_id,
			     struct pruss_mem_region *region);
int pruss_release_mem_region(struct pruss *pruss,
			     struct pruss_mem_region *region);

int pruss_intc_trigger(unsigned int irq);

int pruss_cfg_read(struct pruss *pruss, unsigned int reg, unsigned int *val);
int pruss_cfg_update(struct pruss *pruss, unsigned int reg,
		     unsigned int mask, unsigned int val);

/**
 * pruss_intc_configure() - configure the PRUSS INTC
 * @pruss: the pruss instance
 * @intc_config: PRU core-specific INTC configuration
 *
 * Configures the PRUSS INTC with the provided configuration from
 * a PRU core. Any existing event to channel mappings or channel to
 * host interrupt mappings are checked to make sure there are no
 * conflicting configuration between both the PRU cores. The function
 * is intended to be used only by the PRU remoteproc driver.
 *
 * Returns 0 on success, or a suitable error code otherwise
 */
int pruss_intc_configure(struct pruss *pruss,
			 struct pruss_intc_config *intc_config);

/**
 * pruss_intc_unconfigure() - unconfigure the PRUSS INTC
 * @pruss: the pruss instance
 * @intc_config: PRU core specific INTC configuration
 *
 * Undo whatever was done in pruss_intc_configure() for a PRU core.
 * It should be sufficient to just mark the resources free in the
 * global map and disable the host interrupts and sysevents.
 */
int pruss_intc_unconfigure(struct pruss *pruss,
			   struct pruss_intc_config *intc_config);

/**
 * pruss_cfg_get_gpmux() - get the current GPMUX value for a PRU device
 * @pruss: pruss instance
 * @id: PRU identifier (0-1)
 * @mux: pointer to store the current mux value into
 */
static inline int pruss_cfg_get_gpmux(struct pruss *pruss,
				      enum pruss_pru_id id, u8 *mux)
{
	int ret = 0;
	u32 val;

	ret = pruss_cfg_read(pruss, PRUSS_CFG_GPCFG(id), &val);
	if (!ret)
		*mux = (u8)((val & PRUSS_GPCFG_PRU_MUX_SEL_MASK) >>
			    PRUSS_GPCFG_PRU_MUX_SEL_SHIFT);
	return ret;
}

/**
 * pruss_cfg_set_gpmux() - set the GPMUX value for a PRU device
 * @pruss: pruss instance
 * @pru_id: PRU identifier (0-1)
 * @mux: new mux value for PRU
 */
static inline int pruss_cfg_set_gpmux(struct pruss *pruss,
				      enum pruss_pru_id id, u8 mux)
{
	if (mux >= PRUSS_GP_MUX_SEL_MAX)
		return -EINVAL;

	return pruss_cfg_update(pruss, PRUSS_CFG_GPCFG(id),
				PRUSS_GPCFG_PRU_MUX_SEL_MASK,
				(u32)mux << PRUSS_GPCFG_PRU_MUX_SEL_SHIFT);
}

/**
 * pruss_cfg_miirt_enable() - Enable/disable MII RT Events
 * @pruss: the pruss instance
 * @enable: enable/disable
 *
 * Enable/disable the MII RT Events for the PRUSS.
 */
static inline int pruss_cfg_miirt_enable(struct pruss *pruss, bool enable)
{
	u32 set = enable ? PRUSS_MII_RT_EVENT_EN : 0;

	return pruss_cfg_update(pruss, PRUSS_CFG_MII_RT,
				PRUSS_MII_RT_EVENT_EN, set);
}

/**
 * pruss_cfg_xfr_enable() - Enable/disable XIN XOUT shift functionality
 * @pruss: the pruss instance
 * @enable: enable/disable
 */
static inline int pruss_cfg_xfr_enable(struct pruss *pruss, bool enable)
{
	u32 set = enable ? PRUSS_SPP_XFER_SHIFT_EN : 0;

	return pruss_cfg_update(pruss, PRUSS_CFG_SPP,
				PRUSS_SPP_XFER_SHIFT_EN, set);
}
#else

static inline struct pruss *pruss_get(struct rproc *rproc)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline void pruss_put(struct pruss *pruss) { }

static inline int pruss_request_mem_region(struct pruss *pruss,
					   enum pruss_mem mem_id,
					   struct pruss_mem_region *region)
{
	return -ENOTSUPP;
}

static inline int pruss_release_mem_region(struct pruss *pruss,
					   struct pruss_mem_region *region)
{
	return -ENOTSUPP;
}

static inline int pruss_intc_trigger(unsigned int irq)
{
	return -ENOTSUPP;
}

int pruss_cfg_read(struct pruss *pruss, unsigned int reg, unsigned int *val)
{
	return -ENOTSUPP;
}

int pruss_cfg_update(struct pruss *pruss, unsigned int reg,
		     unsigned int mask, unsigned int val)
{
	return -ENOTSUPP;
}

int pruss_intc_configure(struct pruss *pruss,
			 struct pruss_intc_config *intc_config)
{
	return -ENOTSUPP;
}

int pruss_intc_unconfigure(struct pruss *pruss,
			   struct pruss_intc_config *intc_config)
{
	return -ENOTSUPP;
}

static inline int pruss_cfg_get_gpmux(struct pruss *pruss,
				      enum pruss_pru_id id, u8 *mux)
{
	return -ENOTSUPP;
}

static inline int pruss_cfg_set_gpmux(struct pruss *pruss,
				      enum pruss_pru_id id, u8 mux)
{
	return -ENOTSUPP;
}

static inline int pruss_cfg_miirt_enable(struct pruss *pruss, bool enable)
{
	return -ENOTSUPP;
}

static inline int pruss_cfg_xfr_enable(struct pruss *pruss, bool enable)
{
	return -ENOTSUPP;
}

#endif /* CONFIG_TI_PRUSS */

#if IS_ENABLED(CONFIG_PRUSS_REMOTEPROC)

struct rproc *pru_rproc_get(struct device_node *node, int index);
void pru_rproc_put(struct rproc *rproc);
enum pruss_pru_id pru_rproc_get_id(struct rproc *rproc);
int pru_rproc_set_ctable(struct rproc *rproc, enum pru_ctable_idx c, u32 addr);

/**
 * pruss_cfg_gpimode() - set the GPI mode of the PRU
 * @pruss: the pruss instance handle
 * @pru: the rproc instance handle of the PRU
 * @mode: GPI mode to set
 *
 * Sets the GPI mode for a given PRU by programming the
 * corresponding PRUSS_CFG_GPCFGx register
 *
 * Returns 0 on success, or an error code otherwise
 */
static inline int pruss_cfg_gpimode(struct pruss *pruss, struct rproc *pru,
				    enum pruss_gpi_mode mode)
{
	enum pruss_pru_id id = pru_rproc_get_id(pru);

	if (id < 0)
		return -EINVAL;

	return pruss_cfg_update(pruss, PRUSS_CFG_GPCFG(id),
				PRUSS_GPCFG_PRU_GPI_MODE_MASK,
				mode << PRUSS_GPCFG_PRU_GPI_MODE_SHIFT);
}

#else

static inline struct rproc *pru_rproc_get(struct device_node *node, int index)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline void pru_rproc_put(struct rproc *rproc) { }

static inline enum pruss_pru_id pru_rproc_get_id(struct rproc *rproc)
{
	return -ENOTSUPP;
}

static inline int pru_rproc_set_ctable(struct rproc *rproc,
				       enum pru_ctable_idx c, u32 addr)
{
	return -ENOTSUPP;
}

static inline int pruss_cfg_gpimode(struct pruss *pruss, struct rproc *pru,
				    enum pruss_gpi_mode mode)
{
	return -ENOTSUPP;
}

#endif /* CONFIG_PRUSS_REMOTEPROC */

#endif /* __LINUX_PRUSS_H */
