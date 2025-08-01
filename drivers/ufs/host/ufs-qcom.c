// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2013-2016, Linux Foundation. All rights reserved.
 */

#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/gpio/consumer.h>
#include <linux/interconnect.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/time.h>
#include <linux/unaligned.h>
#include <linux/units.h>

#include <soc/qcom/ice.h>

#include <ufs/ufshcd.h>
#include <ufs/ufshci.h>
#include <ufs/ufs_quirks.h>
#include <ufs/unipro.h>
#include "ufshcd-pltfrm.h"
#include "ufs-qcom.h"

#define MCQ_QCFGPTR_MASK	GENMASK(7, 0)
#define MCQ_QCFGPTR_UNIT	0x200
#define MCQ_SQATTR_OFFSET(c) \
	((((c) >> 16) & MCQ_QCFGPTR_MASK) * MCQ_QCFGPTR_UNIT)
#define MCQ_QCFG_SIZE	0x40

/* De-emphasis for gear-5 */
#define DEEMPHASIS_3_5_dB	0x04
#define NO_DEEMPHASIS		0x0

enum {
	TSTBUS_UAWM,
	TSTBUS_UARM,
	TSTBUS_TXUC,
	TSTBUS_RXUC,
	TSTBUS_DFC,
	TSTBUS_TRLUT,
	TSTBUS_TMRLUT,
	TSTBUS_OCSC,
	TSTBUS_UTP_HCI,
	TSTBUS_COMBINED,
	TSTBUS_WRAPPER,
	TSTBUS_UNIPRO,
	TSTBUS_MAX,
};

#define QCOM_UFS_MAX_GEAR 5
#define QCOM_UFS_MAX_LANE 2

enum {
	MODE_MIN,
	MODE_PWM,
	MODE_HS_RA,
	MODE_HS_RB,
	MODE_MAX,
};

static const struct __ufs_qcom_bw_table {
	u32 mem_bw;
	u32 cfg_bw;
} ufs_qcom_bw_table[MODE_MAX + 1][QCOM_UFS_MAX_GEAR + 1][QCOM_UFS_MAX_LANE + 1] = {
	[MODE_MIN][0][0]		   = { 0,		0 }, /* Bandwidth values in KB/s */
	[MODE_PWM][UFS_PWM_G1][UFS_LANE_1] = { 922,		1000 },
	[MODE_PWM][UFS_PWM_G2][UFS_LANE_1] = { 1844,		1000 },
	[MODE_PWM][UFS_PWM_G3][UFS_LANE_1] = { 3688,		1000 },
	[MODE_PWM][UFS_PWM_G4][UFS_LANE_1] = { 7376,		1000 },
	[MODE_PWM][UFS_PWM_G5][UFS_LANE_1] = { 14752,		1000 },
	[MODE_PWM][UFS_PWM_G1][UFS_LANE_2] = { 1844,		1000 },
	[MODE_PWM][UFS_PWM_G2][UFS_LANE_2] = { 3688,		1000 },
	[MODE_PWM][UFS_PWM_G3][UFS_LANE_2] = { 7376,		1000 },
	[MODE_PWM][UFS_PWM_G4][UFS_LANE_2] = { 14752,		1000 },
	[MODE_PWM][UFS_PWM_G5][UFS_LANE_2] = { 29504,		1000 },
	[MODE_HS_RA][UFS_HS_G1][UFS_LANE_1] = { 127796,		1000 },
	[MODE_HS_RA][UFS_HS_G2][UFS_LANE_1] = { 255591,		1000 },
	[MODE_HS_RA][UFS_HS_G3][UFS_LANE_1] = { 1492582,	102400 },
	[MODE_HS_RA][UFS_HS_G4][UFS_LANE_1] = { 2915200,	204800 },
	[MODE_HS_RA][UFS_HS_G5][UFS_LANE_1] = { 5836800,	409600 },
	[MODE_HS_RA][UFS_HS_G1][UFS_LANE_2] = { 255591,		1000 },
	[MODE_HS_RA][UFS_HS_G2][UFS_LANE_2] = { 511181,		1000 },
	[MODE_HS_RA][UFS_HS_G3][UFS_LANE_2] = { 1492582,	204800 },
	[MODE_HS_RA][UFS_HS_G4][UFS_LANE_2] = { 2915200,	409600 },
	[MODE_HS_RA][UFS_HS_G5][UFS_LANE_2] = { 5836800,	819200 },
	[MODE_HS_RB][UFS_HS_G1][UFS_LANE_1] = { 149422,		1000 },
	[MODE_HS_RB][UFS_HS_G2][UFS_LANE_1] = { 298189,		1000 },
	[MODE_HS_RB][UFS_HS_G3][UFS_LANE_1] = { 1492582,	102400 },
	[MODE_HS_RB][UFS_HS_G4][UFS_LANE_1] = { 2915200,	204800 },
	[MODE_HS_RB][UFS_HS_G5][UFS_LANE_1] = { 5836800,	409600 },
	[MODE_HS_RB][UFS_HS_G1][UFS_LANE_2] = { 298189,		1000 },
	[MODE_HS_RB][UFS_HS_G2][UFS_LANE_2] = { 596378,		1000 },
	[MODE_HS_RB][UFS_HS_G3][UFS_LANE_2] = { 1492582,	204800 },
	[MODE_HS_RB][UFS_HS_G4][UFS_LANE_2] = { 2915200,	409600 },
	[MODE_HS_RB][UFS_HS_G5][UFS_LANE_2] = { 5836800,	819200 },
	[MODE_MAX][0][0]		    = { 7643136,	819200 },
};

static const struct {
	int nminor;
	char *prefix;
} testbus_info[TSTBUS_MAX] = {
	[TSTBUS_UAWM]     = {32, "TSTBUS_UAWM"},
	[TSTBUS_UARM]     = {32, "TSTBUS_UARM"},
	[TSTBUS_TXUC]     = {32, "TSTBUS_TXUC"},
	[TSTBUS_RXUC]     = {32, "TSTBUS_RXUC"},
	[TSTBUS_DFC]      = {32, "TSTBUS_DFC"},
	[TSTBUS_TRLUT]    = {32, "TSTBUS_TRLUT"},
	[TSTBUS_TMRLUT]   = {32, "TSTBUS_TMRLUT"},
	[TSTBUS_OCSC]     = {32, "TSTBUS_OCSC"},
	[TSTBUS_UTP_HCI]  = {32, "TSTBUS_UTP_HCI"},
	[TSTBUS_COMBINED] = {32, "TSTBUS_COMBINED"},
	[TSTBUS_WRAPPER]  = {32, "TSTBUS_WRAPPER"},
	[TSTBUS_UNIPRO]   = {256, "TSTBUS_UNIPRO"},
};

static void ufs_qcom_get_default_testbus_cfg(struct ufs_qcom_host *host);
static unsigned long ufs_qcom_opp_freq_to_clk_freq(struct ufs_hba *hba,
						   unsigned long freq, char *name);
static int ufs_qcom_set_core_clk_ctrl(struct ufs_hba *hba, bool is_scale_up, unsigned long freq);

static struct ufs_qcom_host *rcdev_to_ufs_host(struct reset_controller_dev *rcd)
{
	return container_of(rcd, struct ufs_qcom_host, rcdev);
}

#ifdef CONFIG_SCSI_UFS_CRYPTO
/**
 * ufs_qcom_config_ice_allocator() - ICE core allocator configuration
 *
 * @host: pointer to qcom specific variant structure.
 */
static void ufs_qcom_config_ice_allocator(struct ufs_qcom_host *host)
{
	struct ufs_hba *hba = host->hba;
	static const uint8_t val[4] = { NUM_RX_R1W0, NUM_TX_R0W1, NUM_RX_R1W1, NUM_TX_R1W1 };
	u32 config;

	if (!(host->caps & UFS_QCOM_CAP_ICE_CONFIG) ||
			!(host->hba->caps & UFSHCD_CAP_CRYPTO))
		return;

	config = get_unaligned_le32(val);

	ufshcd_writel(hba, ICE_ALLOCATOR_TYPE, REG_UFS_MEM_ICE_CONFIG);
	ufshcd_writel(hba, config, REG_UFS_MEM_ICE_NUM_CORE);
}

static inline void ufs_qcom_ice_enable(struct ufs_qcom_host *host)
{
	if (host->hba->caps & UFSHCD_CAP_CRYPTO)
		qcom_ice_enable(host->ice);
}

static const struct blk_crypto_ll_ops ufs_qcom_crypto_ops; /* forward decl */

static int ufs_qcom_ice_init(struct ufs_qcom_host *host)
{
	struct ufs_hba *hba = host->hba;
	struct blk_crypto_profile *profile = &hba->crypto_profile;
	struct device *dev = hba->dev;
	struct qcom_ice *ice;
	union ufs_crypto_capabilities caps;
	union ufs_crypto_cap_entry cap;
	int err;
	int i;

	ice = devm_of_qcom_ice_get(dev);
	if (ice == ERR_PTR(-EOPNOTSUPP)) {
		dev_warn(dev, "Disabling inline encryption support\n");
		ice = NULL;
	}

	if (IS_ERR_OR_NULL(ice))
		return PTR_ERR_OR_ZERO(ice);

	host->ice = ice;

	/* Initialize the blk_crypto_profile */

	caps.reg_val = cpu_to_le32(ufshcd_readl(hba, REG_UFS_CCAP));

	/* The number of keyslots supported is (CFGC+1) */
	err = devm_blk_crypto_profile_init(dev, profile, caps.config_count + 1);
	if (err)
		return err;

	profile->ll_ops = ufs_qcom_crypto_ops;
	profile->max_dun_bytes_supported = 8;
	profile->key_types_supported = qcom_ice_get_supported_key_type(ice);
	profile->dev = dev;

	/*
	 * Currently this driver only supports AES-256-XTS.  All known versions
	 * of ICE support it, but to be safe make sure it is really declared in
	 * the crypto capability registers.  The crypto capability registers
	 * also give the supported data unit size(s).
	 */
	for (i = 0; i < caps.num_crypto_cap; i++) {
		cap.reg_val = cpu_to_le32(ufshcd_readl(hba,
						       REG_UFS_CRYPTOCAP +
						       i * sizeof(__le32)));
		if (cap.algorithm_id == UFS_CRYPTO_ALG_AES_XTS &&
		    cap.key_size == UFS_CRYPTO_KEY_SIZE_256)
			profile->modes_supported[BLK_ENCRYPTION_MODE_AES_256_XTS] |=
				cap.sdus_mask * 512;
	}

	hba->caps |= UFSHCD_CAP_CRYPTO;
	hba->quirks |= UFSHCD_QUIRK_CUSTOM_CRYPTO_PROFILE;
	return 0;
}

static inline int ufs_qcom_ice_resume(struct ufs_qcom_host *host)
{
	if (host->hba->caps & UFSHCD_CAP_CRYPTO)
		return qcom_ice_resume(host->ice);

	return 0;
}

static inline int ufs_qcom_ice_suspend(struct ufs_qcom_host *host)
{
	if (host->hba->caps & UFSHCD_CAP_CRYPTO)
		return qcom_ice_suspend(host->ice);

	return 0;
}

static int ufs_qcom_ice_keyslot_program(struct blk_crypto_profile *profile,
					const struct blk_crypto_key *key,
					unsigned int slot)
{
	struct ufs_hba *hba = ufs_hba_from_crypto_profile(profile);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int err;

	ufshcd_hold(hba);
	err = qcom_ice_program_key(host->ice, slot, key);
	ufshcd_release(hba);
	return err;
}

static int ufs_qcom_ice_keyslot_evict(struct blk_crypto_profile *profile,
				      const struct blk_crypto_key *key,
				      unsigned int slot)
{
	struct ufs_hba *hba = ufs_hba_from_crypto_profile(profile);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int err;

	ufshcd_hold(hba);
	err = qcom_ice_evict_key(host->ice, slot);
	ufshcd_release(hba);
	return err;
}

static int ufs_qcom_ice_derive_sw_secret(struct blk_crypto_profile *profile,
					 const u8 *eph_key, size_t eph_key_size,
					 u8 sw_secret[BLK_CRYPTO_SW_SECRET_SIZE])
{
	struct ufs_hba *hba = ufs_hba_from_crypto_profile(profile);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	return qcom_ice_derive_sw_secret(host->ice, eph_key, eph_key_size,
					 sw_secret);
}

static int ufs_qcom_ice_import_key(struct blk_crypto_profile *profile,
				   const u8 *raw_key, size_t raw_key_size,
				   u8 lt_key[BLK_CRYPTO_MAX_HW_WRAPPED_KEY_SIZE])
{
	struct ufs_hba *hba = ufs_hba_from_crypto_profile(profile);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	return qcom_ice_import_key(host->ice, raw_key, raw_key_size, lt_key);
}

static int ufs_qcom_ice_generate_key(struct blk_crypto_profile *profile,
				     u8 lt_key[BLK_CRYPTO_MAX_HW_WRAPPED_KEY_SIZE])
{
	struct ufs_hba *hba = ufs_hba_from_crypto_profile(profile);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	return qcom_ice_generate_key(host->ice, lt_key);
}

static int ufs_qcom_ice_prepare_key(struct blk_crypto_profile *profile,
				    const u8 *lt_key, size_t lt_key_size,
				    u8 eph_key[BLK_CRYPTO_MAX_HW_WRAPPED_KEY_SIZE])
{
	struct ufs_hba *hba = ufs_hba_from_crypto_profile(profile);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	return qcom_ice_prepare_key(host->ice, lt_key, lt_key_size, eph_key);
}

static const struct blk_crypto_ll_ops ufs_qcom_crypto_ops = {
	.keyslot_program	= ufs_qcom_ice_keyslot_program,
	.keyslot_evict		= ufs_qcom_ice_keyslot_evict,
	.derive_sw_secret	= ufs_qcom_ice_derive_sw_secret,
	.import_key		= ufs_qcom_ice_import_key,
	.generate_key		= ufs_qcom_ice_generate_key,
	.prepare_key		= ufs_qcom_ice_prepare_key,
};

#else

static inline void ufs_qcom_ice_enable(struct ufs_qcom_host *host)
{
}

static int ufs_qcom_ice_init(struct ufs_qcom_host *host)
{
	return 0;
}

static inline int ufs_qcom_ice_resume(struct ufs_qcom_host *host)
{
	return 0;
}

static inline int ufs_qcom_ice_suspend(struct ufs_qcom_host *host)
{
	return 0;
}

static void ufs_qcom_config_ice_allocator(struct ufs_qcom_host *host)
{
}

#endif

static void ufs_qcom_disable_lane_clks(struct ufs_qcom_host *host)
{
	if (!host->is_lane_clks_enabled)
		return;

	clk_bulk_disable_unprepare(host->num_clks, host->clks);

	host->is_lane_clks_enabled = false;
}

static int ufs_qcom_enable_lane_clks(struct ufs_qcom_host *host)
{
	int err;

	err = clk_bulk_prepare_enable(host->num_clks, host->clks);
	if (err)
		return err;

	host->is_lane_clks_enabled = true;

	return 0;
}

static int ufs_qcom_init_lane_clks(struct ufs_qcom_host *host)
{
	int err;
	struct device *dev = host->hba->dev;

	if (has_acpi_companion(dev))
		return 0;

	err = devm_clk_bulk_get_all(dev, &host->clks);
	if (err <= 0)
		return err;

	host->num_clks = err;

	return 0;
}

static int ufs_qcom_check_hibern8(struct ufs_hba *hba)
{
	int err;
	u32 tx_fsm_val;
	unsigned long timeout = jiffies + msecs_to_jiffies(HBRN8_POLL_TOUT_MS);

	do {
		err = ufshcd_dme_get(hba,
				UIC_ARG_MIB_SEL(MPHY_TX_FSM_STATE,
					UIC_ARG_MPHY_TX_GEN_SEL_INDEX(0)),
				&tx_fsm_val);
		if (err || tx_fsm_val == TX_FSM_HIBERN8)
			break;

		/* sleep for max. 200us */
		usleep_range(100, 200);
	} while (time_before(jiffies, timeout));

	/*
	 * we might have scheduled out for long during polling so
	 * check the state again.
	 */
	if (time_after(jiffies, timeout))
		err = ufshcd_dme_get(hba,
				UIC_ARG_MIB_SEL(MPHY_TX_FSM_STATE,
					UIC_ARG_MPHY_TX_GEN_SEL_INDEX(0)),
				&tx_fsm_val);

	if (err) {
		dev_err(hba->dev, "%s: unable to get TX_FSM_STATE, err %d\n",
				__func__, err);
	} else if (tx_fsm_val != TX_FSM_HIBERN8) {
		err = tx_fsm_val;
		dev_err(hba->dev, "%s: invalid TX_FSM_STATE = %d\n",
				__func__, err);
	}

	return err;
}

static void ufs_qcom_select_unipro_mode(struct ufs_qcom_host *host)
{
	ufshcd_rmwl(host->hba, QUNIPRO_SEL, QUNIPRO_SEL, REG_UFS_CFG1);

	if (host->hw_ver.major >= 0x05)
		ufshcd_rmwl(host->hba, QUNIPRO_G4_SEL, 0, REG_UFS_CFG0);
}

/*
 * ufs_qcom_host_reset - reset host controller and PHY
 */
static int ufs_qcom_host_reset(struct ufs_hba *hba)
{
	int ret;
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	bool reenable_intr;

	if (!host->core_reset)
		return 0;

	reenable_intr = hba->is_irq_enabled;
	ufshcd_disable_irq(hba);

	ret = reset_control_assert(host->core_reset);
	if (ret) {
		dev_err(hba->dev, "%s: core_reset assert failed, err = %d\n",
				 __func__, ret);
		return ret;
	}

	/*
	 * The hardware requirement for delay between assert/deassert
	 * is at least 3-4 sleep clock (32.7KHz) cycles, which comes to
	 * ~125us (4/32768). To be on the safe side add 200us delay.
	 */
	usleep_range(200, 210);

	ret = reset_control_deassert(host->core_reset);
	if (ret) {
		dev_err(hba->dev, "%s: core_reset deassert failed, err = %d\n",
				 __func__, ret);
		return ret;
	}

	usleep_range(1000, 1100);

	if (reenable_intr)
		ufshcd_enable_irq(hba);

	return 0;
}

static u32 ufs_qcom_get_hs_gear(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	if (host->hw_ver.major >= 0x4)
		return UFS_QCOM_MAX_GEAR(ufshcd_readl(hba, REG_UFS_PARAM0));

	/* Default is HS-G3 */
	return UFS_HS_G3;
}

static int ufs_qcom_power_up_sequence(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct ufs_host_params *host_params = &host->host_params;
	struct phy *phy = host->generic_phy;
	enum phy_mode mode;
	int ret;

	/*
	 * HW ver 5 can only support up to HS-G5 Rate-A due to HW limitations.
	 * If the HS-G5 PHY gear is used, update host_params->hs_rate to Rate-A,
	 * so that the subsequent power mode change shall stick to Rate-A.
	 */
	if (host->hw_ver.major == 0x5) {
		if (host->phy_gear == UFS_HS_G5)
			host_params->hs_rate = PA_HS_MODE_A;
		else
			host_params->hs_rate = PA_HS_MODE_B;
	}

	mode = host_params->hs_rate == PA_HS_MODE_B ? PHY_MODE_UFS_HS_B : PHY_MODE_UFS_HS_A;

	/* Reset UFS Host Controller and PHY */
	ret = ufs_qcom_host_reset(hba);
	if (ret)
		return ret;

	if (phy->power_count)
		phy_power_off(phy);


	/* phy initialization - calibrate the phy */
	ret = phy_init(phy);
	if (ret) {
		dev_err(hba->dev, "%s: phy init failed, ret = %d\n",
			__func__, ret);
		return ret;
	}

	ret = phy_set_mode_ext(phy, mode, host->phy_gear);
	if (ret)
		goto out_disable_phy;

	/* power on phy - start serdes and phy's power and clocks */
	ret = phy_power_on(phy);
	if (ret) {
		dev_err(hba->dev, "%s: phy power on failed, ret = %d\n",
			__func__, ret);
		goto out_disable_phy;
	}

	ret = phy_calibrate(phy);
	if (ret) {
		dev_err(hba->dev, "Failed to calibrate PHY: %d\n", ret);
		goto out_disable_phy;
	}

	ufs_qcom_select_unipro_mode(host);

	return 0;

out_disable_phy:
	phy_exit(phy);

	return ret;
}

/*
 * The UTP controller has a number of internal clock gating cells (CGCs).
 * Internal hardware sub-modules within the UTP controller control the CGCs.
 * Hardware CGCs disable the clock to inactivate UTP sub-modules not involved
 * in a specific operation, UTP controller CGCs are by default disabled and
 * this function enables them (after every UFS link startup) to save some power
 * leakage.
 */
static void ufs_qcom_enable_hw_clk_gating(struct ufs_hba *hba)
{
	int err;

	/* Enable UTP internal clock gating */
	ufshcd_rmwl(hba, REG_UFS_CFG2_CGC_EN_ALL, REG_UFS_CFG2_CGC_EN_ALL,
		    REG_UFS_CFG2);

	/* Ensure that HW clock gating is enabled before next operations */
	ufshcd_readl(hba, REG_UFS_CFG2);

	/* Enable Unipro internal clock gating */
	err = ufshcd_dme_rmw(hba, DL_VS_CLK_CFG_MASK,
			     DL_VS_CLK_CFG_MASK, DL_VS_CLK_CFG);
	if (err)
		goto out;

	err = ufshcd_dme_rmw(hba, PA_VS_CLK_CFG_REG_MASK,
			     PA_VS_CLK_CFG_REG_MASK, PA_VS_CLK_CFG_REG);
	if (err)
		goto out;

	err = ufshcd_dme_rmw(hba, DME_VS_CORE_CLK_CTRL_DME_HW_CGC_EN,
			     DME_VS_CORE_CLK_CTRL_DME_HW_CGC_EN,
			     DME_VS_CORE_CLK_CTRL);
out:
	if (err)
		dev_err(hba->dev, "hw clk gating enabled failed\n");
}

static int ufs_qcom_hce_enable_notify(struct ufs_hba *hba,
				      enum ufs_notify_change_status status)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int err;

	switch (status) {
	case PRE_CHANGE:
		err = ufs_qcom_power_up_sequence(hba);
		if (err)
			return err;

		/*
		 * The PHY PLL output is the source of tx/rx lane symbol
		 * clocks, hence, enable the lane clocks only after PHY
		 * is initialized.
		 */
		err = ufs_qcom_enable_lane_clks(host);
		break;
	case POST_CHANGE:
		/* check if UFS PHY moved from DISABLED to HIBERN8 */
		err = ufs_qcom_check_hibern8(hba);
		ufs_qcom_enable_hw_clk_gating(hba);
		ufs_qcom_ice_enable(host);
		ufs_qcom_config_ice_allocator(host);
		break;
	default:
		dev_err(hba->dev, "%s: invalid status %d\n", __func__, status);
		err = -EINVAL;
		break;
	}
	return err;
}

/**
 * ufs_qcom_cfg_timers - Configure ufs qcom cfg timers
 *
 * @hba: host controller instance
 * @is_pre_scale_up: flag to check if pre scale up condition.
 * @freq: target opp freq
 * Return: zero for success and non-zero in case of a failure.
 */
static int ufs_qcom_cfg_timers(struct ufs_hba *hba, bool is_pre_scale_up, unsigned long freq)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct ufs_clk_info *clki;
	unsigned long clk_freq = 0;
	u32 core_clk_cycles_per_us;

	/*
	 * UTP controller uses SYS1CLK_1US_REG register for Interrupt
	 * Aggregation logic.
	 * It is mandatory to write SYS1CLK_1US_REG register on UFS host
	 * controller V4.0.0 onwards.
	 */
	if (host->hw_ver.major < 4 && !ufshcd_is_intr_aggr_allowed(hba))
		return 0;

	if (hba->use_pm_opp && freq != ULONG_MAX) {
		clk_freq = ufs_qcom_opp_freq_to_clk_freq(hba, freq, "core_clk");
		if (clk_freq)
			goto cfg_timers;
	}

	list_for_each_entry(clki, &hba->clk_list_head, list) {
		if (!strcmp(clki->name, "core_clk")) {
			if (freq == ULONG_MAX) {
				clk_freq = clki->max_freq;
				break;
			}

			if (is_pre_scale_up)
				clk_freq = clki->max_freq;
			else
				clk_freq = clk_get_rate(clki->clk);
			break;
		}

	}

cfg_timers:
	/* If frequency is smaller than 1MHz, set to 1MHz */
	if (clk_freq < DEFAULT_CLK_RATE_HZ)
		clk_freq = DEFAULT_CLK_RATE_HZ;

	core_clk_cycles_per_us = clk_freq / USEC_PER_SEC;
	if (ufshcd_readl(hba, REG_UFS_SYS1CLK_1US) != core_clk_cycles_per_us) {
		ufshcd_writel(hba, core_clk_cycles_per_us, REG_UFS_SYS1CLK_1US);
		/*
		 * make sure above write gets applied before we return from
		 * this function.
		 */
		ufshcd_readl(hba, REG_UFS_SYS1CLK_1US);
	}

	return 0;
}

static int ufs_qcom_link_startup_notify(struct ufs_hba *hba,
					enum ufs_notify_change_status status)
{
	int err = 0;

	switch (status) {
	case PRE_CHANGE:
		if (ufs_qcom_cfg_timers(hba, false, ULONG_MAX)) {
			dev_err(hba->dev, "%s: ufs_qcom_cfg_timers() failed\n",
				__func__);
			return -EINVAL;
		}

		err = ufs_qcom_set_core_clk_ctrl(hba, true, ULONG_MAX);
		if (err)
			dev_err(hba->dev, "cfg core clk ctrl failed\n");
		/*
		 * Some UFS devices (and may be host) have issues if LCC is
		 * enabled. So we are setting PA_Local_TX_LCC_Enable to 0
		 * before link startup which will make sure that both host
		 * and device TX LCC are disabled once link startup is
		 * completed.
		 */
		err = ufshcd_disable_host_tx_lcc(hba);

		break;
	default:
		break;
	}

	return err;
}

static void ufs_qcom_device_reset_ctrl(struct ufs_hba *hba, bool asserted)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	/* reset gpio is optional */
	if (!host->device_reset)
		return;

	gpiod_set_value_cansleep(host->device_reset, asserted);
}

static int ufs_qcom_suspend(struct ufs_hba *hba, enum ufs_pm_op pm_op,
	enum ufs_notify_change_status status)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	if (status == PRE_CHANGE)
		return 0;

	if (!ufs_qcom_is_link_active(hba))
		ufs_qcom_disable_lane_clks(host);


	/* reset the connected UFS device during power down */
	if (ufs_qcom_is_link_off(hba) && host->device_reset)
		ufs_qcom_device_reset_ctrl(hba, true);

	return ufs_qcom_ice_suspend(host);
}

static int ufs_qcom_resume(struct ufs_hba *hba, enum ufs_pm_op pm_op)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int err;

	err = ufs_qcom_enable_lane_clks(host);
	if (err)
		return err;

	return ufs_qcom_ice_resume(host);
}

static void ufs_qcom_dev_ref_clk_ctrl(struct ufs_qcom_host *host, bool enable)
{
	if (host->dev_ref_clk_ctrl_mmio &&
	    (enable ^ host->is_dev_ref_clk_enabled)) {
		u32 temp = readl_relaxed(host->dev_ref_clk_ctrl_mmio);

		if (enable)
			temp |= host->dev_ref_clk_en_mask;
		else
			temp &= ~host->dev_ref_clk_en_mask;

		/*
		 * If we are here to disable this clock it might be immediately
		 * after entering into hibern8 in which case we need to make
		 * sure that device ref_clk is active for specific time after
		 * hibern8 enter.
		 */
		if (!enable) {
			unsigned long gating_wait;

			gating_wait = host->hba->dev_info.clk_gating_wait_us;
			if (!gating_wait) {
				udelay(1);
			} else {
				/*
				 * bRefClkGatingWaitTime defines the minimum
				 * time for which the reference clock is
				 * required by device during transition from
				 * HS-MODE to LS-MODE or HIBERN8 state. Give it
				 * more delay to be on the safe side.
				 */
				gating_wait += 10;
				usleep_range(gating_wait, gating_wait + 10);
			}
		}

		writel_relaxed(temp, host->dev_ref_clk_ctrl_mmio);

		/*
		 * Make sure the write to ref_clk reaches the destination and
		 * not stored in a Write Buffer (WB).
		 */
		readl(host->dev_ref_clk_ctrl_mmio);

		/*
		 * If we call hibern8 exit after this, we need to make sure that
		 * device ref_clk is stable for at least 1us before the hibern8
		 * exit command.
		 */
		if (enable)
			udelay(1);

		host->is_dev_ref_clk_enabled = enable;
	}
}

static int ufs_qcom_icc_set_bw(struct ufs_qcom_host *host, u32 mem_bw, u32 cfg_bw)
{
	struct device *dev = host->hba->dev;
	int ret;

	ret = icc_set_bw(host->icc_ddr, 0, mem_bw);
	if (ret < 0) {
		dev_err(dev, "failed to set bandwidth request: %d\n", ret);
		return ret;
	}

	ret = icc_set_bw(host->icc_cpu, 0, cfg_bw);
	if (ret < 0) {
		dev_err(dev, "failed to set bandwidth request: %d\n", ret);
		return ret;
	}

	return 0;
}

static struct __ufs_qcom_bw_table ufs_qcom_get_bw_table(struct ufs_qcom_host *host)
{
	struct ufs_pa_layer_attr *p = &host->dev_req_params;
	int gear = max_t(u32, p->gear_rx, p->gear_tx);
	int lane = max_t(u32, p->lane_rx, p->lane_tx);

	if (WARN_ONCE(gear > QCOM_UFS_MAX_GEAR,
		      "ICC scaling for UFS Gear (%d) not supported. Using Gear (%d) bandwidth\n",
		      gear, QCOM_UFS_MAX_GEAR))
		gear = QCOM_UFS_MAX_GEAR;

	if (WARN_ONCE(lane > QCOM_UFS_MAX_LANE,
		      "ICC scaling for UFS Lane (%d) not supported. Using Lane (%d) bandwidth\n",
		      lane, QCOM_UFS_MAX_LANE))
		lane = QCOM_UFS_MAX_LANE;

	if (ufshcd_is_hs_mode(p)) {
		if (p->hs_rate == PA_HS_MODE_B)
			return ufs_qcom_bw_table[MODE_HS_RB][gear][lane];
		else
			return ufs_qcom_bw_table[MODE_HS_RA][gear][lane];
	} else {
		return ufs_qcom_bw_table[MODE_PWM][gear][lane];
	}
}

static int ufs_qcom_icc_update_bw(struct ufs_qcom_host *host)
{
	struct __ufs_qcom_bw_table bw_table;

	bw_table = ufs_qcom_get_bw_table(host);

	return ufs_qcom_icc_set_bw(host, bw_table.mem_bw, bw_table.cfg_bw);
}

static void ufs_qcom_set_tx_hs_equalizer(struct ufs_hba *hba, u32 gear, u32 tx_lanes)
{
	u32 equalizer_val;
	int ret, i;

	/* Determine the equalizer value based on the gear */
	equalizer_val = (gear == 5) ? DEEMPHASIS_3_5_dB : NO_DEEMPHASIS;

	for (i = 0; i < tx_lanes; i++) {
		ret = ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_HS_EQUALIZER, i),
				     equalizer_val);
		if (ret)
			dev_err(hba->dev, "%s: failed equalizer lane %d\n",
				__func__, i);
	}
}

static int ufs_qcom_pwr_change_notify(struct ufs_hba *hba,
				enum ufs_notify_change_status status,
				const struct ufs_pa_layer_attr *dev_max_params,
				struct ufs_pa_layer_attr *dev_req_params)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct ufs_host_params *host_params = &host->host_params;
	int ret = 0;

	if (!dev_req_params) {
		pr_err("%s: incoming dev_req_params is NULL\n", __func__);
		return -EINVAL;
	}

	switch (status) {
	case PRE_CHANGE:
		ret = ufshcd_negotiate_pwr_params(host_params, dev_max_params, dev_req_params);
		if (ret) {
			dev_err(hba->dev, "%s: failed to determine capabilities\n",
					__func__);
			return ret;
		}

		/*
		 * During UFS driver probe, always update the PHY gear to match the negotiated
		 * gear, so that, if quirk UFSHCD_QUIRK_REINIT_AFTER_MAX_GEAR_SWITCH is enabled,
		 * the second init can program the optimal PHY settings. This allows one to start
		 * the first init with either the minimum or the maximum support gear.
		 */
		if (hba->ufshcd_state == UFSHCD_STATE_RESET) {
			/*
			 * Skip REINIT if the negotiated gear matches with the
			 * initial phy_gear. Otherwise, update the phy_gear to
			 * program the optimal gear setting during REINIT.
			 */
			if (host->phy_gear == dev_req_params->gear_tx)
				hba->quirks &= ~UFSHCD_QUIRK_REINIT_AFTER_MAX_GEAR_SWITCH;
			else
				host->phy_gear = dev_req_params->gear_tx;
		}

		/* enable the device ref clock before changing to HS mode */
		if (!ufshcd_is_hs_mode(&hba->pwr_info) &&
			ufshcd_is_hs_mode(dev_req_params))
			ufs_qcom_dev_ref_clk_ctrl(host, true);

		if (host->hw_ver.major >= 0x4) {
			ufshcd_dme_configure_adapt(hba,
						dev_req_params->gear_tx,
						PA_INITIAL_ADAPT);
		}

		if (hba->dev_quirks & UFS_DEVICE_QUIRK_PA_TX_DEEMPHASIS_TUNING)
			ufs_qcom_set_tx_hs_equalizer(hba,
					dev_req_params->gear_tx, dev_req_params->lane_tx);

		break;
	case POST_CHANGE:
		/* cache the power mode parameters to use internally */
		memcpy(&host->dev_req_params,
				dev_req_params, sizeof(*dev_req_params));

		ufs_qcom_icc_update_bw(host);

		/* disable the device ref clock if entered PWM mode */
		if (ufshcd_is_hs_mode(&hba->pwr_info) &&
			!ufshcd_is_hs_mode(dev_req_params))
			ufs_qcom_dev_ref_clk_ctrl(host, false);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ufs_qcom_quirk_host_pa_saveconfigtime(struct ufs_hba *hba)
{
	int err;
	u32 pa_vs_config_reg1;

	err = ufshcd_dme_get(hba, UIC_ARG_MIB(PA_VS_CONFIG_REG1),
			     &pa_vs_config_reg1);
	if (err)
		return err;

	/* Allow extension of MSB bits of PA_SaveConfigTime attribute */
	return ufshcd_dme_set(hba, UIC_ARG_MIB(PA_VS_CONFIG_REG1),
			    (pa_vs_config_reg1 | (1 << 12)));
}

static void ufs_qcom_override_pa_tx_hsg1_sync_len(struct ufs_hba *hba)
{
	int err;

	err = ufshcd_dme_peer_set(hba, UIC_ARG_MIB(PA_TX_HSG1_SYNC_LENGTH),
				  PA_TX_HSG1_SYNC_LENGTH_VAL);
	if (err)
		dev_err(hba->dev, "Failed (%d) set PA_TX_HSG1_SYNC_LENGTH\n", err);
}

static int ufs_qcom_apply_dev_quirks(struct ufs_hba *hba)
{
	int err = 0;

	if (hba->dev_quirks & UFS_DEVICE_QUIRK_HOST_PA_SAVECONFIGTIME)
		err = ufs_qcom_quirk_host_pa_saveconfigtime(hba);

	if (hba->dev_quirks & UFS_DEVICE_QUIRK_PA_TX_HSG1_SYNC_LENGTH)
		ufs_qcom_override_pa_tx_hsg1_sync_len(hba);

	return err;
}

/* UFS device-specific quirks */
static struct ufs_dev_quirk ufs_qcom_dev_fixups[] = {
	{ .wmanufacturerid = UFS_VENDOR_SKHYNIX,
	  .model = UFS_ANY_MODEL,
	  .quirk = UFS_DEVICE_QUIRK_DELAY_BEFORE_LPM },
	{ .wmanufacturerid = UFS_VENDOR_TOSHIBA,
	  .model = UFS_ANY_MODEL,
	  .quirk = UFS_DEVICE_QUIRK_DELAY_AFTER_LPM },
	{ .wmanufacturerid = UFS_VENDOR_WDC,
	  .model = UFS_ANY_MODEL,
	  .quirk = UFS_DEVICE_QUIRK_HOST_PA_TACTIVATE },
	{ .wmanufacturerid = UFS_VENDOR_SAMSUNG,
	  .model = UFS_ANY_MODEL,
	  .quirk = UFS_DEVICE_QUIRK_PA_TX_HSG1_SYNC_LENGTH |
		   UFS_DEVICE_QUIRK_PA_TX_DEEMPHASIS_TUNING },
	{}
};

static void ufs_qcom_fixup_dev_quirks(struct ufs_hba *hba)
{
	ufshcd_fixup_dev_quirks(hba, ufs_qcom_dev_fixups);
}

static u32 ufs_qcom_get_ufs_hci_version(struct ufs_hba *hba)
{
	return ufshci_version(2, 0);
}

/**
 * ufs_qcom_advertise_quirks - advertise the known QCOM UFS controller quirks
 * @hba: host controller instance
 *
 * QCOM UFS host controller might have some non standard behaviours (quirks)
 * than what is specified by UFSHCI specification. Advertise all such
 * quirks to standard UFS host controller driver so standard takes them into
 * account.
 */
static void ufs_qcom_advertise_quirks(struct ufs_hba *hba)
{
	const struct ufs_qcom_drvdata *drvdata = of_device_get_match_data(hba->dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	if (host->hw_ver.major == 0x2)
		hba->quirks |= UFSHCD_QUIRK_BROKEN_UFS_HCI_VERSION;

	if (host->hw_ver.major > 0x3)
		hba->quirks |= UFSHCD_QUIRK_REINIT_AFTER_MAX_GEAR_SWITCH;

	if (drvdata && drvdata->quirks)
		hba->quirks |= drvdata->quirks;
}

static void ufs_qcom_set_phy_gear(struct ufs_qcom_host *host)
{
	struct ufs_host_params *host_params = &host->host_params;
	u32 val, dev_major;

	/*
	 * Default to powering up the PHY to the max gear possible, which is
	 * backwards compatible with lower gears but not optimal from
	 * a power usage point of view. After device negotiation, if the
	 * gear is lower a reinit will be performed to program the PHY
	 * to the ideal gear for this combo of controller and device.
	 */
	host->phy_gear = host_params->hs_tx_gear;

	if (host->hw_ver.major < 0x4) {
		/*
		 * These controllers only have one PHY init sequence,
		 * let's power up the PHY using that (the minimum supported
		 * gear, UFS_HS_G2).
		 */
		host->phy_gear = UFS_HS_G2;
	} else if (host->hw_ver.major >= 0x5) {
		val = ufshcd_readl(host->hba, REG_UFS_DEBUG_SPARE_CFG);
		dev_major = FIELD_GET(UFS_DEV_VER_MAJOR_MASK, val);

		/*
		 * Since the UFS device version is populated, let's remove the
		 * REINIT quirk as the negotiated gear won't change during boot.
		 * So there is no need to do reinit.
		 */
		if (dev_major != 0x0)
			host->hba->quirks &= ~UFSHCD_QUIRK_REINIT_AFTER_MAX_GEAR_SWITCH;

		/*
		 * For UFS 3.1 device and older, power up the PHY using HS-G4
		 * PHY gear to save power.
		 */
		if (dev_major > 0x0 && dev_major < 0x4)
			host->phy_gear = UFS_HS_G4;
	}
}

static void ufs_qcom_set_host_params(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct ufs_host_params *host_params = &host->host_params;

	ufshcd_init_host_params(host_params);

	/* This driver only supports symmetic gear setting i.e., hs_tx_gear == hs_rx_gear */
	host_params->hs_tx_gear = host_params->hs_rx_gear = ufs_qcom_get_hs_gear(hba);
}

static void ufs_qcom_set_host_caps(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	if (host->hw_ver.major >= 0x5)
		host->caps |= UFS_QCOM_CAP_ICE_CONFIG;
}

static void ufs_qcom_set_caps(struct ufs_hba *hba)
{
	hba->caps |= UFSHCD_CAP_CLK_GATING | UFSHCD_CAP_HIBERN8_WITH_CLK_GATING;
	hba->caps |= UFSHCD_CAP_CLK_SCALING | UFSHCD_CAP_WB_WITH_CLK_SCALING;
	hba->caps |= UFSHCD_CAP_AUTO_BKOPS_SUSPEND;
	hba->caps |= UFSHCD_CAP_WB_EN;
	hba->caps |= UFSHCD_CAP_AGGR_POWER_COLLAPSE;
	hba->caps |= UFSHCD_CAP_RPM_AUTOSUSPEND;

	ufs_qcom_set_host_caps(hba);
}

/**
 * ufs_qcom_setup_clocks - enables/disable clocks
 * @hba: host controller instance
 * @on: If true, enable clocks else disable them.
 * @status: PRE_CHANGE or POST_CHANGE notify
 *
 * There are certain clocks which comes from the PHY so it needs
 * to be managed together along with controller clocks which also
 * provides a better power saving. Hence keep phy_power_off/on calls
 * in ufs_qcom_setup_clocks, so that PHY's regulators & clks can be
 * turned on/off along with UFS's clocks.
 *
 * Return: 0 on success, non-zero on failure.
 */
static int ufs_qcom_setup_clocks(struct ufs_hba *hba, bool on,
				 enum ufs_notify_change_status status)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct phy *phy;
	int err;

	/*
	 * In case ufs_qcom_init() is not yet done, simply ignore.
	 * This ufs_qcom_setup_clocks() shall be called from
	 * ufs_qcom_init() after init is done.
	 */
	if (!host)
		return 0;

	phy = host->generic_phy;

	switch (status) {
	case PRE_CHANGE:
		if (on) {
			ufs_qcom_icc_update_bw(host);
		} else {
			if (!ufs_qcom_is_link_active(hba)) {
				/* disable device ref_clk */
				ufs_qcom_dev_ref_clk_ctrl(host, false);
			}

			err = phy_power_off(phy);
			if (err) {
				dev_err(hba->dev, "phy power off failed, ret=%d\n", err);
				return err;
			}
		}
		break;
	case POST_CHANGE:
		if (on) {
			err = phy_power_on(phy);
			if (err) {
				dev_err(hba->dev, "phy power on failed, ret = %d\n", err);
				return err;
			}

			/* enable the device ref clock for HS mode*/
			if (ufshcd_is_hs_mode(&hba->pwr_info))
				ufs_qcom_dev_ref_clk_ctrl(host, true);
		} else {
			ufs_qcom_icc_set_bw(host, ufs_qcom_bw_table[MODE_MIN][0][0].mem_bw,
					    ufs_qcom_bw_table[MODE_MIN][0][0].cfg_bw);
		}
		break;
	}

	return 0;
}

static int
ufs_qcom_reset_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct ufs_qcom_host *host = rcdev_to_ufs_host(rcdev);

	ufs_qcom_assert_reset(host->hba);
	/* provide 1ms delay to let the reset pulse propagate. */
	usleep_range(1000, 1100);
	return 0;
}

static int
ufs_qcom_reset_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct ufs_qcom_host *host = rcdev_to_ufs_host(rcdev);

	ufs_qcom_deassert_reset(host->hba);

	/*
	 * after reset deassertion, phy will need all ref clocks,
	 * voltage, current to settle down before starting serdes.
	 */
	usleep_range(1000, 1100);
	return 0;
}

static const struct reset_control_ops ufs_qcom_reset_ops = {
	.assert = ufs_qcom_reset_assert,
	.deassert = ufs_qcom_reset_deassert,
};

static int ufs_qcom_icc_init(struct ufs_qcom_host *host)
{
	struct device *dev = host->hba->dev;
	int ret;

	host->icc_ddr = devm_of_icc_get(dev, "ufs-ddr");
	if (IS_ERR(host->icc_ddr))
		return dev_err_probe(dev, PTR_ERR(host->icc_ddr),
				    "failed to acquire interconnect path\n");

	host->icc_cpu = devm_of_icc_get(dev, "cpu-ufs");
	if (IS_ERR(host->icc_cpu))
		return dev_err_probe(dev, PTR_ERR(host->icc_cpu),
				    "failed to acquire interconnect path\n");

	/*
	 * Set Maximum bandwidth vote before initializing the UFS controller and
	 * device. Ideally, a minimal interconnect vote would suffice for the
	 * initialization, but a max vote would allow faster initialization.
	 */
	ret = ufs_qcom_icc_set_bw(host, ufs_qcom_bw_table[MODE_MAX][0][0].mem_bw,
				  ufs_qcom_bw_table[MODE_MAX][0][0].cfg_bw);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to set bandwidth request\n");

	return 0;
}

/**
 * ufs_qcom_init - bind phy with controller
 * @hba: host controller instance
 *
 * Binds PHY with controller and powers up PHY enabling clocks
 * and regulators.
 *
 * Return: -EPROBE_DEFER if binding fails, returns negative error
 * on phy power up failure and returns zero on success.
 */
static int ufs_qcom_init(struct ufs_hba *hba)
{
	int err;
	struct device *dev = hba->dev;
	struct ufs_qcom_host *host;
	struct ufs_clk_info *clki;
	const struct ufs_qcom_drvdata *drvdata = of_device_get_match_data(hba->dev);

	host = devm_kzalloc(dev, sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	/* Make a two way bind between the qcom host and the hba */
	host->hba = hba;
	ufshcd_set_variant(hba, host);

	/* Setup the optional reset control of HCI */
	host->core_reset = devm_reset_control_get_optional(hba->dev, "rst");
	if (IS_ERR(host->core_reset)) {
		err = dev_err_probe(dev, PTR_ERR(host->core_reset),
				    "Failed to get reset control\n");
		goto out_variant_clear;
	}

	/* Fire up the reset controller. Failure here is non-fatal. */
	host->rcdev.of_node = dev->of_node;
	host->rcdev.ops = &ufs_qcom_reset_ops;
	host->rcdev.owner = dev->driver->owner;
	host->rcdev.nr_resets = 1;
	err = devm_reset_controller_register(dev, &host->rcdev);
	if (err)
		dev_warn(dev, "Failed to register reset controller\n");

	if (!has_acpi_companion(dev)) {
		host->generic_phy = devm_phy_get(dev, "ufsphy");
		if (IS_ERR(host->generic_phy)) {
			err = dev_err_probe(dev, PTR_ERR(host->generic_phy), "Failed to get PHY\n");
			goto out_variant_clear;
		}
	}

	err = ufs_qcom_icc_init(host);
	if (err)
		goto out_variant_clear;

	host->device_reset = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(host->device_reset)) {
		err = dev_err_probe(dev, PTR_ERR(host->device_reset),
				    "Failed to acquire device reset gpio\n");
		goto out_variant_clear;
	}

	ufs_qcom_get_controller_revision(hba, &host->hw_ver.major,
		&host->hw_ver.minor, &host->hw_ver.step);

	host->dev_ref_clk_ctrl_mmio = hba->mmio_base + REG_UFS_CFG1;
	host->dev_ref_clk_en_mask = BIT(26);

	list_for_each_entry(clki, &hba->clk_list_head, list) {
		if (!strcmp(clki->name, "core_clk_unipro"))
			clki->keep_link_active = true;
	}

	err = ufs_qcom_init_lane_clks(host);
	if (err)
		goto out_variant_clear;

	ufs_qcom_set_caps(hba);
	ufs_qcom_advertise_quirks(hba);
	ufs_qcom_set_host_params(hba);
	ufs_qcom_set_phy_gear(host);

	err = ufs_qcom_ice_init(host);
	if (err)
		goto out_variant_clear;

	ufs_qcom_setup_clocks(hba, true, POST_CHANGE);

	ufs_qcom_get_default_testbus_cfg(host);
	err = ufs_qcom_testbus_config(host);
	if (err)
		/* Failure is non-fatal */
		dev_warn(dev, "%s: failed to configure the testbus %d\n",
				__func__, err);

	if (drvdata && drvdata->no_phy_retention)
		hba->spm_lvl = UFS_PM_LVL_5;

	return 0;

out_variant_clear:
	ufshcd_set_variant(hba, NULL);

	return err;
}

static void ufs_qcom_exit(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	ufs_qcom_disable_lane_clks(host);
	phy_power_off(host->generic_phy);
	phy_exit(host->generic_phy);
}

/**
 * ufs_qcom_set_clk_40ns_cycles - Configure 40ns clk cycles
 *
 * @hba: host controller instance
 * @cycles_in_1us: No of cycles in 1us to be configured
 *
 * Returns error if dme get/set configuration for 40ns fails
 * and returns zero on success.
 */
static int ufs_qcom_set_clk_40ns_cycles(struct ufs_hba *hba,
					u32 cycles_in_1us)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	u32 cycles_in_40ns;
	u32 reg;
	int err;

	/*
	 * UFS host controller V4.0.0 onwards needs to program
	 * PA_VS_CORE_CLK_40NS_CYCLES attribute per programmed
	 * frequency of unipro core clk of UFS host controller.
	 */
	if (host->hw_ver.major < 4)
		return 0;

	/*
	 * Generic formulae for cycles_in_40ns = (freq_unipro/25) is not
	 * applicable for all frequencies. For ex: ceil(37.5 MHz/25) will
	 * be 2 and ceil(403 MHZ/25) will be 17 whereas Hardware
	 * specification expect to be 16. Hence use exact hardware spec
	 * mandated value for cycles_in_40ns instead of calculating using
	 * generic formulae.
	 */
	switch (cycles_in_1us) {
	case UNIPRO_CORE_CLK_FREQ_403_MHZ:
		cycles_in_40ns = 16;
		break;
	case UNIPRO_CORE_CLK_FREQ_300_MHZ:
		cycles_in_40ns = 12;
		break;
	case UNIPRO_CORE_CLK_FREQ_201_5_MHZ:
		cycles_in_40ns = 8;
		break;
	case UNIPRO_CORE_CLK_FREQ_150_MHZ:
		cycles_in_40ns = 6;
		break;
	case UNIPRO_CORE_CLK_FREQ_100_MHZ:
		cycles_in_40ns = 4;
		break;
	case  UNIPRO_CORE_CLK_FREQ_75_MHZ:
		cycles_in_40ns = 3;
		break;
	case UNIPRO_CORE_CLK_FREQ_37_5_MHZ:
		cycles_in_40ns = 2;
		break;
	default:
		dev_err(hba->dev, "UNIPRO clk freq %u MHz not supported\n",
				cycles_in_1us);
		return -EINVAL;
	}

	err = ufshcd_dme_get(hba, UIC_ARG_MIB(PA_VS_CORE_CLK_40NS_CYCLES), &reg);
	if (err)
		return err;

	reg &= ~PA_VS_CORE_CLK_40NS_CYCLES_MASK;
	reg |= cycles_in_40ns;

	return ufshcd_dme_set(hba, UIC_ARG_MIB(PA_VS_CORE_CLK_40NS_CYCLES), reg);
}

static int ufs_qcom_set_core_clk_ctrl(struct ufs_hba *hba, bool is_scale_up, unsigned long freq)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct list_head *head = &hba->clk_list_head;
	struct ufs_clk_info *clki;
	u32 cycles_in_1us = 0;
	u32 core_clk_ctrl_reg;
	unsigned long clk_freq;
	int err;

	if (hba->use_pm_opp && freq != ULONG_MAX) {
		clk_freq = ufs_qcom_opp_freq_to_clk_freq(hba, freq, "core_clk_unipro");
		if (clk_freq) {
			cycles_in_1us = ceil(clk_freq, HZ_PER_MHZ);
			goto set_core_clk_ctrl;
		}
	}

	list_for_each_entry(clki, head, list) {
		if (!IS_ERR_OR_NULL(clki->clk) &&
		    !strcmp(clki->name, "core_clk_unipro")) {
			if (!clki->max_freq) {
				cycles_in_1us = 150; /* default for backwards compatibility */
				break;
			}

			if (freq == ULONG_MAX) {
				cycles_in_1us = ceil(clki->max_freq, HZ_PER_MHZ);
				break;
			}

			if (is_scale_up)
				cycles_in_1us = ceil(clki->max_freq, HZ_PER_MHZ);
			else
				cycles_in_1us = ceil(clk_get_rate(clki->clk), HZ_PER_MHZ);
			break;
		}
	}

set_core_clk_ctrl:
	err = ufshcd_dme_get(hba,
			    UIC_ARG_MIB(DME_VS_CORE_CLK_CTRL),
			    &core_clk_ctrl_reg);
	if (err)
		return err;

	/* Bit mask is different for UFS host controller V4.0.0 onwards */
	if (host->hw_ver.major >= 4) {
		if (!FIELD_FIT(CLK_1US_CYCLES_MASK_V4, cycles_in_1us))
			return -ERANGE;
		core_clk_ctrl_reg &= ~CLK_1US_CYCLES_MASK_V4;
		core_clk_ctrl_reg |= FIELD_PREP(CLK_1US_CYCLES_MASK_V4, cycles_in_1us);
	} else {
		if (!FIELD_FIT(CLK_1US_CYCLES_MASK, cycles_in_1us))
			return -ERANGE;
		core_clk_ctrl_reg &= ~CLK_1US_CYCLES_MASK;
		core_clk_ctrl_reg |= FIELD_PREP(CLK_1US_CYCLES_MASK, cycles_in_1us);
	}

	/* Clear CORE_CLK_DIV_EN */
	core_clk_ctrl_reg &= ~DME_VS_CORE_CLK_CTRL_CORE_CLK_DIV_EN_BIT;

	err = ufshcd_dme_set(hba,
			    UIC_ARG_MIB(DME_VS_CORE_CLK_CTRL),
			    core_clk_ctrl_reg);
	if (err)
		return err;

	/* Configure unipro core clk 40ns attribute */
	return ufs_qcom_set_clk_40ns_cycles(hba, cycles_in_1us);
}

static int ufs_qcom_clk_scale_up_pre_change(struct ufs_hba *hba, unsigned long freq)
{
	int ret;

	ret = ufs_qcom_cfg_timers(hba, true, freq);
	if (ret) {
		dev_err(hba->dev, "%s ufs cfg timer failed\n", __func__);
		return ret;
	}
	/* set unipro core clock attributes and clear clock divider */
	return ufs_qcom_set_core_clk_ctrl(hba, true, freq);
}

static int ufs_qcom_clk_scale_up_post_change(struct ufs_hba *hba)
{
	return 0;
}

static int ufs_qcom_clk_scale_down_pre_change(struct ufs_hba *hba)
{
	int err;
	u32 core_clk_ctrl_reg;

	err = ufshcd_dme_get(hba,
			    UIC_ARG_MIB(DME_VS_CORE_CLK_CTRL),
			    &core_clk_ctrl_reg);

	/* make sure CORE_CLK_DIV_EN is cleared */
	if (!err &&
	    (core_clk_ctrl_reg & DME_VS_CORE_CLK_CTRL_CORE_CLK_DIV_EN_BIT)) {
		core_clk_ctrl_reg &= ~DME_VS_CORE_CLK_CTRL_CORE_CLK_DIV_EN_BIT;
		err = ufshcd_dme_set(hba,
				    UIC_ARG_MIB(DME_VS_CORE_CLK_CTRL),
				    core_clk_ctrl_reg);
	}

	return err;
}

static int ufs_qcom_clk_scale_down_post_change(struct ufs_hba *hba, unsigned long freq)
{
	int ret;

	ret = ufs_qcom_cfg_timers(hba, false, freq);
	if (ret) {
		dev_err(hba->dev, "%s: ufs_qcom_cfg_timers() failed\n",	__func__);
		return ret;
	}
	/* set unipro core clock attributes and clear clock divider */
	return ufs_qcom_set_core_clk_ctrl(hba, false, freq);
}

static int ufs_qcom_clk_scale_notify(struct ufs_hba *hba, bool scale_up,
				     unsigned long target_freq,
				     enum ufs_notify_change_status status)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int err;

	/* check the host controller state before sending hibern8 cmd */
	if (!ufshcd_is_hba_active(hba))
		return 0;

	if (status == PRE_CHANGE) {
		err = ufshcd_uic_hibern8_enter(hba);
		if (err)
			return err;
		if (scale_up)
			err = ufs_qcom_clk_scale_up_pre_change(hba, target_freq);
		else
			err = ufs_qcom_clk_scale_down_pre_change(hba);

		if (err) {
			ufshcd_uic_hibern8_exit(hba);
			return err;
		}
	} else {
		if (scale_up)
			err = ufs_qcom_clk_scale_up_post_change(hba);
		else
			err = ufs_qcom_clk_scale_down_post_change(hba, target_freq);


		if (err) {
			ufshcd_uic_hibern8_exit(hba);
			return err;
		}

		ufs_qcom_icc_update_bw(host);
		ufshcd_uic_hibern8_exit(hba);
	}

	return 0;
}

static void ufs_qcom_enable_test_bus(struct ufs_qcom_host *host)
{
	ufshcd_rmwl(host->hba, UFS_REG_TEST_BUS_EN,
			UFS_REG_TEST_BUS_EN, REG_UFS_CFG1);
	ufshcd_rmwl(host->hba, TEST_BUS_EN, TEST_BUS_EN, REG_UFS_CFG1);
}

static void ufs_qcom_get_default_testbus_cfg(struct ufs_qcom_host *host)
{
	/* provide a legal default configuration */
	host->testbus.select_major = TSTBUS_UNIPRO;
	host->testbus.select_minor = 37;
}

static bool ufs_qcom_testbus_cfg_is_ok(struct ufs_qcom_host *host)
{
	if (host->testbus.select_major >= TSTBUS_MAX) {
		dev_err(host->hba->dev,
			"%s: UFS_CFG1[TEST_BUS_SEL} may not equal 0x%05X\n",
			__func__, host->testbus.select_major);
		return false;
	}

	return true;
}

int ufs_qcom_testbus_config(struct ufs_qcom_host *host)
{
	int reg;
	int offset;
	u32 mask = TEST_BUS_SUB_SEL_MASK;

	if (!host)
		return -EINVAL;

	if (!ufs_qcom_testbus_cfg_is_ok(host))
		return -EPERM;

	switch (host->testbus.select_major) {
	case TSTBUS_UAWM:
		reg = UFS_TEST_BUS_CTRL_0;
		offset = 24;
		break;
	case TSTBUS_UARM:
		reg = UFS_TEST_BUS_CTRL_0;
		offset = 16;
		break;
	case TSTBUS_TXUC:
		reg = UFS_TEST_BUS_CTRL_0;
		offset = 8;
		break;
	case TSTBUS_RXUC:
		reg = UFS_TEST_BUS_CTRL_0;
		offset = 0;
		break;
	case TSTBUS_DFC:
		reg = UFS_TEST_BUS_CTRL_1;
		offset = 24;
		break;
	case TSTBUS_TRLUT:
		reg = UFS_TEST_BUS_CTRL_1;
		offset = 16;
		break;
	case TSTBUS_TMRLUT:
		reg = UFS_TEST_BUS_CTRL_1;
		offset = 8;
		break;
	case TSTBUS_OCSC:
		reg = UFS_TEST_BUS_CTRL_1;
		offset = 0;
		break;
	case TSTBUS_WRAPPER:
		reg = UFS_TEST_BUS_CTRL_2;
		offset = 16;
		break;
	case TSTBUS_COMBINED:
		reg = UFS_TEST_BUS_CTRL_2;
		offset = 8;
		break;
	case TSTBUS_UTP_HCI:
		reg = UFS_TEST_BUS_CTRL_2;
		offset = 0;
		break;
	case TSTBUS_UNIPRO:
		reg = UFS_UNIPRO_CFG;
		offset = 20;
		mask = 0xFFF;
		break;
	/*
	 * No need for a default case, since
	 * ufs_qcom_testbus_cfg_is_ok() checks that the configuration
	 * is legal
	 */
	}
	mask <<= offset;
	ufshcd_rmwl(host->hba, TEST_BUS_SEL,
		    (u32)host->testbus.select_major << 19,
		    REG_UFS_CFG1);
	ufshcd_rmwl(host->hba, mask,
		    (u32)host->testbus.select_minor << offset,
		    reg);
	ufs_qcom_enable_test_bus(host);

	return 0;
}

static void ufs_qcom_dump_testbus(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int i, j, nminor = 0, testbus_len = 0;
	u32 *testbus __free(kfree) = NULL;
	char *prefix;

	testbus = kmalloc_array(256, sizeof(u32), GFP_KERNEL);
	if (!testbus)
		return;

	for (j = 0; j < TSTBUS_MAX; j++) {
		nminor = testbus_info[j].nminor;
		prefix = testbus_info[j].prefix;
		host->testbus.select_major = j;
		testbus_len = nminor * sizeof(u32);
		for (i = 0; i < nminor; i++) {
			host->testbus.select_minor = i;
			ufs_qcom_testbus_config(host);
			testbus[i] = ufshcd_readl(hba, UFS_TEST_BUS);
		}
		print_hex_dump(KERN_ERR, prefix, DUMP_PREFIX_OFFSET,
			       16, 4, testbus, testbus_len, false);
	}
}

static int ufs_qcom_dump_regs(struct ufs_hba *hba, size_t offset, size_t len,
			      const char *prefix, enum ufshcd_res id)
{
	u32 *regs __free(kfree) = NULL;
	size_t pos;

	if (offset % 4 != 0 || len % 4 != 0)
		return -EINVAL;

	regs = kzalloc(len, GFP_ATOMIC);
	if (!regs)
		return -ENOMEM;

	for (pos = 0; pos < len; pos += 4)
		regs[pos / 4] = readl(hba->res[id].base + offset + pos);

	print_hex_dump(KERN_ERR, prefix,
		       len > 4 ? DUMP_PREFIX_OFFSET : DUMP_PREFIX_NONE,
		       16, 4, regs, len, false);

	return 0;
}

static void ufs_qcom_dump_mcq_hci_regs(struct ufs_hba *hba)
{
	struct dump_info {
		size_t offset;
		size_t len;
		const char *prefix;
		enum ufshcd_res id;
	};

	struct dump_info mcq_dumps[] = {
		{0x0, 256 * 4, "MCQ HCI-0 ", RES_MCQ},
		{0x400, 256 * 4, "MCQ HCI-1 ", RES_MCQ},
		{0x0, 5 * 4, "MCQ VS-0 ", RES_MCQ_VS},
		{0x0, 256 * 4, "MCQ SQD-0 ", RES_MCQ_SQD},
		{0x400, 256 * 4, "MCQ SQD-1 ", RES_MCQ_SQD},
		{0x800, 256 * 4, "MCQ SQD-2 ", RES_MCQ_SQD},
		{0xc00, 256 * 4, "MCQ SQD-3 ", RES_MCQ_SQD},
		{0x1000, 256 * 4, "MCQ SQD-4 ", RES_MCQ_SQD},
		{0x1400, 256 * 4, "MCQ SQD-5 ", RES_MCQ_SQD},
		{0x1800, 256 * 4, "MCQ SQD-6 ", RES_MCQ_SQD},
		{0x1c00, 256 * 4, "MCQ SQD-7 ", RES_MCQ_SQD},
	};

	for (int i = 0; i < ARRAY_SIZE(mcq_dumps); i++) {
		ufs_qcom_dump_regs(hba, mcq_dumps[i].offset, mcq_dumps[i].len,
				   mcq_dumps[i].prefix, mcq_dumps[i].id);
		cond_resched();
	}
}

static void ufs_qcom_dump_dbg_regs(struct ufs_hba *hba)
{
	u32 reg;
	struct ufs_qcom_host *host;

	host = ufshcd_get_variant(hba);

	dev_err(hba->dev, "HW_H8_ENTER_CNT=%d\n", ufshcd_readl(hba, REG_UFS_HW_H8_ENTER_CNT));
	dev_err(hba->dev, "HW_H8_EXIT_CNT=%d\n", ufshcd_readl(hba, REG_UFS_HW_H8_EXIT_CNT));

	dev_err(hba->dev, "SW_H8_ENTER_CNT=%d\n", ufshcd_readl(hba, REG_UFS_SW_H8_ENTER_CNT));
	dev_err(hba->dev, "SW_H8_EXIT_CNT=%d\n", ufshcd_readl(hba, REG_UFS_SW_H8_EXIT_CNT));

	dev_err(hba->dev, "SW_AFTER_HW_H8_ENTER_CNT=%d\n",
			ufshcd_readl(hba, REG_UFS_SW_AFTER_HW_H8_ENTER_CNT));

	ufshcd_dump_regs(hba, REG_UFS_SYS1CLK_1US, 16 * 4,
			 "HCI Vendor Specific Registers ");

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_UFS_DBG_RD_REG_OCSC);
	ufshcd_dump_regs(hba, reg, 44 * 4, "UFS_UFS_DBG_RD_REG_OCSC ");

	reg = ufshcd_readl(hba, REG_UFS_CFG1);
	reg |= UTP_DBG_RAMS_EN;
	ufshcd_writel(hba, reg, REG_UFS_CFG1);

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_UFS_DBG_RD_EDTL_RAM);
	ufshcd_dump_regs(hba, reg, 32 * 4, "UFS_UFS_DBG_RD_EDTL_RAM ");

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_UFS_DBG_RD_DESC_RAM);
	ufshcd_dump_regs(hba, reg, 128 * 4, "UFS_UFS_DBG_RD_DESC_RAM ");

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_UFS_DBG_RD_PRDT_RAM);
	ufshcd_dump_regs(hba, reg, 64 * 4, "UFS_UFS_DBG_RD_PRDT_RAM ");

	/* clear bit 17 - UTP_DBG_RAMS_EN */
	ufshcd_rmwl(hba, UTP_DBG_RAMS_EN, 0, REG_UFS_CFG1);

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_DBG_RD_REG_UAWM);
	ufshcd_dump_regs(hba, reg, 4 * 4, "UFS_DBG_RD_REG_UAWM ");

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_DBG_RD_REG_UARM);
	ufshcd_dump_regs(hba, reg, 4 * 4, "UFS_DBG_RD_REG_UARM ");

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_DBG_RD_REG_TXUC);
	ufshcd_dump_regs(hba, reg, 48 * 4, "UFS_DBG_RD_REG_TXUC ");

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_DBG_RD_REG_RXUC);
	ufshcd_dump_regs(hba, reg, 27 * 4, "UFS_DBG_RD_REG_RXUC ");

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_DBG_RD_REG_DFC);
	ufshcd_dump_regs(hba, reg, 19 * 4, "UFS_DBG_RD_REG_DFC ");

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_DBG_RD_REG_TRLUT);
	ufshcd_dump_regs(hba, reg, 34 * 4, "UFS_DBG_RD_REG_TRLUT ");

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_DBG_RD_REG_TMRLUT);
	ufshcd_dump_regs(hba, reg, 9 * 4, "UFS_DBG_RD_REG_TMRLUT ");

	if (hba->mcq_enabled) {
		reg = ufs_qcom_get_debug_reg_offset(host, UFS_RD_REG_MCQ);
		ufshcd_dump_regs(hba, reg, 64 * 4, "HCI MCQ Debug Registers ");
	}

	/* ensure below dumps occur only in task context due to blocking calls. */
	if (in_task()) {
		/* Dump MCQ Host Vendor Specific Registers */
		if (hba->mcq_enabled)
			ufs_qcom_dump_mcq_hci_regs(hba);

		/* voluntarily yield the CPU as we are dumping too much data */
		ufshcd_dump_regs(hba, UFS_TEST_BUS, 4, "UFS_TEST_BUS ");
		cond_resched();
		ufs_qcom_dump_testbus(hba);
	}
}

/**
 * ufs_qcom_device_reset() - toggle the (optional) device reset line
 * @hba: per-adapter instance
 *
 * Toggles the (optional) reset line to reset the attached device.
 */
static int ufs_qcom_device_reset(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	/* reset gpio is optional */
	if (!host->device_reset)
		return -EOPNOTSUPP;

	/*
	 * The UFS device shall detect reset pulses of 1us, sleep for 10us to
	 * be on the safe side.
	 */
	ufs_qcom_device_reset_ctrl(hba, true);
	usleep_range(10, 15);

	ufs_qcom_device_reset_ctrl(hba, false);
	usleep_range(10, 15);

	return 0;
}

#if IS_ENABLED(CONFIG_DEVFREQ_GOV_SIMPLE_ONDEMAND)
static void ufs_qcom_config_scaling_param(struct ufs_hba *hba,
					struct devfreq_dev_profile *p,
					struct devfreq_simple_ondemand_data *d)
{
	p->polling_ms = 60;
	p->timer = DEVFREQ_TIMER_DELAYED;
	d->upthreshold = 70;
	d->downdifferential = 5;

	hba->clk_scaling.suspend_on_no_request = true;
}
#else
static void ufs_qcom_config_scaling_param(struct ufs_hba *hba,
		struct devfreq_dev_profile *p,
		struct devfreq_simple_ondemand_data *data)
{
}
#endif

/* Resources */
static const struct ufshcd_res_info ufs_res_info[RES_MAX] = {
	{.name = "ufs_mem",},
	{.name = "mcq",},
	/* Submission Queue DAO */
	{.name = "mcq_sqd",},
	/* Submission Queue Interrupt Status */
	{.name = "mcq_sqis",},
	/* Completion Queue DAO */
	{.name = "mcq_cqd",},
	/* Completion Queue Interrupt Status */
	{.name = "mcq_cqis",},
	/* MCQ vendor specific */
	{.name = "mcq_vs",},
};

static int ufs_qcom_mcq_config_resource(struct ufs_hba *hba)
{
	struct platform_device *pdev = to_platform_device(hba->dev);
	struct ufshcd_res_info *res;
	struct resource *res_mem, *res_mcq;
	int i, ret;

	memcpy(hba->res, ufs_res_info, sizeof(ufs_res_info));

	for (i = 0; i < RES_MAX; i++) {
		res = &hba->res[i];
		res->resource = platform_get_resource_byname(pdev,
							     IORESOURCE_MEM,
							     res->name);
		if (!res->resource) {
			dev_info(hba->dev, "Resource %s not provided\n", res->name);
			if (i == RES_UFS)
				return -ENODEV;
			continue;
		} else if (i == RES_UFS) {
			res_mem = res->resource;
			res->base = hba->mmio_base;
			continue;
		}

		res->base = devm_ioremap_resource(hba->dev, res->resource);
		if (IS_ERR(res->base)) {
			dev_err(hba->dev, "Failed to map res %s, err=%d\n",
					 res->name, (int)PTR_ERR(res->base));
			ret = PTR_ERR(res->base);
			res->base = NULL;
			return ret;
		}
	}

	/* MCQ resource provided in DT */
	res = &hba->res[RES_MCQ];
	/* Bail if MCQ resource is provided */
	if (res->base)
		goto out;

	/* Explicitly allocate MCQ resource from ufs_mem */
	res_mcq = devm_kzalloc(hba->dev, sizeof(*res_mcq), GFP_KERNEL);
	if (!res_mcq)
		return -ENOMEM;

	res_mcq->start = res_mem->start +
			 MCQ_SQATTR_OFFSET(hba->mcq_capabilities);
	res_mcq->end = res_mcq->start + hba->nr_hw_queues * MCQ_QCFG_SIZE - 1;
	res_mcq->flags = res_mem->flags;
	res_mcq->name = "mcq";

	ret = insert_resource(&iomem_resource, res_mcq);
	if (ret) {
		dev_err(hba->dev, "Failed to insert MCQ resource, err=%d\n",
			ret);
		return ret;
	}

	res->base = devm_ioremap_resource(hba->dev, res_mcq);
	if (IS_ERR(res->base)) {
		dev_err(hba->dev, "MCQ registers mapping failed, err=%d\n",
			(int)PTR_ERR(res->base));
		ret = PTR_ERR(res->base);
		goto ioremap_err;
	}

out:
	hba->mcq_base = res->base;
	return 0;
ioremap_err:
	res->base = NULL;
	remove_resource(res_mcq);
	return ret;
}

static int ufs_qcom_op_runtime_config(struct ufs_hba *hba)
{
	struct ufshcd_res_info *mem_res, *sqdao_res;
	struct ufshcd_mcq_opr_info_t *opr;
	int i;

	mem_res = &hba->res[RES_UFS];
	sqdao_res = &hba->res[RES_MCQ_SQD];

	if (!mem_res->base || !sqdao_res->base)
		return -EINVAL;

	for (i = 0; i < OPR_MAX; i++) {
		opr = &hba->mcq_opr[i];
		opr->offset = sqdao_res->resource->start -
			      mem_res->resource->start + 0x40 * i;
		opr->stride = 0x100;
		opr->base = sqdao_res->base + 0x40 * i;
	}

	return 0;
}

static int ufs_qcom_get_hba_mac(struct ufs_hba *hba)
{
	/* Qualcomm HC supports up to 64 */
	return MAX_SUPP_MAC;
}

static int ufs_qcom_get_outstanding_cqs(struct ufs_hba *hba,
					unsigned long *ocqs)
{
	struct ufshcd_res_info *mcq_vs_res = &hba->res[RES_MCQ_VS];

	if (!mcq_vs_res->base)
		return -EINVAL;

	*ocqs = readl(mcq_vs_res->base + UFS_MEM_CQIS_VS);

	return 0;
}

static void ufs_qcom_write_msi_msg(struct msi_desc *desc, struct msi_msg *msg)
{
	struct device *dev = msi_desc_to_dev(desc);
	struct ufs_hba *hba = dev_get_drvdata(dev);

	ufshcd_mcq_config_esi(hba, msg);
}

struct ufs_qcom_irq {
	unsigned int		irq;
	unsigned int		idx;
	struct ufs_hba		*hba;
};

static irqreturn_t ufs_qcom_mcq_esi_handler(int irq, void *data)
{
	struct ufs_qcom_irq *qi = data;
	struct ufs_hba *hba = qi->hba;
	struct ufs_hw_queue *hwq = &hba->uhq[qi->idx];

	ufshcd_mcq_write_cqis(hba, 0x1, qi->idx);
	ufshcd_mcq_poll_cqe_lock(hba, hwq);

	return IRQ_HANDLED;
}

static void ufs_qcom_irq_free(struct ufs_qcom_irq *uqi)
{
	for (struct ufs_qcom_irq *q = uqi; q->irq; q++)
		devm_free_irq(q->hba->dev, q->irq, q->hba);

	platform_device_msi_free_irqs_all(uqi->hba->dev);
	devm_kfree(uqi->hba->dev, uqi);
}

DEFINE_FREE(ufs_qcom_irq, struct ufs_qcom_irq *, if (_T) ufs_qcom_irq_free(_T))

static int ufs_qcom_config_esi(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int nr_irqs, ret;

	if (host->esi_enabled)
		return 0;

	/*
	 * 1. We only handle CQs as of now.
	 * 2. Poll queues do not need ESI.
	 */
	nr_irqs = hba->nr_hw_queues - hba->nr_queues[HCTX_TYPE_POLL];

	struct ufs_qcom_irq *qi __free(ufs_qcom_irq) =
		devm_kcalloc(hba->dev, nr_irqs, sizeof(*qi), GFP_KERNEL);
	if (!qi)
		return -ENOMEM;
	/* Preset so __free() has a pointer to hba in all error paths */
	qi[0].hba = hba;

	ret = platform_device_msi_init_and_alloc_irqs(hba->dev, nr_irqs,
						      ufs_qcom_write_msi_msg);
	if (ret) {
		dev_err(hba->dev, "Failed to request Platform MSI %d\n", ret);
		return ret;
	}

	for (int idx = 0; idx < nr_irqs; idx++) {
		qi[idx].irq = msi_get_virq(hba->dev, idx);
		qi[idx].idx = idx;
		qi[idx].hba = hba;

		ret = devm_request_irq(hba->dev, qi[idx].irq, ufs_qcom_mcq_esi_handler,
				       IRQF_SHARED, "qcom-mcq-esi", qi + idx);
		if (ret) {
			dev_err(hba->dev, "%s: Fail to request IRQ for %d, err = %d\n",
				__func__, qi[idx].irq, ret);
			qi[idx].irq = 0;
			return ret;
		}
	}

	retain_and_null_ptr(qi);

	if (host->hw_ver.major >= 6) {
		ufshcd_rmwl(hba, ESI_VEC_MASK, FIELD_PREP(ESI_VEC_MASK, MAX_ESI_VEC - 1),
			    REG_UFS_CFG3);
	}
	ufshcd_mcq_enable_esi(hba);
	host->esi_enabled = true;
	return 0;
}

static unsigned long ufs_qcom_opp_freq_to_clk_freq(struct ufs_hba *hba,
						   unsigned long freq, char *name)
{
	struct ufs_clk_info *clki;
	struct dev_pm_opp *opp;
	unsigned long clk_freq;
	int idx = 0;
	bool found = false;

	opp = dev_pm_opp_find_freq_exact_indexed(hba->dev, freq, 0, true);
	if (IS_ERR(opp)) {
		dev_err(hba->dev, "Failed to find OPP for exact frequency %lu\n", freq);
		return 0;
	}

	list_for_each_entry(clki, &hba->clk_list_head, list) {
		if (!strcmp(clki->name, name)) {
			found = true;
			break;
		}

		idx++;
	}

	if (!found) {
		dev_err(hba->dev, "Failed to find clock '%s' in clk list\n", name);
		dev_pm_opp_put(opp);
		return 0;
	}

	clk_freq = dev_pm_opp_get_freq_indexed(opp, idx);

	dev_pm_opp_put(opp);

	return clk_freq;
}

static u32 ufs_qcom_freq_to_gear_speed(struct ufs_hba *hba, unsigned long freq)
{
	u32 gear = UFS_HS_DONT_CHANGE;
	unsigned long unipro_freq;

	if (!hba->use_pm_opp)
		return gear;

	unipro_freq = ufs_qcom_opp_freq_to_clk_freq(hba, freq, "core_clk_unipro");
	switch (unipro_freq) {
	case 403000000:
		gear = UFS_HS_G5;
		break;
	case 300000000:
		gear = UFS_HS_G4;
		break;
	case 201500000:
		gear = UFS_HS_G3;
		break;
	case 150000000:
	case 100000000:
		gear = UFS_HS_G2;
		break;
	case 75000000:
	case 37500000:
		gear = UFS_HS_G1;
		break;
	default:
		dev_err(hba->dev, "%s: Unsupported clock freq : %lu\n", __func__, freq);
		return UFS_HS_DONT_CHANGE;
	}

	return min_t(u32, gear, hba->max_pwr_info.info.gear_rx);
}

/*
 * struct ufs_hba_qcom_vops - UFS QCOM specific variant operations
 *
 * The variant operations configure the necessary controller and PHY
 * handshake during initialization.
 */
static const struct ufs_hba_variant_ops ufs_hba_qcom_vops = {
	.name                   = "qcom",
	.init                   = ufs_qcom_init,
	.exit                   = ufs_qcom_exit,
	.get_ufs_hci_version	= ufs_qcom_get_ufs_hci_version,
	.clk_scale_notify	= ufs_qcom_clk_scale_notify,
	.setup_clocks           = ufs_qcom_setup_clocks,
	.hce_enable_notify      = ufs_qcom_hce_enable_notify,
	.link_startup_notify    = ufs_qcom_link_startup_notify,
	.pwr_change_notify	= ufs_qcom_pwr_change_notify,
	.apply_dev_quirks	= ufs_qcom_apply_dev_quirks,
	.fixup_dev_quirks       = ufs_qcom_fixup_dev_quirks,
	.suspend		= ufs_qcom_suspend,
	.resume			= ufs_qcom_resume,
	.dbg_register_dump	= ufs_qcom_dump_dbg_regs,
	.device_reset		= ufs_qcom_device_reset,
	.config_scaling_param = ufs_qcom_config_scaling_param,
	.mcq_config_resource	= ufs_qcom_mcq_config_resource,
	.get_hba_mac		= ufs_qcom_get_hba_mac,
	.op_runtime_config	= ufs_qcom_op_runtime_config,
	.get_outstanding_cqs	= ufs_qcom_get_outstanding_cqs,
	.config_esi		= ufs_qcom_config_esi,
	.freq_to_gear_speed	= ufs_qcom_freq_to_gear_speed,
};

/**
 * ufs_qcom_probe - probe routine of the driver
 * @pdev: pointer to Platform device handle
 *
 * Return: zero for success and non-zero for failure.
 */
static int ufs_qcom_probe(struct platform_device *pdev)
{
	int err;
	struct device *dev = &pdev->dev;

	/* Perform generic probe */
	err = ufshcd_pltfrm_init(pdev, &ufs_hba_qcom_vops);
	if (err)
		return dev_err_probe(dev, err, "ufshcd_pltfrm_init() failed\n");

	return 0;
}

/**
 * ufs_qcom_remove - set driver_data of the device to NULL
 * @pdev: pointer to platform device handle
 *
 * Always returns 0
 */
static void ufs_qcom_remove(struct platform_device *pdev)
{
	struct ufs_hba *hba =  platform_get_drvdata(pdev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	ufshcd_pltfrm_remove(pdev);
	if (host->esi_enabled)
		platform_device_msi_free_irqs_all(hba->dev);
}

static const struct ufs_qcom_drvdata ufs_qcom_sm8550_drvdata = {
	.quirks = UFSHCD_QUIRK_BROKEN_LSDBS_CAP,
	.no_phy_retention = true,
};

static const struct of_device_id ufs_qcom_of_match[] __maybe_unused = {
	{ .compatible = "qcom,ufshc" },
	{ .compatible = "qcom,sm8550-ufshc", .data = &ufs_qcom_sm8550_drvdata },
	{ .compatible = "qcom,sm8650-ufshc", .data = &ufs_qcom_sm8550_drvdata },
	{},
};
MODULE_DEVICE_TABLE(of, ufs_qcom_of_match);

#ifdef CONFIG_ACPI
static const struct acpi_device_id ufs_qcom_acpi_match[] = {
	{ "QCOM24A5" },
	{ },
};
MODULE_DEVICE_TABLE(acpi, ufs_qcom_acpi_match);
#endif

static const struct dev_pm_ops ufs_qcom_pm_ops = {
	SET_RUNTIME_PM_OPS(ufshcd_runtime_suspend, ufshcd_runtime_resume, NULL)
	.prepare	 = ufshcd_suspend_prepare,
	.complete	 = ufshcd_resume_complete,
#ifdef CONFIG_PM_SLEEP
	.suspend         = ufshcd_system_suspend,
	.resume          = ufshcd_system_resume,
	.freeze          = ufshcd_system_freeze,
	.restore         = ufshcd_system_restore,
	.thaw            = ufshcd_system_thaw,
#endif
};

static struct platform_driver ufs_qcom_pltform = {
	.probe	= ufs_qcom_probe,
	.remove = ufs_qcom_remove,
	.driver	= {
		.name	= "ufshcd-qcom",
		.pm	= &ufs_qcom_pm_ops,
		.of_match_table = of_match_ptr(ufs_qcom_of_match),
		.acpi_match_table = ACPI_PTR(ufs_qcom_acpi_match),
	},
};
module_platform_driver(ufs_qcom_pltform);

MODULE_DESCRIPTION("Qualcomm UFS host controller driver");
MODULE_LICENSE("GPL v2");
