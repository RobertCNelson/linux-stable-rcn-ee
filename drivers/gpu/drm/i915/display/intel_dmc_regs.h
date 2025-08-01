/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2022 Intel Corporation
 */

#ifndef __INTEL_DMC_REGS_H__
#define __INTEL_DMC_REGS_H__

#include "intel_display_reg_defs.h"

enum dmc_event_id {
	DMC_EVENT_TRUE = 0x0,
	DMC_EVENT_FALSE = 0x1,
};

enum maindmc_event_id {
	MAINDMC_EVENT_CMP_ZERO = 0x8,
	MAINDMC_EVENT_CMP_ODD = 0x9,
	MAINDMC_EVENT_CMP_NEG = 0xa,
	MAINDMC_EVENT_CMP_CARRY = 0xb,

	MAINDMC_EVENT_TMR0_DONE = 0x14,
	MAINDMC_EVENT_TMR1_DONE = 0x15,
	MAINDMC_EVENT_TMR2_DONE = 0x16,
	MAINDMC_EVENT_COUNT0_DONE = 0x17,
	MAINDMC_EVENT_COUNT1_DONE = 0x18,
	MAINDMC_EVENT_PERF_CNTR_DARBF = 0x19,

	MAINDMC_EVENT_SCANLINE_INRANGE_FQ_A_TRIGGER = 0x22,
	MAINDMC_EVENT_SCANLINE_INRANGE_FQ_B_TRIGGER = 0x23,
	MAINDMC_EVENT_SCANLINE_INRANGE_FQ_C_TRIGGER = 0x24,
	MAINDMC_EVENT_SCANLINE_INRANGE_FQ_D_TRIGGER = 0x25,
	MAINDMC_EVENT_1KHZ_FQ_A_TRIGGER = 0x26,
	MAINDMC_EVENT_1KHZ_FQ_B_TRIGGER = 0x27,
	MAINDMC_EVENT_1KHZ_FQ_C_TRIGGER = 0x28,
	MAINDMC_EVENT_1KHZ_FQ_D_TRIGGER = 0x29,
	MAINDMC_EVENT_SCANLINE_COMP_A = 0x2a,
	MAINDMC_EVENT_SCANLINE_COMP_B = 0x2b,
	MAINDMC_EVENT_SCANLINE_COMP_C = 0x2c,
	MAINDMC_EVENT_SCANLINE_COMP_D = 0x2d,
	MAINDMC_EVENT_VBLANK_DELAYED_A = 0x2e,
	MAINDMC_EVENT_VBLANK_DELAYED_B = 0x2f,
	MAINDMC_EVENT_VBLANK_DELAYED_C = 0x30,
	MAINDMC_EVENT_VBLANK_DELAYED_D = 0x31,
	MAINDMC_EVENT_VBLANK_A = 0x32,
	MAINDMC_EVENT_VBLANK_B = 0x33,
	MAINDMC_EVENT_VBLANK_C = 0x34,
	MAINDMC_EVENT_VBLANK_D = 0x35,
	MAINDMC_EVENT_HBLANK_A = 0x36,
	MAINDMC_EVENT_HBLANK_B = 0x37,
	MAINDMC_EVENT_HBLANK_C = 0x38,
	MAINDMC_EVENT_HBLANK_D = 0x39,
	MAINDMC_EVENT_VSYNC_A = 0x3a,
	MAINDMC_EVENT_VSYNC_B = 0x3b,
	MAINDMC_EVENT_VSYNC_C = 0x3c,
	MAINDMC_EVENT_VSYNC_D = 0x3d,
	MAINDMC_EVENT_SCANLINE_A = 0x3e,
	MAINDMC_EVENT_SCANLINE_B = 0x3f,
	MAINDMC_EVENT_SCANLINE_C = 0x40,
	MAINDMC_EVENT_SCANLINE_D = 0x41,

	MAINDMC_EVENT_PLANE1_FLIP_A = 0x42,
	MAINDMC_EVENT_PLANE2_FLIP_A = 0x43,
	MAINDMC_EVENT_PLANE3_FLIP_A = 0x44,
	MAINDMC_EVENT_PLANE4_FLIP_A = 0x45,
	MAINDMC_EVENT_PLANE5_FLIP_A = 0x46,
	MAINDMC_EVENT_PLANE6_FLIP_A = 0x47,
	MAINDMC_EVENT_PLANE7_FLIP_A = 0x48,
	MAINDMC_EVENT_PLANE1_FLIP_B = 0x49,
	MAINDMC_EVENT_PLANE2_FLIP_B = 0x4a,
	MAINDMC_EVENT_PLANE3_FLIP_B = 0x4b,
	MAINDMC_EVENT_PLANE4_FLIP_B = 0x4c,
	MAINDMC_EVENT_PLANE5_FLIP_B = 0x4d,
	MAINDMC_EVENT_PLANE6_FLIP_B = 0x4e,
	MAINDMC_EVENT_PLANE7_FLIP_B = 0x4f,
	MAINDMC_EVENT_PLANE1_FLIP_C = 0x50,
	MAINDMC_EVENT_PLANE2_FLIP_C = 0x51,
	MAINDMC_EVENT_PLANE3_FLIP_C = 0x52,
	MAINDMC_EVENT_PLANE4_FLIP_C = 0x53,
	MAINDMC_EVENT_PLANE5_FLIP_C = 0x54,
	MAINDMC_EVENT_PLANE6_FLIP_C = 0x55,
	MAINDMC_EVENT_PLANE7_FLIP_C = 0x56,
	MAINDMC_EVENT_PLANE1_FLIP_D = 0x57,
	MAINDMC_EVENT_PLANE2_FLIP_D = 0x58,
	MAINDMC_EVENT_PLANE3_FLIP_D = 0x59,
	MAINDMC_EVENT_PLANE4_FLIP_D = 0x5a,
	MAINDMC_EVENT_PLANE5_FLIP_D = 0x5b,
	MAINDMC_EVENT_PLANE6_FLIP_D = 0x5c,
	MAINDMC_EVENT_PLANE7_FLIP_D = 0x5d,
	MAINDMC_EVENT_PLANE1_FLIP_DONE_A = 0x5e,
	MAINDMC_EVENT_PLANE2_FLIP_DONE_A = 0x5f,
	MAINDMC_EVENT_PLANE3_FLIP_DONE_A = 0x60,
	MAINDMC_EVENT_PLANE4_FLIP_DONE_A = 0x61,
	MAINDMC_EVENT_PLANE5_FLIP_DONE_A = 0x62,
	MAINDMC_EVENT_PLANE6_FLIP_DONE_A = 0x63,
	MAINDMC_EVENT_PLANE7_FLIP_DONE_A = 0x64,
	MAINDMC_EVENT_PLANE1_FLIP_DONE_B = 0x65,
	MAINDMC_EVENT_PLANE2_FLIP_DONE_B = 0x66,
	MAINDMC_EVENT_PLANE3_FLIP_DONE_B = 0x67,
	MAINDMC_EVENT_PLANE4_FLIP_DONE_B = 0x68,
	MAINDMC_EVENT_PLANE5_FLIP_DONE_B = 0x69,
	MAINDMC_EVENT_PLANE6_FLIP_DONE_B = 0x6a,
	MAINDMC_EVENT_PLANE7_FLIP_DONE_B = 0x6b,
	MAINDMC_EVENT_PLANE1_FLIP_DONE_C = 0x6c,
	MAINDMC_EVENT_PLANE2_FLIP_DONE_C = 0x6d,
	MAINDMC_EVENT_PLANE3_FLIP_DONE_C = 0x6e,
	MAINDMC_EVENT_PLANE4_FLIP_DONE_C = 0x6f,
	MAINDMC_EVENT_PLANE5_FLIP_DONE_C = 0x70,
	MAINDMC_EVENT_PLANE6_FLIP_DONE_C = 0x71,
	MAINDMC_EVENT_PLANE7_FLIP_DONE_C = 0x72,
	MAINDMC_EVENT_PLANE1_FLIP_DONE_D = 0x73,
	MAINDMC_EVENT_PLANE2_FLIP_DONE_D = 0x74,
	MAINDMC_EVENT_PLANE3_FLIP_DONE_D = 0x75,
	MAINDMC_EVENT_PLANE4_FLIP_DONE_D = 0x76,
	MAINDMC_EVENT_PLANE5_FLIP_DONE_D = 0x77,
	MAINDMC_EVENT_PLANE6_FLIP_DONE_D = 0x78,
	MAINDMC_EVENT_PLANE7_FLIP_DONE_D = 0x79,

	MAINDMC_EVENT_WIDI_GTT_FAULT_SL1 = 0x7d,
	MAINDMC_EVENT_WIDI_GTT_FAULT_SL2 = 0x7e,
	MAINDMC_EVENT_WIDI_CAP_ACTIVE_SL1 = 0x7f,
	MAINDMC_EVENT_WIDI_CAP_ACTIVE_SL2 = 0x80,

	MAINDMC_EVENT_RENUKE_A = 0x85,
	MAINDMC_EVENT_RENUKE_B = 0x86,
	MAINDMC_EVENT_RENUKE_C = 0x87,
	MAINDMC_EVENT_RENUKE_D = 0x88,
	MAINDMC_EVENT_DPFC_FIFO_FULL_A = 0x89,
	MAINDMC_EVENT_DPFC_FIFO_FULL_B = 0x8a,
	MAINDMC_EVENT_DPFC_FIFO_FULL_C = 0x8b,
	MAINDMC_EVENT_DPFC_FIFO_FULL_D = 0x8c,
	MAINDMC_EVENT_DPFC_PIXEL_CNT_MISMATCH_A = 0x8d,
	MAINDMC_EVENT_DPFC_PIXEL_CNT_MISMATCH_B = 0x8e,
	MAINDMC_EVENT_DPFC_PIXEL_CNT_MISMATCH_C = 0x8f,
	MAINDMC_EVENT_DPFC_PIXEL_CNT_MISMATCH_D = 0x90,
	MAINDMC_EVENT_DPFC_COMPTAG_UNDERRUN_A = 0x91,
	MAINDMC_EVENT_DPFC_COMPTAG_UNDERRUN_B = 0x92,
	MAINDMC_EVENT_DPFC_COMPTAG_UNDERRUN_C = 0x93,
	MAINDMC_EVENT_DPFC_COMPTAG_UNDERRUN_D = 0x94,
	MAINDMC_EVENT_DPFC_FIFO_NOT_EMPTY_A = 0x95,
	MAINDMC_EVENT_DPFC_FIFO_NOT_EMPTY_B = 0x96,
	MAINDMC_EVENT_DPFC_FIFO_NOT_EMPTY_C = 0x97,
	MAINDMC_EVENT_DPFC_FIFO_NOT_EMPTY_D = 0x98,
	MAINDMC_EVENT_DPFC_COMPTAG_MISMATCH_A = 0x99,
	MAINDMC_EVENT_DPFC_COMPTAG_MISMATCH_B = 0x9a,
	MAINDMC_EVENT_DPFC_COMPTAG_MISMATCH_C = 0x9b,
	MAINDMC_EVENT_DPFC_COMPTAG_MISMATCH_D = 0x9c,
	MAINDMC_EVENT_DISP_PCH_INT = 0x9d,
	MAINDMC_EVENT_GTT_ERR = 0x9e,
	MAINDMC_EVENT_VTD_ERR = 0x9f,
	MAINDMC_EVENT_FULL_FQ_WAKE_TRIGGER_A = 0xa0,
	MAINDMC_EVENT_FULL_FQ_WAKE_TRIGGER_B = 0xa1,
	MAINDMC_EVENT_FULL_FQ_WAKE_TRIGGER_C = 0xa2,
	MAINDMC_EVENT_FULL_FQ_WAKE_TRIGGER_D = 0xa3,
	MAINDMC_EVENT_PIPEDMC_CHICKEN_FW_EVENT_A = 0xa4,
	MAINDMC_EVENT_PIPEDMC_CHICKEN_FW_EVENT_B = 0xa5,
	MAINDMC_EVENT_PIPEDMC_CHICKEN_FW_EVENT_C = 0xa6,
	MAINDMC_EVENT_PIPEDMC_CHICKEN_FW_EVENT_D = 0xa7,

	MAINDMC_EVENT_DC_CLOCK_OFF_START_EDP = 0xb2,
	MAINDMC_EVENT_DC_CLOCK_OFF_START_DSI = 0xb3,
	MAINDMC_EVENT_DCPR_DMC_CSR_START = 0xb4,
	MAINDMC_EVENT_IN_PSR = 0xb5,

	MAINDMC_EVENT_IN_MEMUP = 0xb7,
	MAINDMC_EVENT_IN_VGA = 0xb8,

	MAINDMC_EVENT_IN_KVM_SESSION = 0xba,
	MAINDMC_EVENT_DEWAKE = 0xbb,

	MAINDMC_EVENT_TRAP_HIT = 0xbd,
	MAINDMC_EVENT_CLK_USEC = 0xbe,
	MAINDMC_EVENT_CLK_MSEC = 0xbf,

	MAINDMC_EVENT_CHICKEN1 = 0xc8,
	MAINDMC_EVENT_CHICKEN2 = 0xc9,
	MAINDMC_EVENT_CHICKEN3 = 0xca,
	MAINDMC_EVENT_DDT_UBP = 0xcb,

	MAINDMC_EVENT_HP_LATENCY = 0xcd,
	MAINDMC_EVENT_LP_LATENCY = 0xce,
	MAINDMC_EVENT_WIDI_LP_REQ_SL1 = 0xcf,
	MAINDMC_EVENT_WIDI_LP_REQ_SL2 = 0xd0,

	MAINDMC_EVENT_DG_DMC_EVT_0 = 0xd3,
	MAINDMC_EVENT_DG_DMC_EVT_1 = 0xd4,
	MAINDMC_EVENT_DG_DMC_EVT_2 = 0xd5,
	MAINDMC_EVENT_DG_DMC_EVT_3 = 0xd6,
	MAINDMC_EVENT_DG_DMC_EVT_4 = 0xd7,
	MAINDMC_EVENT_DACFE_CLK_STOP = 0xd8,
	MAINDMC_EVENT_DACFE_AZILIA_SDI_WAKE = 0xd9,
	MAINDMC_EVENT_AUDIO_DOUBLE_FUNC_GRP_RST = 0xda,
	MAINDMC_EVENT_AUDIO_CMD_VALID = 0xdb,
	MAINDMC_EVENT_AUDIO_FRM_SYNC_BCLK = 0xdc,
	MAINDMC_EVENT_AUDIO_FRM_SYNC_CDCLK = 0xdd,
	MAINDMC_EVENT_AUDIO_PRESENCE_DETECT_A = 0xde,
	MAINDMC_EVENT_AUDIO_PRESENCE_DETECT_B = 0xdf,
	MAINDMC_EVENT_AUDIO_PRESENCE_DETECT_C = 0xe0,
	MAINDMC_EVENT_AUDIO_PRESENCE_DETECT_E = 0xe1,
	MAINDMC_EVENT_CMTG_SCANLINE_IN_GB_DC6v = 0xe2,
	MAINDMC_EVENT_DCPR_CMTG_SCANLINE_OUTSIDE_GB = 0xe3,
	MAINDMC_EVENT_DC6v_BACKWARD_COMPAT = 0xe4,
	MAINDMC_EVENT_DPMA_PM_ABORT = 0xe5,

	MAINDMC_EVENT_STACK_OVF = 0xfc,
	MAINDMC_EVENT_NO_CLAIM = 0xfd,
	MAINDMC_EVENT_UNK_CMD = 0xfe,
	MAINDMC_EVENT_HTP_MOD = 0xff,
};

enum pipedmc_event_id {
	PIPEDMC_EVENT_TMR0_DONE = 0x14,
	PIPEDMC_EVENT_TMR1_DONE = 0x15,
	PIPEDMC_EVENT_TMR2_DONE = 0x16,
	PIPEDMC_EVENT_COUNT0_DONE = 0x17,
	PIPEDMC_EVENT_COUNT1_DONE = 0x18,
	PIPEDMC_EVENT_PGA_PGB_RESTORE_DONE = 0x19,
	PIPEDMC_EVENT_PG1_PG2_RESTORE_DONE = 0x1a,
	PIPEDMC_EVENT_PGA_PGB_SAVE_DONE = 0x1b,
	PIPEDMC_EVENT_PG1_PG2_SAVE_DONE = 0x1c,

	PIPEDMC_EVENT_FULL_FQ_WAKE_TRIGGER = 0x2b,
	PIPEDMC_EVENT_1KHZ_FQ_TRIGGER = 0x2c,
	PIPEDMC_EVENT_SCANLINE_INRANGE_FQ_TRIGGER = 0x2d,
	PIPEDMC_EVENT_SCANLINE_INRANGE = 0x2e,
	PIPEDMC_EVENT_SCANLINE_OUTRANGE = 0x2f,
	PIPEDMC_EVENT_SCANLINE_EQUAL = 0x30,
	PIPEDMC_EVENT_DELAYED_VBLANK = 0x31,
	PIPEDMC_EVENT_VBLANK = 0x32,
	PIPEDMC_EVENT_HBLANK = 0x33,
	PIPEDMC_EVENT_VSYNC = 0x34,
	PIPEDMC_EVENT_SCANLINE_FROM_DMUX = 0x35,
	PIPEDMC_EVENT_PLANE1_FLIP = 0x36,
	PIPEDMC_EVENT_PLANE2_FLIP = 0x37,
	PIPEDMC_EVENT_PLANE3_FLIP = 0x38,
	PIPEDMC_EVENT_PLANE4_FLIP = 0x39,
	PIPEDMC_EVENT_PLANE5_FLIP = 0x3a,
	PIPEDMC_EVENT_PLANE6_FLIP = 0x3b,
	PIPEDMC_EVENT_PLANE7_FLIP = 0x3c,
	PIPEDMC_EVENT_ADAPTIVE_DCB_TRIGGER = 0x3d,

	PIPEDMC_EVENT_PLANE1_FLIP_DONE = 0x56,
	PIPEDMC_EVENT_PLANE2_FLIP_DONE = 0x57,
	PIPEDMC_EVENT_PLANE3_FLIP_DONE = 0x58,
	PIPEDMC_EVENT_PLANE4_FLIP_DONE = 0x59,
	PIPEDMC_EVENT_PLANE5_FLIP_DONE = 0x5a,
	PIPEDMC_EVENT_PLANE6_FLIP_DONE = 0x5b,
	PIPEDMC_EVENT_PLANE7_FLIP_DONE = 0x5c,

	PIPEDMC_EVENT_GTT_ERR = 0x9b,

	PIPEDMC_EVENT_IN_PSR = 0xb5,
	PIPEDMC_EVENT_DSI_DMC_IDLE = 0xb6,
	PIPEDMC_EVENT_PSR2_DMC_IDLE = 0xb7,
	PIPEDMC_EVENT_IN_VGA = 0xb8,

	PIPEDMC_EVENT_TRAP_HIT = 0xbd,
	PIPEDMC_EVENT_CLK_USEC = 0xbe,
	PIPEDMC_EVENT_CLK_MSEC = 0xbf,

	PIPEDMC_EVENT_CHICKEN1 = 0xc8,
	PIPEDMC_EVENT_CHICKEN2 = 0xc9,
	PIPEDMC_EVENT_CHICKEN3 = 0xca,
	PIPEDMC_EVENT_DDT_UBP = 0xcb,

	PIPEDMC_EVENT_LP_LATENCY = 0xce,

	PIPEDMC_EVENT_LACE_PART_A_HIST_TRIGGER = 0xdf,
	PIPEDMC_EVENT_LACE_PART_B_HIST_TRIGGER = 0xe0,

	PIPEDMC_EVENT_STACK_OVF = 0xfc,
	PIPEDMC_EVENT_NO_CLAIM = 0xfd,
	PIPEDMC_EVENT_UNK_CMD = 0xfe,
	PIPEDMC_EVENT_HTP_MOD = 0xff,
};

#define DMC_PROGRAM(addr, i)	_MMIO((addr) + (i) * 4)
#define DMC_SSP_BASE_ADDR_GEN9	0x00002FC0

#define _PIPEDMC_CONTROL_A		0x45250
#define _PIPEDMC_CONTROL_B		0x45254
#define PIPEDMC_CONTROL(pipe)		_MMIO_PIPE(pipe, \
						   _PIPEDMC_CONTROL_A, \
						   _PIPEDMC_CONTROL_B)
#define  PIPEDMC_ENABLE			REG_BIT(0)

#define MTL_PIPEDMC_CONTROL		_MMIO(0x45250)
#define  PIPEDMC_ENABLE_MTL(pipe)	REG_BIT(((pipe) - PIPE_A) * 4)

#define _PIPEDMC_LOAD_HTP_A		0x5f000
#define _PIPEDMC_LOAD_HTP_B		0x5f400
#define PIPEDMC_LOAD_HTP(pipe)		_MMIO_PIPE((pipe), _PIPEDMC_LOAD_HTP_A, _PIPEDMC_LOAD_HTP_B)

#define _PIPEDMC_CTL_A		0x5f064
#define _PIPEDMC_CTL_B		0x5f464
#define PIPEDMC_CTL(pipe)		_MMIO_PIPE((pipe), _PIPEDMC_CTL_A, _PIPEDMC_CTL_B)
#define   PIPEDMC_HALT			REG_BIT(31)
#define   PIPEDMC_STEP			REG_BIT(27)
#define   PIPEDMC_CLOCKGATE		REG_BIT(23)

#define _PIPEDMC_STATUS_A		0x5f06c
#define _PIPEDMC_STATUS_B		0x5f46c
#define PIPEDMC_STATUS(pipe)		_MMIO_PIPE((pipe), _PIPEDMC_STATUS_A, _PIPEDMC_STATUS_B)
#define   PIPEDMC_SSP			REG_GENMASK(31, 16)
#define   PIPEDMC_INT_VECTOR_MASK	REG_GENMASK(15, 8)
/* PIPEDMC_INT_VECTOR values defined by firmware */
#define   PIPEDMC_INT_VECTOR_SCANLINE_COMP_ERROR	REG_FIELD_PREP(PIPEDMC_INT_VECTOR_MASK, 0x1)
#define   PIPEDMC_INT_VECTOR_DC6V_FLIPQ_OVERLAP_ERROR	REG_FIELD_PREP(PIPEDMC_INT_VECTOR_MASK, 0x2)
#define   PIPEDMC_INT_VECTOR_FLIPQ_PROG_DONE		REG_FIELD_PREP(PIPEDMC_INT_VECTOR_MASK, 0xff) /* Wa_16018781658:lnl[a0] */
#define   PIPEDMC_EVT_PENDING		REG_GENMASK(7, 0)

#define _PIPEDMC_FQ_CTRL_A		0x5f078
#define _PIPEDMC_FQ_CTRL_B		0x5f478
#define PIPEDMC_FQ_CTRL(pipe)		_MMIO_PIPE((pipe), _PIPEDMC_FQ_CTRL_A, _PIPEDMC_FQ_CTRL_B)
#define   PIPEDMC_FQ_CTRL_ENABLE	REG_BIT(31)
#define   PIPEDMC_FQ_CTRL_ASYNC		REG_BIT(29)
#define   PIPEDMC_FQ_CTRL_PREEMPT	REG_BIT(0)

#define _PIPEDMC_FQ_STATUS_A		0x5f098
#define _PIPEDMC_FQ_STATUS_B		0x5f498
#define PIPEDMC_FQ_STATUS(pipe)		_MMIO_PIPE((pipe), _PIPEDMC_FQ_STATUS_A, _PIPEDMC_FQ_STATUS_B)
#define   PIPEDMC_FQ_STATUS_BUSY		REG_BIT(31)
#define   PIPEDMC_FQ_STATUS_W2_LIVE_STATUS	REG_BIT(1)
#define   PIPEDMC_FQ_STATUS_W1_LIVE_STATUS	REG_BIT(0)

#define _PIPEDMC_FPQ_ATOMIC_TP_A	0x5f0a0
#define _PIPEDMC_FPQ_ATOMIC_TP_B	0x5f4a0
#define PIPEDMC_FPQ_ATOMIC_TP(pipe)	_MMIO_PIPE((pipe), _PIPEDMC_FPQ_ATOMIC_TP_A, _PIPEDMC_FPQ_ATOMIC_TP_B)
#define   PIPEDMC_FPQ_PLANEQ_3_TP_MASK	REG_GENMASK(31, 26)
#define   PIPEDMC_FPQ_PLANEQ_3_TP(tail)	REG_FIELD_PREP(PIPEDMC_FPQ_PLANEQ_3_TP_MASK, (tail))
#define   PIPEDMC_FPQ_PLANEQ_2_TP_MASK	REG_GENMASK(24, 19)
#define   PIPEDMC_FPQ_PLANEQ_2_TP(tail)	REG_FIELD_PREP(PIPEDMC_FPQ_PLANEQ_2_TP_MASK, (tail))
#define   PIPEDMC_FPQ_PLANEQ_1_TP_MASK	REG_GENMASK(17, 12)
#define   PIPEDMC_FPQ_PLANEQ_1_TP(tail)	REG_FIELD_PREP(PIPEDMC_FPQ_PLANEQ_1_TP_MASK, (tail))
#define   PIPEDMC_FPQ_FASTQ_TP_MASK	REG_GENMASK(10, 6)
#define   PIPEDMC_FPQ_FASTQ_TP(tail)	REG_FIELD_PREP(PIPEDMC_FPQ_FASTQ_TP_MASK, (tail))
#define   PIPEDMC_FPQ_GENERALQ_TP_MASK	REG_GENMASK(4, 0)
#define   PIPEDMC_FPQ_GENERALQ_TP(tail)	REG_FIELD_PREP(PIPEDMC_FPQ_GENERALQ_TP_MASK, (tail))

#define _PIPEDMC_FPQ_LINES_TO_W1_A	0x5f0a4
#define _PIPEDMC_FPQ_LINES_TO_W1_B	0x5f4a4
#define PIPEDMC_FPQ_LINES_TO_W1		_MMIO_PIPE((pipe), _PIPEDMC_FPQ_LINES_TO_W1_A, _PIPEDMC_FPQ_LINES_TO_W1_B)

#define _PIPEDMC_FPQ_LINES_TO_W2_A	0x5f0a8
#define _PIPEDMC_FPQ_LINES_TO_W2_B	0x5f4a8
#define PIPEDMC_FPQ_LINES_TO_W2		_MMIO_PIPE((pipe), _PIPEDMC_FPQ_LINES_TO_W2_A, _PIPEDMC_FPQ_LINES_TO_W2_B)

#define _PIPEDMC_SCANLINECMP_A		0x5f11c
#define _PIPEDMC_SCANLINECMP_B		0x5f51c
#define PIPEDMC_SCANLINECMP(pipe)	_MMIO_PIPE((pipe), _PIPEDMC_SCANLINECMP_A, _PIPEDMC_SCANLINECMP_B)
#define   PIPEDMC_SCANLINECMP_EN	REG_BIT(31)
#define   PIPEDMC_SCANLINE_NUMBER	REG_GENMASK(20, 0)

#define _PIPEDMC_SCANLINECMPLOWER_A	0x5f120
#define _PIPEDMC_SCANLINECMPLOWER_B	0x5f520
#define PIPEDMC_SCANLINECMPLOWER(pipe)	_MMIO_PIPE((pipe), _PIPEDMC_SCANLINECMPLOWER_A, _PIPEDMC_SCANLINECMPLOWER_B)
#define   PIPEDMC_SCANLINEINRANGECMP_EN		REG_BIT(31)
#define   PIPEDMC_SCANLINEOUTRANGECMP_EN	REG_BIT(30)
#define   PIPEDMC_SCANLINE_LOWER_MASK		REG_GENMASK(20, 0)
#define   PIPEDMC_SCANLINE_LOWER(scanline)	REG_FIELD_PREP(PIPEDMC_SCANLINE_LOWER_MASK, (scanline))

#define _PIPEDMC_SCANLINECMPUPPER_A	0x5f124
#define _PIPEDMC_SCANLINECMPUPPER_B	0x5f524
#define PIPEDMC_SCANLINECMPUPPER(pipe)	_MMIO_PIPE((pipe), _PIPEDMC_SCANLINECMPUPPER_A, _PIPEDMC_SCANLINECMPUPPER_B)
#define   PIPEDMC_SCANLINE_UPPER_MASK		REG_GENMASK(20, 0)
#define   PIPEDMC_SCANLINE_UPPER(scanline)	REG_FIELD_PREP(PIPEDMC_SCANLINE_UPPER_MASK, (scanline))

#define _MMIO_PIPEDMC_FPQ(pipe, fq_id, \
			  reg_fpq1_a, reg_fpq2_a, reg_fpq3_a, reg_fpq4_a, \
			  reg_fpq1_b, reg_fpq2_b, reg_fpq3_b, reg_fpq4_b) \
	_MMIO(_PICK_EVEN_2RANGES((fq_id), INTEL_FLIPQ_PLANE_3, \
				 _PIPE((pipe), (reg_fpq1_a), (reg_fpq1_b)), \
				 _PIPE((pipe), (reg_fpq2_a), (reg_fpq2_b)), \
				 _PIPE((pipe), (reg_fpq3_a), (reg_fpq3_b)), \
				 _PIPE((pipe), (reg_fpq4_a), (reg_fpq4_b))))

#define _PIPEDMC_FPQ1_HP_A		0x5f128
#define _PIPEDMC_FPQ2_HP_A		0x5f138
#define _PIPEDMC_FPQ3_HP_A		0x5f168
#define _PIPEDMC_FPQ4_HP_A		0x5f174
#define _PIPEDMC_FPQ5_HP_A		0x5f180
#define _PIPEDMC_FPQ1_HP_B		0x5f528
#define _PIPEDMC_FPQ2_HP_B		0x5f538
#define _PIPEDMC_FPQ3_HP_B		0x5f568
#define _PIPEDMC_FPQ4_HP_B		0x5f574
#define _PIPEDMC_FPQ5_HP_B		0x5f580
#define PIPEDMC_FPQ_HP(pipe, fq_id)	_MMIO_PIPEDMC_FPQ((pipe), (fq_id), \
							  _PIPEDMC_FPQ1_HP_A, _PIPEDMC_FPQ2_HP_A, \
							  _PIPEDMC_FPQ3_HP_A, _PIPEDMC_FPQ4_HP_A, \
							  _PIPEDMC_FPQ1_HP_B, _PIPEDMC_FPQ2_HP_B, \
							  _PIPEDMC_FPQ3_HP_B, _PIPEDMC_FPQ4_HP_B)

#define _PIPEDMC_FPQ1_TP_A		0x5f12c
#define _PIPEDMC_FPQ2_TP_A		0x5f13c
#define _PIPEDMC_FPQ3_TP_A		0x5f16c
#define _PIPEDMC_FPQ4_TP_A		0x5f178
#define _PIPEDMC_FPQ5_TP_A		0x5f184
#define _PIPEDMC_FPQ1_TP_B		0x5f52c
#define _PIPEDMC_FPQ2_TP_B		0x5f53c
#define _PIPEDMC_FPQ3_TP_B		0x5f56c
#define _PIPEDMC_FPQ4_TP_B		0x5f578
#define _PIPEDMC_FPQ5_TP_B		0x5f584
#define PIPEDMC_FPQ_TP(pipe, fq_id)	_MMIO_PIPEDMC_FPQ((pipe), (fq_id), \
							  _PIPEDMC_FPQ1_TP_A, _PIPEDMC_FPQ2_TP_A, \
							  _PIPEDMC_FPQ3_TP_A, _PIPEDMC_FPQ4_TP_A, \
							  _PIPEDMC_FPQ1_TP_B, _PIPEDMC_FPQ2_TP_B, \
							  _PIPEDMC_FPQ3_TP_B, _PIPEDMC_FPQ4_TP_B)

#define _PIPEDMC_FPQ1_CHP_A		0x5f130
#define _PIPEDMC_FPQ2_CHP_A		0x5f140
#define _PIPEDMC_FPQ3_CHP_A		0x5f170
#define _PIPEDMC_FPQ4_CHP_A		0x5f17c
#define _PIPEDMC_FPQ5_CHP_A		0x5f188
#define _PIPEDMC_FPQ1_CHP_B		0x5f530
#define _PIPEDMC_FPQ2_CHP_B		0x5f540
#define _PIPEDMC_FPQ3_CHP_B		0x5f570
#define _PIPEDMC_FPQ4_CHP_B		0x5f57c
#define _PIPEDMC_FPQ5_CHP_B		0x5f588
#define PIPEDMC_FPQ_CHP(pipe, fq_id)	_MMIO_PIPEDMC_FPQ((pipe), (fq_id), \
							  _PIPEDMC_FPQ1_CHP_A, _PIPEDMC_FPQ2_CHP_A, \
							  _PIPEDMC_FPQ3_CHP_A, _PIPEDMC_FPQ4_CHP_A, \
							  _PIPEDMC_FPQ1_CHP_B, _PIPEDMC_FPQ2_CHP_B, \
							  _PIPEDMC_FPQ3_CHP_B, _PIPEDMC_FPQ4_CHP_B)

#define _PIPEDMC_FPQ_TS_A		0x5f134
#define _PIPEDMC_FPQ_TS_B		0x5f534
#define PIPEDMC_FPQ_TS(pipe)		_MMIO_PIPE((pipe), _PIPEDMC_FPQ_TS_A, _PIPEDMC_FPQ_TS_B)

#define _PIPEDMC_SCANLINE_RO_A		0x5f144
#define _PIPEDMC_SCANLINE_RO_B		0x5f544
#define PIPEDMC_SCANLINE_RO(pipe)	_MMIO_PIPE((pipe), _PIPEDMC_SCANLINE_RO_A, _PIPEDMC_SCANLINE_RO_B)

#define _PIPEDMC_FPQ_CTL1_A		0x5f160
#define _PIPEDMC_FPQ_CTL1_B		0x5f560
#define PIPEDMC_FPQ_CTL1(pipe)		_MMIO_PIPE((pipe), _PIPEDMC_FPQ_CTL1_A, _PIPEDMC_FPQ_CTL1_B)
#define   PIPEDMC_SW_DMC_WAKE		REG_BIT(0)

#define _PIPEDMC_FPQ_CTL2_A		0x5f164
#define _PIPEDMC_FPQ_CTL2_B		0x5f564
#define PIPEDMC_FPQ_CTL2(pipe)		_MMIO_PIPE((pipe), _PIPEDMC_FPQ_CTL2_A, _PIPEDMC_FPQ_CTL2_B)
#define   PIPEDMC_DMC_INT_AT_DELAYED_VBLANK	REG_BIT(1)
#define   PIPEDMC_W1_DMC_WAKE			REG_BIT(0)

#define _PIPEDMC_INTERRUPT_A		0x5f190 /* lnl+ */
#define _PIPEDMC_INTERRUPT_B		0x5f590 /* lnl+ */
#define PIPEDMC_INTERRUPT(pipe)		_MMIO_PIPE((pipe), _PIPEDMC_INTERRUPT_A, _PIPEDMC_INTERRUPT_B)
#define _PIPEDMC_INTERRUPT_MASK_A	0x5f194 /* lnl+ */
#define _PIPEDMC_INTERRUPT_MASK_B	0x5f594 /* lnl+ */
#define PIPEDMC_INTERRUPT_MASK(pipe)	_MMIO_PIPE((pipe), _PIPEDMC_INTERRUPT_MASK_A, _PIPEDMC_INTERRUPT_MASK_B)
#define   PIPEDMC_FLIPQ_PROG_DONE	REG_BIT(3)
#define   PIPEDMC_ERROR			REG_BIT(2)
#define   PIPEDMC_GTT_FAULT		REG_BIT(1)
#define   PIPEDMC_ATS_FAULT		REG_BIT(0)

#define PIPEDMC_BLOCK_PKGC_SW_A	0x5f1d0
#define PIPEDMC_BLOCK_PKGC_SW_B	0x5F5d0
#define PIPEDMC_BLOCK_PKGC_SW(pipe)				_MMIO_PIPE(pipe, \
									   PIPEDMC_BLOCK_PKGC_SW_A, \
									   PIPEDMC_BLOCK_PKGC_SW_B)
#define PIPEDMC_BLOCK_PKGC_SW_BLOCK_PKGC_ALWAYS			BIT(31)
#define PIPEDMC_BLOCK_PKGC_SW_BLOCK_PKGC_UNTIL_NEXT_FRAMESTART	BIT(15)

#define _ADLP_PIPEDMC_REG_MMIO_BASE_A	0x5f000
#define _TGL_PIPEDMC_REG_MMIO_BASE_A	0x92000

#define __PIPEDMC_REG_MMIO_BASE(i915, dmc_id) \
	((DISPLAY_VER(i915) >= 13 ? _ADLP_PIPEDMC_REG_MMIO_BASE_A : \
				    _TGL_PIPEDMC_REG_MMIO_BASE_A) + \
	 0x400 * ((dmc_id) - 1))

#define __DMC_REG_MMIO_BASE		0x8f000

#define _DMC_REG_MMIO_BASE(i915, dmc_id) \
	((dmc_id) == DMC_FW_MAIN ? __DMC_REG_MMIO_BASE : \
				   __PIPEDMC_REG_MMIO_BASE(i915, dmc_id))

#define _DMC_REG(i915, dmc_id, reg) \
	((reg) - __DMC_REG_MMIO_BASE + _DMC_REG_MMIO_BASE(i915, dmc_id))

#define DMC_EVENT_HANDLER_COUNT_GEN12	8

#define _DMC_EVT_HTP_0			0x8f004

#define DMC_EVT_HTP(i915, dmc_id, handler) \
	_MMIO(_DMC_REG(i915, dmc_id, _DMC_EVT_HTP_0) + 4 * (handler))

#define _DMC_EVT_CTL_0			0x8f034

#define DMC_EVT_CTL(i915, dmc_id, handler) \
	_MMIO(_DMC_REG(i915, dmc_id, _DMC_EVT_CTL_0) + 4 * (handler))

#define DMC_EVT_CTL_ENABLE		REG_BIT(31)
#define DMC_EVT_CTL_RECURRING		REG_BIT(30)
#define DMC_EVT_CTL_TYPE_MASK		REG_GENMASK(17, 16)
#define DMC_EVT_CTL_TYPE_LEVEL_0	0
#define DMC_EVT_CTL_TYPE_LEVEL_1	1
#define DMC_EVT_CTL_TYPE_EDGE_1_0	2
#define DMC_EVT_CTL_TYPE_EDGE_0_1	3
#define DMC_EVT_CTL_EVENT_ID_MASK	REG_GENMASK(15, 8)

#define DMC_HTP_ADDR_SKL	0x00500034
#define DMC_SSP_BASE		_MMIO(0x8F074)
#define DMC_HTP_SKL		_MMIO(0x8F004)
#define DMC_LAST_WRITE		_MMIO(0x8F034)
#define DMC_LAST_WRITE_VALUE	0xc003b400
#define DMC_MMIO_START_RANGE	0x80000
#define DMC_MMIO_END_RANGE     0x8FFFF
#define DMC_V1_MMIO_START_RANGE		0x80000
#define TGL_MAIN_MMIO_START		0x8F000
#define TGL_MAIN_MMIO_END		0x8FFFF
#define _TGL_PIPEA_MMIO_START		0x92000
#define _TGL_PIPEA_MMIO_END		0x93FFF
#define _TGL_PIPEB_MMIO_START		0x96000
#define _TGL_PIPEB_MMIO_END		0x97FFF
#define ADLP_PIPE_MMIO_START		0x5F000
#define ADLP_PIPE_MMIO_END		0x5FFFF

#define TGL_PIPE_MMIO_START(dmc_id)	_PICK_EVEN(((dmc_id) - 1), _TGL_PIPEA_MMIO_START,\
					      _TGL_PIPEB_MMIO_START)

#define TGL_PIPE_MMIO_END(dmc_id)	_PICK_EVEN(((dmc_id) - 1), _TGL_PIPEA_MMIO_END,\
					      _TGL_PIPEB_MMIO_END)

#define SKL_DMC_DC3_DC5_COUNT	_MMIO(0x80030)
#define SKL_DMC_DC5_DC6_COUNT	_MMIO(0x8002C)
#define BXT_DMC_DC3_DC5_COUNT	_MMIO(0x80038)
#define TGL_DMC_DEBUG_DC5_COUNT	_MMIO(0x101084)
#define TGL_DMC_DEBUG_DC6_COUNT	_MMIO(0x101088)
#define DG1_DMC_DEBUG_DC5_COUNT	_MMIO(0x134154)

#define TGL_DMC_DEBUG3		_MMIO(0x101090)
#define DG1_DMC_DEBUG3		_MMIO(0x13415c)

#define DMC_WAKELOCK_CFG	_MMIO(0x8F1B0)
#define  DMC_WAKELOCK_CFG_ENABLE REG_BIT(31)
#define DMC_WAKELOCK1_CTL	_MMIO(0x8F140)
#define  DMC_WAKELOCK_CTL_REQ	 REG_BIT(31)
#define  DMC_WAKELOCK_CTL_ACK	 REG_BIT(15)

#define DMC_FQ_W2_PTS_CFG_SEL	_MMIO(0x8f240)
#define   PIPE_D_DMC_W2_PTS_CONFIG_SELECT_MASK	REG_GENMASK(26, 24)
#define   PIPE_D_DMC_W2_PTS_CONFIG_SELECT(pipe)	REG_FIELD_PREP(PIPE_D_DMC_W2_PTS_CONFIG_SELECT_MASK, (pipe))
#define   PIPE_C_DMC_W2_PTS_CONFIG_SELECT_MASK	REG_GENMASK(18, 16)
#define   PIPE_C_DMC_W2_PTS_CONFIG_SELECT(pipe)	REG_FIELD_PREP(PIPE_C_DMC_W2_PTS_CONFIG_SELECT_MASK, (pipe))
#define   PIPE_B_DMC_W2_PTS_CONFIG_SELECT_MASK	REG_GENMASK(10, 8)
#define   PIPE_B_DMC_W2_PTS_CONFIG_SELECT(pipe)	REG_FIELD_PREP(PIPE_B_DMC_W2_PTS_CONFIG_SELECT_MASK, (pipe))
#define   PIPE_A_DMC_W2_PTS_CONFIG_SELECT_MASK	REG_GENMASK(2, 0)
#define   PIPE_A_DMC_W2_PTS_CONFIG_SELECT(pipe)	REG_FIELD_PREP(PIPE_A_DMC_W2_PTS_CONFIG_SELECT_MASK, (pipe))

/* plane/general flip queue entries */
#define PIPEDMC_FQ_RAM(start_mmioaddr, i)	_MMIO((start_mmioaddr) + (i) * 4)
/* LNL */
/* DW0 pts */
/* DW1 head */
/* DW2 size/etc. */
#define LNL_FQ_INTERRUPT	REG_BIT(31)
#define LNL_FQ_DSB_ID_MASK	REG_GENMASK(30, 29)
#define LNL_FQ_DSB_ID(dsb_id)	REG_FIELD_PREP(LNL_FQ_DSB_ID_MASK, (dsb_id))
#define LNL_FQ_EXECUTED		REG_BIT(28)
#define LNL_FQ_DSB_SIZE_MASK	REG_GENMASK(15, 0)
#define LNL_FQ_DSB_SIZE(size)	REG_FIELD_PREP(LNL_FQ_DSB_SIZE_MASK, (size))
/* DW3 reserved (plane queues) */
/* DW3 second DSB head (general queue) */
/* DW4 second DSB size/etc. (general queue) */
/* DW5 reserved (general queue) */

/* PTL+ */
/* DW0 pts */
/* DW1 reserved */
/* DW2 size/etc. */
#define PTL_FQ_INTERRUPT	REG_BIT(31)
#define PTL_FQ_NEED_PUSH	REG_BIT(30)
#define PTL_FQ_BLOCK_PUSH	REG_BIT(29)
#define PTL_FQ_EXECUTED		REG_BIT(28)
#define PTL_FQ_DSB_ID_MASK	REG_GENMASK(25, 24)
#define PTL_FQ_DSB_ID(dsb_id)	REG_FIELD_PREP(PTL_FQ_DSB_ID_MASK, (dsb_id))
#define PTL_FQ_DSB_SIZE_MASK	REG_GENMASK(15, 0)
#define PTL_FQ_DSB_SIZE(size)	REG_FIELD_PREP(PTL_FQ_DSB_SIZE_MASK, (size))
/* DW3 head */
/* DW4 second DSB size/etc. (general queue) */
/* DW5 second DSB head (general queue) */

/* undocumented magic DMC variables */
#define PTL_PIPEDMC_EXEC_TIME_LINES(start_mmioaddr) _MMIO((start_mmioaddr) + 0x6b8)
#define PTL_PIPEDMC_END_OF_EXEC_GB(start_mmioaddr) _MMIO((start_mmioaddr) + 0x6c0)

#endif /* __INTEL_DMC_REGS_H__ */
