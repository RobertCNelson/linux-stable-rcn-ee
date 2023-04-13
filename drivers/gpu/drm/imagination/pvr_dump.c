// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#include <generated/utsrelease.h>
#include <linux/devcoredump.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/utsname.h>
#include <uapi/drm/pvr_drm.h>

#include "pvr_context.h"
#include "pvr_device.h"
#include "pvr_dump.h"
#include "pvr_rogue_fwif.h"

static const struct {
	u32 offset;
	u32 flags;
} pvr_dump_registers[] = {
	{ROGUE_CR_EVENT_STATUS, PVR_COREDUMP_REGISTER_FLAG_SIZE_32BIT},
	{ROGUE_CR_BIF_FAULT_BANK0_MMU_STATUS, PVR_COREDUMP_REGISTER_FLAG_SIZE_32BIT},
	{ROGUE_CR_BIF_FAULT_BANK0_REQ_STATUS, PVR_COREDUMP_REGISTER_FLAG_SIZE_64BIT},
	{ROGUE_CR_BIF_FAULT_BANK1_MMU_STATUS, PVR_COREDUMP_REGISTER_FLAG_SIZE_32BIT},
	{ROGUE_CR_BIF_FAULT_BANK1_REQ_STATUS, PVR_COREDUMP_REGISTER_FLAG_SIZE_64BIT},
	{ROGUE_CR_BIF_MMU_STATUS, PVR_COREDUMP_REGISTER_FLAG_SIZE_32BIT},
	{ROGUE_CR_BIF_MMU_ENTRY, PVR_COREDUMP_REGISTER_FLAG_SIZE_32BIT},
	{ROGUE_CR_BIF_MMU_ENTRY_STATUS, PVR_COREDUMP_REGISTER_FLAG_SIZE_64BIT},
	{ROGUE_CR_BIF_STATUS_MMU, PVR_COREDUMP_REGISTER_FLAG_SIZE_32BIT},
};

static void
pvr_coredump_write_header(u32 size, u8 **p)
{
	struct pvr_coredump_header *header = (struct pvr_coredump_header *)*p;

	header->magic = cpu_to_le32(PVR_COREDUMP_HEADER_MAGIC);
	header->major_version = cpu_to_le32(PVR_COREDUMP_HEADER_VERSION_MAJ);
	header->minor_version = cpu_to_le32(PVR_COREDUMP_HEADER_VERSION_MIN);
	header->size = cpu_to_le32(size);

	*p = (u8 *)(header + 1);
}

static void
pvr_coredump_write_block_header(u32 block_type, u32 size, u8 **p)
{
	struct pvr_coredump_block_header *block_header;

	block_header = (struct pvr_coredump_block_header *)*p;
	block_header->type = cpu_to_le32(block_type);
	block_header->size = cpu_to_le32(size);
	*p = (u8 *)(block_header + 1);
}

static void
pvr_coredump_write_devinfo(struct pvr_device *pvr_dev, u32 context_id, u8 **p)
{
	struct pvr_coredump_block_devinfo *devinfo;

	pvr_coredump_write_block_header(PVR_COREDUMP_BLOCK_TYPE_DEVINFO, sizeof(*devinfo), p);

	devinfo = (struct pvr_coredump_block_devinfo *)*p;
	devinfo->gpu_id = cpu_to_le64(pvr_gpu_id_to_packed_bvnc(&pvr_dev->gpu_id));
	devinfo->fw_version.major = cpu_to_le32(pvr_dev->fw_version.major);
	devinfo->fw_version.minor = cpu_to_le32(pvr_dev->fw_version.minor);
	if (context_id) {
		struct pvr_context *ctx = pvr_context_lookup_id(pvr_dev, context_id);

		if (ctx) {
			strscpy(devinfo->process_name, ctx->process_name,
				sizeof(devinfo->process_name));

			pvr_context_put(ctx);
		}
	}
	strscpy(devinfo->kernel_version, UTS_RELEASE, sizeof(devinfo->kernel_version));
	*p = (u8 *)(devinfo + 1);
}

static void
pvr_coredump_write_registers(struct pvr_device *pvr_dev, u8 **p)
{
	struct pvr_coredump_block_register *reg;
	u32 i;

	pvr_coredump_write_block_header(PVR_COREDUMP_BLOCK_TYPE_REGISTERS,
					sizeof(*reg) * ARRAY_SIZE(pvr_dump_registers), p);

	reg = (struct pvr_coredump_block_register *)*p;

	for (i = 0; i < ARRAY_SIZE(pvr_dump_registers); i++) {
		reg->offset = cpu_to_le32(pvr_dump_registers[i].offset);
		reg->flags = cpu_to_le32(pvr_dump_registers[i].flags);

		switch (reg->flags & PVR_COREDUMP_REGISTER_FLAG_SIZE_MASK) {
		case PVR_COREDUMP_REGISTER_FLAG_SIZE_32BIT:
			reg->value = cpu_to_le32(__pvr_cr_read32(pvr_dev, reg->offset));
			break;

		case PVR_COREDUMP_REGISTER_FLAG_SIZE_64BIT:
			reg->value = cpu_to_le64(__pvr_cr_read64(pvr_dev, reg->offset));
			break;

		default:
			WARN_ON(1);
			reg->value = 0;
			break;
		}

		reg++;
	}

	*p = (u8 *)reg;
}

static void
pvr_coredump_write_reset_data(struct rogue_fwif_fwccb_cmd_context_reset_data *fw_reset_data,
			      u8 **p)
{
	struct pvr_coredump_block_reset_data *reset_data;

	pvr_coredump_write_block_header(PVR_COREDUMP_BLOCK_TYPE_CONTEXT_RESET_DATA,
					sizeof(*reset_data), p);

	reset_data = (struct pvr_coredump_block_reset_data *)*p;
	reset_data->context_id = cpu_to_le32(fw_reset_data->server_common_context_id);
	reset_data->reset_reason = cpu_to_le32(fw_reset_data->reset_reason);
	reset_data->dm = cpu_to_le32(fw_reset_data->dm);
	reset_data->reset_job_ref = cpu_to_le32(fw_reset_data->reset_job_ref);
	reset_data->flags = cpu_to_le32(fw_reset_data->flags);
	reset_data->fault_address = cpu_to_le64(fw_reset_data->fault_address);
	*p = (u8 *)(reset_data + 1);
}

static void
pvr_coredump_write_hwrinfo(struct pvr_device *pvr_dev,
			   u8 **p)
{
	struct rogue_fwif_hwrinfobuf *hwrinfobuf = pvr_dev->fw_dev.hwrinfobuf;
	struct pvr_coredump_block_hwrinfo *hwrinfo;
	struct rogue_hwrinfo *fw_hwrinfo;

	pvr_coredump_write_block_header(PVR_COREDUMP_BLOCK_TYPE_HWRINFO, sizeof(*hwrinfo), p);

	/* Record the most recent HWR. */
	fw_hwrinfo = &hwrinfobuf->hwr_info[(hwrinfobuf->write_index - 1) &
					   ROGUE_FWIF_HWINFO_LAST_INDEX];

	hwrinfo = (struct pvr_coredump_block_hwrinfo *)*p;

	hwrinfo->hwr_type = cpu_to_le32(fw_hwrinfo->hwr_type);
	hwrinfo->dm = cpu_to_le32(fw_hwrinfo->dm);
	hwrinfo->core_id = cpu_to_le32(fw_hwrinfo->core_id);
	hwrinfo->event_status = cpu_to_le32(fw_hwrinfo->event_status);
	hwrinfo->dm_state = cpu_to_le32(fw_hwrinfo->hwr_recovery_flags);
	hwrinfo->active_hwrt_data = cpu_to_le32(fw_hwrinfo->active_hwrt_data);

	switch (hwrinfo->hwr_type) {
	case ROGUE_HWRTYPE_BIF0FAULT:
	case ROGUE_HWRTYPE_BIF1FAULT:
	case ROGUE_HWRTYPE_TEXASBIF0FAULT:
	case ROGUE_HWRTYPE_MMURISCVFAULT:
		hwrinfo->hwr_data.bif_info.bif_req_status =
			cpu_to_le64(fw_hwrinfo->hwr_data.bif_info.bif_req_status);
		hwrinfo->hwr_data.bif_info.bif_mmu_status =
			cpu_to_le64(fw_hwrinfo->hwr_data.bif_info.bif_mmu_status);
		break;

	case ROGUE_HWRTYPE_ECCFAULT:
		hwrinfo->hwr_data.ecc_info.fault_gpu =
			cpu_to_le32(fw_hwrinfo->hwr_data.ecc_info.fault_gpu);
		break;

	case ROGUE_HWRTYPE_MMUFAULT:
	case ROGUE_HWRTYPE_MMUMETAFAULT:
		hwrinfo->hwr_data.mmu_info.mmu_status[0] =
			cpu_to_le64(fw_hwrinfo->hwr_data.mmu_info.mmu_status[0]);
		hwrinfo->hwr_data.mmu_info.mmu_status[1] =
			cpu_to_le64(fw_hwrinfo->hwr_data.mmu_info.mmu_status[1]);
		break;

	case ROGUE_HWRTYPE_POLLFAILURE:
		hwrinfo->hwr_data.poll_info.thread_num =
			cpu_to_le32(fw_hwrinfo->hwr_data.poll_info.thread_num);
		hwrinfo->hwr_data.poll_info.cr_poll_addr =
			cpu_to_le32(fw_hwrinfo->hwr_data.poll_info.cr_poll_addr);
		hwrinfo->hwr_data.poll_info.cr_poll_mask =
			cpu_to_le32(fw_hwrinfo->hwr_data.poll_info.cr_poll_mask);
		hwrinfo->hwr_data.poll_info.cr_poll_last_value =
			cpu_to_le32(fw_hwrinfo->hwr_data.poll_info.cr_poll_last_value);
		break;

	case ROGUE_HWRTYPE_MIPSTLBFAULT:
		hwrinfo->hwr_data.tlb_info.bad_addr =
			cpu_to_le32(fw_hwrinfo->hwr_data.tlb_info.bad_addr);
		hwrinfo->hwr_data.tlb_info.entry_lo =
			cpu_to_le32(fw_hwrinfo->hwr_data.tlb_info.entry_lo);
		break;

	case ROGUE_HWRTYPE_OVERRUN:
	case ROGUE_HWRTYPE_UNKNOWNFAILURE:
		/* Nothing to save. */
	default:
		break;
	}

	*p = (u8 *)(hwrinfo + 1);
}

static void
pvr_coredump(struct pvr_device *pvr_dev,
	     struct rogue_fwif_fwccb_cmd_context_reset_data *fw_reset_data,
	     u32 context_id)
{
	u8 *data;
	u32 size;
	u8 *p;

	size = sizeof(struct pvr_coredump_header);

	size += sizeof(struct pvr_coredump_block_header) +
		sizeof(struct pvr_coredump_block_devinfo);

	size += sizeof(struct pvr_coredump_block_header) +
		sizeof(struct pvr_coredump_block_register) * ARRAY_SIZE(pvr_dump_registers);

	size += sizeof(struct pvr_coredump_block_header) +
		sizeof(struct pvr_coredump_block_hwrinfo);

	if (fw_reset_data) {
		size += sizeof(struct pvr_coredump_block_header) +
			sizeof(struct pvr_coredump_block_reset_data);
	}

	/*
	 * Add flags to avoid triggering the OOM killer or error reporting, we should just warn if
	 * we can't allocate the buffer.
	 */
	data = __vmalloc(size, GFP_KERNEL | __GFP_ZERO | __GFP_NOWARN | __GFP_NORETRY);
	if (!data) {
		drm_warn(from_pvr_device(pvr_dev), "Failed to allocate devcoredump buffer\n");
		return;
	}

	p = data;

	pvr_coredump_write_header(size, &p);
	pvr_coredump_write_devinfo(pvr_dev, context_id, &p);
	pvr_coredump_write_registers(pvr_dev, &p);
	pvr_coredump_write_hwrinfo(pvr_dev, &p);

	if (fw_reset_data)
		pvr_coredump_write_reset_data(fw_reset_data, &p);

	dev_coredumpv(from_pvr_device(pvr_dev)->dev, data, size, GFP_KERNEL);
}

static const char *
get_reset_reason_desc(enum rogue_context_reset_reason reason)
{
	switch (reason) {
	case ROGUE_CONTEXT_RESET_REASON_NONE:
		return "None";
	case ROGUE_CONTEXT_RESET_REASON_GUILTY_LOCKUP:
		return "Guilty lockup";
	case ROGUE_CONTEXT_RESET_REASON_INNOCENT_LOCKUP:
		return "Innocent lockup";
	case ROGUE_CONTEXT_RESET_REASON_GUILTY_OVERRUNING:
		return "Guilty overrunning";
	case ROGUE_CONTEXT_RESET_REASON_INNOCENT_OVERRUNING:
		return "Innocent overrunning";
	case ROGUE_CONTEXT_RESET_REASON_HARD_CONTEXT_SWITCH:
		return "Hard context switch";
	case ROGUE_CONTEXT_RESET_REASON_FW_WATCHDOG:
		return "Firmware watchdog";
	case ROGUE_CONTEXT_RESET_REASON_FW_PAGEFAULT:
		return "Firmware pagefault";
	case ROGUE_CONTEXT_RESET_REASON_FW_EXEC_ERR:
		return "Firmware execution error";
	case ROGUE_CONTEXT_RESET_REASON_HOST_WDG_FW_ERR:
		return "Host watchdog";
	case ROGUE_CONTEXT_GEOM_OOM_DISABLED:
		return "Geometry OOM disabled";

	default:
		return "Unknown";
	}
}

static const char *
get_dm_name(u32 dm)
{
	switch (dm) {
	case PVR_FWIF_DM_GP:
		return "General purpose";
	case PVR_FWIF_DM_2D:
		return "2D";
	case PVR_FWIF_DM_GEOM:
		return "Geometry";
	case PVR_FWIF_DM_FRAG:
		return "Fragment";
	case PVR_FWIF_DM_CDM:
		return "Compute";
	case PVR_FWIF_DM_RAY:
		return "Raytracing";
	case PVR_FWIF_DM_GEOM2:
		return "Geometry 2";
	case PVR_FWIF_DM_GEOM3:
		return "Geometry 3";
	case PVR_FWIF_DM_GEOM4:
		return "Geometry 4";

	default:
		return "Unknown";
	}
}

/**
 * pvr_context_reset_notification() - Handle context reset notification from FW
 * @pvr_dev: Device pointer.
 * @data: Data provided by FW.
 *
 * This will decode the data structure provided by FW and print the results via drm_info().
 */
void
pvr_context_reset_notification(struct pvr_device *pvr_dev,
			       struct rogue_fwif_fwccb_cmd_context_reset_data *data)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);

	if (data->flags & ROGUE_FWIF_FWCCB_CMD_CONTEXT_RESET_FLAG_ALL_CTXS) {
		drm_info(drm_dev, "Received context reset notification for all contexts\n");
	} else {
		drm_info(drm_dev, "Received context reset notification on context %u\n",
			 data->server_common_context_id);
	}

	drm_info(drm_dev, "  Reset reason=%u (%s)\n", data->reset_reason,
		 get_reset_reason_desc(data->reset_reason));
	drm_info(drm_dev, "  Data Master=%u (%s)\n", data->dm, get_dm_name(data->dm));
	drm_info(drm_dev, "  Job ref=%u\n", data->reset_job_ref);

	if (data->flags & ROGUE_FWIF_FWCCB_CMD_CONTEXT_RESET_FLAG_PF) {
		drm_info(drm_dev, "  Page fault occurred, fault address=%llx\n",
			 (unsigned long long)data->fault_address);
	}

	pvr_coredump(pvr_dev, data,
		     (data->flags & ROGUE_FWIF_FWCCB_CMD_CONTEXT_RESET_FLAG_ALL_CTXS) ? 0 :
		      data->server_common_context_id);
}
