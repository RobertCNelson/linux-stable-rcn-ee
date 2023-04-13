/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#ifndef __PVR_FW_H__
#define __PVR_FW_H__

#include "pvr_fw_info.h"
#include "pvr_fw_trace.h"

#include <drm/drm_mm.h>

#include <linux/types.h>

/* Forward declarations from "pvr_device.h". */
struct pvr_device;
struct pvr_file;

/* Forward declaration from "pvr_gem.h". */
struct pvr_fw_object;

/* Forward declaration from "pvr_vm.h". */
struct pvr_vm_context;

#define ROGUE_FWIF_FWCCB_NUMCMDS_LOG2 5

#define ROGUE_FWIF_KCCB_NUMCMDS_LOG2_DEFAULT 7

/**
 * struct pvr_fw_funcs - FW processor function table
 */
struct pvr_fw_funcs {
	/**
	 * @init:
	 *
	 * FW processor specific initialisation.
	 * @pvr_dev: Target PowerVR device.
	 *
	 * This function must call pvr_fw_heap_calculate() to initialise the firmware heap for this
	 * FW processor.
	 *
	 * This function is mandatory.
	 *
	 * Returns:
	 *  * 0 on success, or
	 *  * Any appropriate error on failure.
	 */
	int (*init)(struct pvr_device *pvr_dev);

	/**
	 * @fini:
	 *
	 * FW processor specific finalisation.
	 * @pvr_dev: Target PowerVR device.
	 *
	 * This function is optional.
	 */
	void (*fini)(struct pvr_device *pvr_dev);

	/**
	 * @fw_process:
	 *
	 * Load and process firmware image.
	 * @pvr_dev: Target PowerVR device.
	 * @fw: Pointer to firmware image.
	 * @layout_entries: Layout of firmware memory.
	 * @num_layout_entries: Number of entries in @layout_entries.
	 * @fw_code_ptr: Pointer to firmware code section.
	 * @fw_data_ptr: Pointer to firmware data section.
	 * @fw_core_code_ptr: Pointer to firmware core code section. May be %NULL.
	 * @fw_core_data_ptr: Pointer to firmware core data section. May be %NULL.
	 * @core_code_alloc_size: Total allocation size of core code section.
	 *
	 * This function is mandatory.
	 *
	 * Returns:
	 *  * 0 on success, or
	 *  * Any appropriate error on failure.
	 */
	int (*fw_process)(struct pvr_device *pvr_dev, const u8 *fw,
			  const struct pvr_fw_layout_entry *layout_entries, u32 num_layout_entries,
			  u8 *fw_code_ptr, u8 *fw_data_ptr, u8 *fw_core_code_ptr,
			  u8 *fw_core_data_ptr, u32 core_code_alloc_size);

	/**
	 * @vm_map:
	 *
	 * Map FW object into FW processor address space.
	 * @pvr_dev: Target PowerVR device.
	 * @fw_obj: FW object to map.
	 *
	 * This function is mandatory.
	 *
	 * Returns:
	 *  * 0 on success, or
	 *  * Any appropriate error on failure.
	 */
	int (*vm_map)(struct pvr_device *pvr_dev, struct pvr_fw_object *fw_obj);

	/**
	 * @vm_unmap:
	 *
	 * Unmap FW object from FW processor address space.
	 * @pvr_dev: Target PowerVR device.
	 * @fw_obj: FW object to map.
	 *
	 * This function is mandatory.
	 */
	void (*vm_unmap)(struct pvr_device *pvr_dev, struct pvr_fw_object *fw_obj);

	/**
	 * @get_fw_addr_with_offset:
	 *
	 * Called to get address of object in firmware address space, with offset.
	 * @fw_obj: Pointer to object.
	 * @offset: Desired offset from start of object.
	 *
	 * This function is mandatory.
	 *
	 * Returns:
	 *  * Address in firmware address space.
	 */
	u32 (*get_fw_addr_with_offset)(struct pvr_fw_object *fw_obj, u32 offset);

	/**
	 * @wrapper_init:
	 *
	 * Called to initialise FW wrapper.
	 * @pvr_dev: Target PowerVR device.
	 *
	 * This function is mandatory.
	 *
	 * Returns:
	 *  * 0 on success.
	 *  * Any appropriate error on failure.
	 */
	int (*wrapper_init)(struct pvr_device *pvr_dev);

	/**
	 * @check_and_ack_irq:
	 *
	 * Called to check if a GPU interrupt has occurred, and to acknowledge if it has.
	 * @pvr_dev: Target PowerVR device.
	 *
	 * This function is mandatory.
	 *
	 * Returns:
	 *  * %true if an interrupt has occurred, or
	 *  * %false if no interrupt has occurred.
	 */
	bool (*check_and_ack_irq)(struct pvr_device *pvr_dev);

	/**
	 * @has_fixed_data_addr:
	 *
	 * Called to check if firmware fixed data must be loaded at the address given by the
	 * firmware layout table.
	 *
	 * This function is mandatory.
	 *
	 * Returns:
	 *  * %true if firmware fixed data must be loaded at the address given by the firmware
	 *    layout table.
	 *  * %false otherwise.
	 */
	bool (*has_fixed_data_addr)(void);
};

/**
 * struct pvr_fw_mem - FW memory allocations
 */
struct pvr_fw_mem {
	/** @code_obj: Object representing firmware code. */
	struct pvr_fw_object *code_obj;

	/** @data_obj: Object representing firmware data. */
	struct pvr_fw_object *data_obj;

	/**
	 * @core_code_obj: Object representing firmware core code. May be
	 *                 %NULL if firmware does not contain this section.
	 */
	struct pvr_fw_object *core_code_obj;

	/**
	 * @core_data_obj: Object representing firmware core data. May be
	 *                 %NULL if firmware does not contain this section.
	 */
	struct pvr_fw_object *core_data_obj;

	/**
	 * @fwif_connection_ctl_obj: Object representing FWIF connection control
	 *                           structure.
	 */
	struct pvr_fw_object *fwif_connection_ctl_obj;

	/** @osinit_obj: Object representing FW OSINIT structure. */
	struct pvr_fw_object *osinit_obj;

	/** @sysinit_obj: Object representing FW SYSINIT structure. */
	struct pvr_fw_object *sysinit_obj;

	/** @osdata_obj: Object representing FW OSDATA structure. */
	struct pvr_fw_object *osdata_obj;

	/** @hwrinfobuf_obj: Object representing FW hwrinfobuf structure. */
	struct pvr_fw_object *hwrinfobuf_obj;

	/** @sysdata_obj: Object representing FW SYSDATA structure. */
	struct pvr_fw_object *sysdata_obj;

	/** @power_sync_obj: Object representing power sync state. */
	struct pvr_fw_object *power_sync_obj;

	/** @fault_page_obj: Object representing FW fault page. */
	struct pvr_fw_object *fault_page_obj;

	/** @gpu_util_fwcb_obj: Object representing FW GPU utilisation control structure. */
	struct pvr_fw_object *gpu_util_fwcb_obj;

	/** @runtime_cfg_obj: Object representing FW runtime config structure. */
	struct pvr_fw_object *runtime_cfg_obj;

	/** @mmucache_sync_obj: Object used as the sync parameter in an MMU cache operation. */
	struct pvr_fw_object *mmucache_sync_obj;
};

struct pvr_fw_device {
	/** @firmware: Handle to the firmware loaded into the device. */
	const struct firmware *firmware;

	/** @mem: Structure containing objects representing firmware memory allocations. */
	struct pvr_fw_mem mem;

	/** @booted: %true if the firmware has been booted, %false otherwise. */
	bool booted;

	/**
	 * @processor_type: FW processor type for this device. Must be one of
	 *                  %PVR_FW_PROCESSOR_TYPE_*.
	 */
	u16 processor_type;

	/** @funcs: Function table for the FW processor used by this device. */
	const struct pvr_fw_funcs *funcs;

	/** @processor_data: Pointer to data specific to FW processor. */
	union {
		/** @mips_data: Pointer to MIPS-specific data. */
		struct pvr_fw_mips_data *mips_data;
	} processor_data;

	/** @fw_heap_info: Firmware heap information. */
	struct {
		/** @gpu_addr: Base address of firmware heap in GPU address space. */
		u64 gpu_addr;

		/** @size: Size of main area of heap. */
		u32 size;

		/** @offset_mask: Mask for offsets within FW heap. */
		u32 offset_mask;

		/** @raw_size: Raw size of heap, including reserved areas. */
		u32 raw_size;

		/** @log2_size: Log2 of raw size of heap. */
		u32 log2_size;

		/** @config_offset: Offset of config area within heap. */
		u32 config_offset;

		/** @reserved_size: Size of reserved area in heap. */
		u32 reserved_size;
	} fw_heap_info;

	/** @fw_mm: Firmware address space allocator. */
	struct drm_mm fw_mm;

	/** @fw_mm_lock: Lock protecting access to &fw_mm. */
	spinlock_t fw_mm_lock;

	/** @fw_mm_base: Base address of address space managed by @fw_mm. */
	u64 fw_mm_base;

	/**
	 * @fwif_connection_ctl: Pointer to CPU mapping of FWIF connection
	 *                       control structure.
	 */
	struct rogue_fwif_connection_ctl *fwif_connection_ctl;

	/** @fwif_sysinit: Pointer to CPU mapping of FW SYSINIT structure. */
	struct rogue_fwif_sysinit *fwif_sysinit;

	/** @fwif_sysdata: Pointer to CPU mapping of FW SYSDATA structure. */
	struct rogue_fwif_sysdata *fwif_sysdata;

	/** @fwif_osinit: Pointer to CPU mapping of FW OSINIT structure. */
	struct rogue_fwif_osinit *fwif_osinit;

	/** @fwif_osdata: Pointer to CPU mapping of FW OSDATA structure. */
	struct rogue_fwif_osdata *fwif_osdata;

	/** @power_sync: Pointer to CPU mapping of power sync state. */
	u32 *power_sync;

	/** @hwrinfobuf: Pointer to CPU mapping of FW HWR info buffer. */
	struct rogue_fwif_hwrinfobuf *hwrinfobuf;

	/** @fw_trace: Device firmware trace buffer state. */
	struct pvr_fw_trace fw_trace;
};

extern const struct pvr_fw_funcs pvr_fw_funcs_meta;
extern const struct pvr_fw_funcs pvr_fw_funcs_mips;

int pvr_fw_init(struct pvr_device *pvr_dev);
void pvr_fw_fini(struct pvr_device *pvr_dev);

int pvr_wait_for_fw_boot(struct pvr_device *pvr_dev);

void pvr_fw_mts_schedule(struct pvr_device *pvr_dev, u32 val);

int
pvr_fw_mem_context_create(struct pvr_device *pvr_dev, struct pvr_vm_context *vm_ctx,
			  struct pvr_fw_object **fw_mem_ctx_obj_out);
void
pvr_fw_mem_context_destroy(struct pvr_fw_object *fw_mem_ctx_obj);

void
pvr_fw_heap_info_init(struct pvr_device *pvr_dev, u32 log2_size, u32 reserved_size);

const struct pvr_fw_layout_entry *
pvr_fw_find_layout_entry(const struct pvr_fw_layout_entry *layout_entries, u32 num_layout_entries,
			 enum pvr_fw_section_id id);
int
pvr_fw_find_mmu_segment(u32 addr, u32 size, const struct pvr_fw_layout_entry *layout_entries,
			u32 num_layout_entries, void *fw_code_ptr, void *fw_data_ptr,
			void *fw_core_code_ptr, void *fw_core_data_ptr,
			void **host_addr_out);

int
pvr_fw_structure_cleanup(struct pvr_device *pvr_dev, u32 type, struct pvr_fw_object *fw_obj,
			 u32 offset);

#endif /* __PVR_FW_H__ */
