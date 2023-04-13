/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#ifndef __PVR_DEVICE_INFO_H__
#define __PVR_DEVICE_INFO_H__

#include <linux/types.h>

struct pvr_device;

/**
 * struct pvr_device_features - Hardware feature information
 */
struct pvr_device_features {
	bool has_cdm_control_stream_format : 1;
	bool has_cluster_grouping : 1;
	bool has_common_store_size_in_dwords : 1;
	bool has_compute : 1;
	bool has_compute_morton_capable : 1;
	bool has_compute_overlap : 1;
	bool has_fb_cdc_v4 : 1;
	bool has_fbcdc_algorithm : 1;
	bool has_gpu_multicore_support : 1;
	bool has_isp_max_tiles_in_flight : 1;
	bool has_isp_samples_per_pixel : 1;
	bool has_isp_zls_d24_s8_packing_ogl_mode : 1;
	bool has_max_partitions : 1;
	bool has_meta : 1;
	bool has_meta_coremem_size : 1;
	bool has_mips : 1;
	bool has_num_clusters : 1;
	bool has_num_isp_ipp_pipes : 1;
	bool has_num_raster_pipes : 1;
	bool has_phys_bus_width : 1;
	bool has_riscv_fw_processor : 1;
	bool has_roguexe : 1;
	bool has_s7_top_infrastructure : 1;
	bool has_simple_internal_parameter_format : 1;
	bool has_slc_cache_line_size_in_bits : 1;
	bool has_sys_bus_secure_reset : 1;
	bool has_tessellation : 1;
	bool has_tile_size_x : 1;
	bool has_tile_size_y : 1;
	bool has_tpu_dm_global_registers : 1;
	bool has_usc_min_output_registers_per_pix : 1;
	bool has_vdm_drawindirect : 1;
	bool has_vdm_object_level_lls : 1;
	bool has_virtual_address_space_bits : 1;
	bool has_xe_memory_hierarchy : 1;
	bool has_xpu_max_slaves : 1;
	bool has_xt_top_infrastructure : 1;
	bool has_zls_subtile : 1;

	u8 cdm_control_stream_format;
	u32 common_store_size_in_dwords;
	u8 fbcdc_algorithm;
	u16 isp_max_tiles_in_flight;
	bool isp_samples_per_pixel;
	u16 max_partitions;
	bool meta;
	u32 meta_coremem_size;
	bool mips;
	u16 num_clusters;
	u8 num_isp_ipp_pipes;
	u8 num_raster_pipes;
	u16 phys_bus_width;
	bool riscv_fw_processor;
	u32 simple_internal_parameter_format;
	u16 slc_cache_line_size_in_bits;
	u16 tile_size_x;
	u16 tile_size_y;
	u16 usc_min_output_registers_per_pix;
	u16 virtual_address_space_bits;
	u8 xpu_max_slaves;
};

/**
 * struct pvr_device_quirks - Hardware quirk information
 */
struct pvr_device_quirks {
	bool has_brn44079 : 1;
	bool has_brn47217 : 1;
	bool has_brn48492 : 1;
	bool has_brn48545 : 1;
	bool has_brn49927 : 1;
	bool has_brn51764 : 1;
	bool has_brn52354 : 1;
	bool has_brn62269 : 1;
	bool has_brn63142 : 1;
	bool has_brn63553 : 1;
	bool has_brn66011 : 1;
};

/**
 * struct pvr_device_enhancements - Hardware enhancement information
 */
struct pvr_device_enhancements {
	bool has_ern35421 : 1;
	bool has_ern38020 : 1;
	bool has_ern38748 : 1;
	bool has_ern42064 : 1;
};

int pvr_device_info_init(struct pvr_device *pvr_dev);

/*
 * Meta cores
 *
 * These are the values for the 'meta' feature when the feature is present
 * (as per @pvr_device_features)/
 */
#define PVR_META_MTP218 (1)
#define PVR_META_MTP219 (2)
#define PVR_META_LTP218 (3)
#define PVR_META_LTP217 (4)

enum {
	PVR_FEATURE_CDM_USER_MODE_QUEUE,
	PVR_FEATURE_CLUSTER_GROUPING,
	PVR_FEATURE_COMPUTE_MORTON_CAPABLE,
	PVR_FEATURE_FB_CDC_V4,
	PVR_FEATURE_GPU_MULTICORE_SUPPORT,
	PVR_FEATURE_ISP_ZLS_D24_S8_PACKING_OGL_MODE,
	PVR_FEATURE_REQUIRES_FB_CDC_ZLS_SETUP,
	PVR_FEATURE_S7_TOP_INFRASTRUCTURE,
	PVR_FEATURE_TESSELLATION,
	PVR_FEATURE_TPU_DM_GLOBAL_REGISTERS,
	PVR_FEATURE_VDM_DRAWINDIRECT,
	PVR_FEATURE_VDM_OBJECT_LEVEL_LLS,
	PVR_FEATURE_ZLS_SUBTILE,
};

#endif /* __PVR_DEVICE_INFO_H__ */
