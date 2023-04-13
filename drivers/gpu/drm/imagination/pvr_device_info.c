// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#include "pvr_device.h"
#include "pvr_device_info.h"

#include <drm/drm_print.h>

#include <linux/types.h>

const struct pvr_device_features pvr_device_4_V_2_51 = {
	.has_cdm_control_stream_format = true,
	.has_cluster_grouping = true,
	.has_common_store_size_in_dwords = true,
	.has_compute = true,
	.has_compute_morton_capable = true,
	.has_compute_overlap = true,
	.has_fbcdc_algorithm = true,
	.has_isp_max_tiles_in_flight = true,
	.has_isp_samples_per_pixel = true,
	.has_max_partitions = true,
	.has_meta = true,
	.has_meta_coremem_size = true,
	.has_num_clusters = true,
	.has_num_isp_ipp_pipes = true,
	.has_num_raster_pipes = true,
	.has_phys_bus_width = true,
	.has_slc_cache_line_size_in_bits = true,
	.has_tile_size_x = true,
	.has_tile_size_y = true,
	.has_usc_min_output_registers_per_pix = true,
	.has_virtual_address_space_bits = true,
	.has_xt_top_infrastructure = true,
	.has_zls_subtile = true,

	.cdm_control_stream_format = 1,
	.common_store_size_in_dwords = 1280U * 4U * 4U,
	.fbcdc_algorithm = 2,
	.isp_max_tiles_in_flight = 4,
	.isp_samples_per_pixel = 2,
	.max_partitions = 8,
	.meta = true,
	.meta_coremem_size = 32,
	.num_clusters = 2,
	.num_isp_ipp_pipes = 8,
	.num_raster_pipes = 1,
	.phys_bus_width = 40,
	.slc_cache_line_size_in_bits = 512,
	.tile_size_x = 32,
	.tile_size_y = 32,
	.usc_min_output_registers_per_pix = 2,
	.virtual_address_space_bits = 40,
};

const struct pvr_device_quirks pvr_device_quirks_4_40_2_51 = {
	.has_brn44079 = true,
	.has_brn48492 = true,
	.has_brn48545 = true,
	.has_brn49927 = true,
	.has_brn51764 = true,
	.has_brn52354 = true,
	.has_brn62269 = true,
	.has_brn63142 = true,
	.has_brn66011 = true,
};

struct pvr_device_enhancements pvr_device_enhancements_4_40_2_51 = {
	.has_ern35421 = true,
	.has_ern38020 = true,
	.has_ern38748 = true,
	.has_ern42064 = true,
};

const struct pvr_device_features pvr_device_33_V_11_3 = {
	.has_cdm_control_stream_format = true,
	.has_common_store_size_in_dwords = true,
	.has_compute = true,
	.has_isp_max_tiles_in_flight = true,
	.has_isp_samples_per_pixel = true,
	.has_max_partitions = true,
	.has_mips = true,
	.has_num_clusters = true,
	.has_num_isp_ipp_pipes = true,
	.has_num_raster_pipes = true,
	.has_phys_bus_width = true,
	.has_roguexe = true,
	.has_simple_internal_parameter_format = true,
	.has_slc_cache_line_size_in_bits = true,
	.has_sys_bus_secure_reset = true,
	.has_tile_size_x = true,
	.has_tile_size_y = true,
	.has_usc_min_output_registers_per_pix = true,
	.has_virtual_address_space_bits = true,
	.has_xe_memory_hierarchy = true,

	.cdm_control_stream_format = 1,
	.common_store_size_in_dwords = 512U * 4U * 4U,
	.isp_max_tiles_in_flight = 1,
	.isp_samples_per_pixel = 1,
	.max_partitions = 4,
	.mips = true,
	.num_clusters = 1,
	.num_isp_ipp_pipes = 1,
	.num_raster_pipes = 1,
	.phys_bus_width = 36,
	.simple_internal_parameter_format = 2,
	.slc_cache_line_size_in_bits = 512,
	.tile_size_x = 16,
	.tile_size_y = 16,
	.usc_min_output_registers_per_pix = 1,
	.virtual_address_space_bits = 40,
};

const struct pvr_device_quirks pvr_device_quirks_33_15_11_3 = {
	.has_brn63553 = true,
};

struct pvr_device_enhancements pvr_device_enhancements_33_15_11_3 = {
	.has_ern35421 = true,
	.has_ern38748 = true,
};

const struct pvr_device_features pvr_device_36_V_104_796 = {
	.has_cdm_control_stream_format = true,
	.has_common_store_size_in_dwords = true,
	.has_compute = true,
	.has_compute_overlap = true,
	.has_fbcdc_algorithm = true,
	.has_gpu_multicore_support = true,
	.has_isp_max_tiles_in_flight = true,
	.has_isp_samples_per_pixel = true,
	.has_max_partitions = true,
	.has_num_clusters = true,
	.has_num_isp_ipp_pipes = true,
	.has_num_raster_pipes = true,
	.has_phys_bus_width = true,
	.has_riscv_fw_processor = true,
	.has_roguexe = true,
	.has_simple_internal_parameter_format = true,
	.has_slc_cache_line_size_in_bits = true,
	.has_sys_bus_secure_reset = true,
	.has_tile_size_x = true,
	.has_tile_size_y = true,
	.has_tpu_dm_global_registers = true,
	.has_usc_min_output_registers_per_pix = true,
	.has_virtual_address_space_bits = true,
	.has_xe_memory_hierarchy = true,
	.has_xpu_max_slaves = true,

	.cdm_control_stream_format = 1,
	.common_store_size_in_dwords = 1344U * 4U * 4U,
	.fbcdc_algorithm = 50,
	.isp_max_tiles_in_flight = 6,
	.isp_samples_per_pixel = 4,
	.max_partitions = 16,
	.num_clusters = 1,
	.num_isp_ipp_pipes = 6,
	.num_raster_pipes = 1,
	.phys_bus_width = 36,
	.riscv_fw_processor = true,
	.simple_internal_parameter_format = 2,
	.slc_cache_line_size_in_bits = 512,
	.tile_size_x = 16,
	.tile_size_y = 16,
	.usc_min_output_registers_per_pix = 2,
	.virtual_address_space_bits = 40,
	.xpu_max_slaves = 3,
};

const struct pvr_device_quirks pvr_device_quirks_36_53_104_796 = {
	.has_brn44079 = true,
};

struct pvr_device_enhancements pvr_device_enhancements_36_53_104_796 = {
	.has_ern35421 = true,
	.has_ern38748 = true,
};

/**
 * pvr_device_info_init() - Initialize a PowerVR device's hardware features and quirks
 * @pvr_dev: Target PowerVR device.
 *
 * This function relies on &pvr_dev.gpu_id having already been initialized. If
 * PowerVR device version is supported then sets &pvr_dev.features and &pvr_dev.quirks.
 *
 * Return:
 *  * 0 on success, or
 *  * -%ENODEV if the device is not supported.
 */
int
pvr_device_info_init(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct pvr_gpu_id *gpu_id = &pvr_dev->gpu_id;
	const u64 bvnc = pvr_gpu_id_to_packed_bvnc(gpu_id);

	/*
	 * This macro results in a "Macros with multiple statements should be
	 * enclosed in a do - while loop" checkpatch error. However, following
	 * this advice would make the macro look a bit odd and isn't necessary
	 * in this particular case, as the macro has a very specific use and a
	 * very limited lifetime. The error can therefore be ignored.
	 */
#define CASE_PACKED_BVNC_DEVICE_INFO(b, v, n, c)                  \
	case PVR_PACKED_BVNC(b, v, n, c):                         \
		pvr_dev->features = pvr_device_##b##_V_##n##_##c; \
		pvr_dev->quirks = pvr_device_quirks_##b##_##v##_##n##_##c; \
		pvr_dev->enhancements = pvr_device_enhancements_##b##_##v##_##n##_##c; \
		return 0

	switch (bvnc) {
		CASE_PACKED_BVNC_DEVICE_INFO(4, 40, 2, 51);
		CASE_PACKED_BVNC_DEVICE_INFO(33, 15, 11, 3);
		CASE_PACKED_BVNC_DEVICE_INFO(36, 53, 104, 796);
	}

#undef CASE_PACKED_BVNC_DEVICE_INFO

	drm_warn(drm_dev, "Unsupported BVNC: %u.%u.%u.%u\n", gpu_id->b,
		 gpu_id->v, gpu_id->n, gpu_id->c);

	return -ENODEV;
}
