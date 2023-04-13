// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#include "pvr_free_list.h"
#include "pvr_hwrt.h"
#include "pvr_gem.h"
#include "pvr_rogue_cr_defs_client.h"
#include "pvr_rogue_fwif.h"

#include <drm/drm_gem.h>
#include <linux/bitops.h>
#include <linux/math.h>
#include <linux/slab.h>
#include <linux/xarray.h>
#include <uapi/drm/pvr_drm.h>

static_assert(ROGUE_FWIF_NUM_RTDATAS == 2);
static_assert(ROGUE_FWIF_NUM_GEOMDATAS == 1);
static_assert(ROGUE_FWIF_NUM_RTDATA_FREELISTS == 2);

/**
 * struct pvr_rt_mtile_info - Render target macrotile information
 */
struct pvr_rt_mtile_info {
	u32 mtile_x[3];
	u32 mtile_y[3];
	u32 tile_max_x;
	u32 tile_max_y;
	u32 tile_size_x;
	u32 tile_size_y;
	u32 num_tiles_x;
	u32 num_tiles_y;
};

/* Size of Shadow Render Target Cache entry */
#define SRTC_ENTRY_SIZE sizeof(u32)
/* Size of Renders Accumulation Array entry */
#define RAA_ENTRY_SIZE sizeof(u32)

static int
hwrt_init_kernel_structure(struct pvr_file *pvr_file,
			   struct drm_pvr_ioctl_create_hwrt_dataset_args *args,
			   struct pvr_hwrt_dataset *hwrt)
{
	struct pvr_device *pvr_dev = pvr_file->pvr_dev;
	int err;
	int i;

	hwrt->pvr_dev = pvr_dev;

	/* Get pointers to the free lists */
	for (i = 0; i < ARRAY_SIZE(hwrt->free_lists); i++) {
		hwrt->free_lists[i] = pvr_free_list_lookup(pvr_file,  args->free_list_handles[i]);
		if (!hwrt->free_lists[i]) {
			err = -EINVAL;
			goto err_put_free_lists;
		}
	}

	if (hwrt->free_lists[ROGUE_FW_LOCAL_FREELIST]->current_pages <
	    pvr_get_free_list_min_pages(pvr_dev)) {
		err = -EINVAL;
		goto err_put_free_lists;
	}

	return 0;

err_put_free_lists:
	for (i = 0; i < ARRAY_SIZE(hwrt->free_lists); i++) {
		pvr_free_list_put(hwrt->free_lists[i]);
		hwrt->free_lists[i] = NULL;
	}

	return err;
}

static void
hwrt_fini_kernel_structure(struct pvr_hwrt_dataset *hwrt)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hwrt->free_lists); i++) {
		pvr_free_list_put(hwrt->free_lists[i]);
		hwrt->free_lists[i] = NULL;
	}
}

static void
hwrt_fini_common_fw_structure(struct pvr_hwrt_dataset *hwrt)
{
	pvr_fw_object_release(hwrt->common_fw_obj);
}

static int
get_cr_isp_mtile_size_val(struct pvr_device *pvr_dev, u32 samples,
			  struct pvr_rt_mtile_info *info, u32 *value_out)
{
	u32 x = info->mtile_x[0];
	u32 y = info->mtile_y[0];
	u32 samples_per_pixel;
	int err;

	err = PVR_FEATURE_VALUE(pvr_dev, isp_samples_per_pixel, &samples_per_pixel);
	if (err)
		goto err_out;

	if (samples_per_pixel == 1) {
		if (samples >= 4)
			x <<= 1;
		if (samples >= 2)
			y <<= 1;
	} else if (samples_per_pixel == 2) {
		if (samples >= 8)
			x <<= 1;
		if (samples >= 4)
			y <<= 1;
	} else if (samples_per_pixel == 4) {
		if (samples >= 8)
			y <<= 1;
	} else {
		WARN(true, "Unsupported ISP samples per pixel value");
		err = -EINVAL;
		goto err_out;
	}

	*value_out = ((x << ROGUE_CR_ISP_MTILE_SIZE_X_SHIFT) & ~ROGUE_CR_ISP_MTILE_SIZE_X_CLRMSK) |
		     ((y << ROGUE_CR_ISP_MTILE_SIZE_Y_SHIFT) & ~ROGUE_CR_ISP_MTILE_SIZE_Y_CLRMSK);

err_out:
	return err;
}

static int
get_cr_multisamplectl_val(u32 samples, bool y_flip, u64 *value_out)
{
	static const struct {
		u8 x[8];
		u8 y[8];
	} sample_positions[4] = {
		/* 1 sample */
		{
			.x = { 8 },
			.y = { 8 },
		},
		/* 2 samples */
		{
			.x = { 12, 4 },
			.y = { 12, 4 },
		},
		/* 4 samples */
		{
			.x = { 6, 14, 2, 10 },
			.y = { 2, 6, 10, 14 },
		},
		/* 8 samples */
		{
			.x = { 9, 7, 13, 5, 3, 1, 11, 15 },
			.y = { 5, 11, 9, 3, 13, 7, 15, 1 },
		},
	};
	int idx = fls(samples) - 1;
	u64 value = 0;
	u32 i;

	if (idx < 0 || idx > 3)
		return -EINVAL;

	for (i = 0; i < 8; i++) {
		value |= sample_positions[idx].x[i] << (i * 8);
		if (y_flip)
			value = ((16 - sample_positions[idx].y[i]) & 0xf) << (i * 8 + 4);
		else
			value = (sample_positions[idx].y[i]) << (i * 8 + 4);
	}

	*value_out = value;

	return 0;
}

static int
get_cr_te_aa_val(struct pvr_device *pvr_dev, u32 samples, u32 *value_out)
{
	u32 samples_per_pixel;
	u32 value = 0;
	int err = 0;

	err = PVR_FEATURE_VALUE(pvr_dev, isp_samples_per_pixel, &samples_per_pixel);
	if (err)
		goto err_out;

	switch (samples_per_pixel) {
	case 1:
		if (samples >= 2)
			value |= ROGUE_CR_TE_AA_Y_EN;
		if (samples >= 4)
			value |= ROGUE_CR_TE_AA_X_EN;
		break;
	case 2:
		if (samples >= 2)
			value |= ROGUE_CR_TE_AA_X2_EN;
		if (samples >= 4)
			value |= ROGUE_CR_TE_AA_Y_EN;
		if (samples >= 8)
			value |= ROGUE_CR_TE_AA_X_EN;
		break;
	case 4:
		if (samples >= 2)
			value |= ROGUE_CR_TE_AA_X2_EN;
		if (samples >= 4)
			value |= ROGUE_CR_TE_AA_Y2_EN;
		if (samples >= 8)
			value |= ROGUE_CR_TE_AA_Y_EN;
		break;
	default:
		WARN(true, "Unsupported ISP samples per pixel value");
		err = -EINVAL;
		goto err_out;
	}

	*value_out = value;

err_out:
	return err;
}

static int
hwrt_init_common_fw_structure(struct pvr_file *pvr_file,
			      struct drm_pvr_ioctl_create_hwrt_dataset_args *args,
			      struct pvr_hwrt_dataset *hwrt)
{
	struct drm_pvr_create_hwrt_geom_data_args *geom_data_args = &args->geom_data_args;
	struct pvr_device *pvr_dev = pvr_file->pvr_dev;
	struct rogue_fwif_hwrtdata_common *hwrt_data_common_fw;
	struct pvr_rt_mtile_info info;
	int err;

	PVR_FEATURE_VALUE(pvr_dev, tile_size_x, &info.tile_size_x);
	PVR_FEATURE_VALUE(pvr_dev, tile_size_y, &info.tile_size_y);

	info.num_tiles_x = DIV_ROUND_UP(args->width, info.tile_size_x);
	info.num_tiles_y = DIV_ROUND_UP(args->height, info.tile_size_y);

	if (PVR_HAS_FEATURE(pvr_dev, simple_internal_parameter_format)) {
		u32 parameter_format;

		PVR_FEATURE_VALUE(pvr_dev, simple_internal_parameter_format, &parameter_format);
		WARN_ON(parameter_format != 2);

		/*
		 * Set up 16 macrotiles with a multiple of 2x2 tiles per macrotile, which is
		 * aligned to a tile group.
		 */
		info.mtile_x[0] = DIV_ROUND_UP(info.num_tiles_x, 8) * 2;
		info.mtile_y[0] = DIV_ROUND_UP(info.num_tiles_y, 8) * 2;
		info.mtile_x[1] = 0;
		info.mtile_y[1] = 0;
		info.mtile_x[2] = 0;
		info.mtile_y[2] = 0;
		info.tile_max_x = round_up(info.num_tiles_x, 2) - 1;
		info.tile_max_y = round_up(info.num_tiles_y, 2) - 1;
	} else {
		/* Set up 16 macrotiles with a multiple of 4x4 tiles per macrotile. */
		info.mtile_x[0] = round_up(DIV_ROUND_UP(info.num_tiles_x, 4), 4);
		info.mtile_y[0] = round_up(DIV_ROUND_UP(info.num_tiles_y, 4), 4);
		info.mtile_x[1] = info.mtile_x[0] * 2;
		info.mtile_y[1] = info.mtile_y[0] * 2;
		info.mtile_x[2] = info.mtile_x[0] * 3;
		info.mtile_y[2] = info.mtile_y[0] * 3;
		info.tile_max_x = info.num_tiles_x - 1;
		info.tile_max_y = info.num_tiles_y - 1;
	}

	/*
	 * Create and map the FW structure so we can initialise it. This is not
	 * accessed on the CPU side post-initialisation so the mapping lifetime
	 * is only for this function.
	 */
	hwrt_data_common_fw =
		pvr_gem_create_and_map_fw_object(pvr_dev,
						 sizeof(*hwrt_data_common_fw),
						 PVR_BO_FW_FLAGS_DEVICE_UNCACHED |
						 DRM_PVR_BO_CREATE_ZEROED, &hwrt->common_fw_obj);
	if (IS_ERR(hwrt_data_common_fw)) {
		err = PTR_ERR(hwrt_data_common_fw);
		goto err_out;
	}

	hwrt_data_common_fw->geom_caches_need_zeroing = false;

	hwrt_data_common_fw->isp_merge_lower_x = args->isp_merge_lower_x;
	hwrt_data_common_fw->isp_merge_lower_y = args->isp_merge_lower_y;
	hwrt_data_common_fw->isp_merge_upper_x = args->isp_merge_upper_x;
	hwrt_data_common_fw->isp_merge_upper_y = args->isp_merge_upper_y;
	hwrt_data_common_fw->isp_merge_scale_x = args->isp_merge_scale_x;
	hwrt_data_common_fw->isp_merge_scale_y = args->isp_merge_scale_y;

	err = get_cr_multisamplectl_val(args->samples, false,
					&hwrt_data_common_fw->multi_sample_ctl);
	if (err)
		goto err_put_fw_obj;

	err = get_cr_multisamplectl_val(args->samples, true,
					&hwrt_data_common_fw->flipped_multi_sample_ctl);
	if (err)
		goto err_put_fw_obj;

	hwrt_data_common_fw->mtile_stride = info.mtile_x[0] * info.mtile_y[0];

	err = get_cr_te_aa_val(pvr_dev, args->samples, &hwrt_data_common_fw->teaa);
	if (err)
		goto err_put_fw_obj;

	hwrt_data_common_fw->screen_pixel_max =
		(((args->width - 1) << ROGUE_CR_PPP_SCREEN_PIXXMAX_SHIFT) &
		 ~ROGUE_CR_PPP_SCREEN_PIXXMAX_CLRMSK) |
		(((args->height - 1) << ROGUE_CR_PPP_SCREEN_PIXYMAX_SHIFT) &
		 ~ROGUE_CR_PPP_SCREEN_PIXYMAX_CLRMSK);

	hwrt_data_common_fw->te_screen =
		((info.tile_max_x << ROGUE_CR_TE_SCREEN_XMAX_SHIFT) &
		 ~ROGUE_CR_TE_SCREEN_XMAX_CLRMSK) |
		((info.tile_max_y << ROGUE_CR_TE_SCREEN_YMAX_SHIFT) &
		 ~ROGUE_CR_TE_SCREEN_YMAX_CLRMSK);
	hwrt_data_common_fw->te_mtile1 =
		((info.mtile_x[0] << ROGUE_CR_TE_MTILE1_X1_SHIFT) & ~ROGUE_CR_TE_MTILE1_X1_CLRMSK) |
		((info.mtile_x[1] << ROGUE_CR_TE_MTILE1_X2_SHIFT) & ~ROGUE_CR_TE_MTILE1_X2_CLRMSK) |
		((info.mtile_x[2] << ROGUE_CR_TE_MTILE1_X3_SHIFT) & ~ROGUE_CR_TE_MTILE1_X3_CLRMSK);
	hwrt_data_common_fw->te_mtile2 =
		((info.mtile_y[0] << ROGUE_CR_TE_MTILE2_Y1_SHIFT) & ~ROGUE_CR_TE_MTILE2_Y1_CLRMSK) |
		((info.mtile_y[1] << ROGUE_CR_TE_MTILE2_Y2_SHIFT) & ~ROGUE_CR_TE_MTILE2_Y2_CLRMSK) |
		((info.mtile_y[2] << ROGUE_CR_TE_MTILE2_Y3_SHIFT) & ~ROGUE_CR_TE_MTILE2_Y3_CLRMSK);

	err = get_cr_isp_mtile_size_val(pvr_dev, args->samples, &info,
					&hwrt_data_common_fw->isp_mtile_size);
	if (err)
		goto err_put_fw_obj;

	hwrt_data_common_fw->tpc_stride = geom_data_args->tpc_stride;
	hwrt_data_common_fw->tpc_size = geom_data_args->tpc_size;

	hwrt_data_common_fw->rgn_header_size = args->region_header_size;

	pvr_fw_object_vunmap(hwrt->common_fw_obj, false);

	return 0;

err_put_fw_obj:
	pvr_fw_object_vunmap(hwrt->common_fw_obj, false);

err_out:
	return err;
}

static int
hwrt_data_init_fw_structure(struct pvr_file *pvr_file,
			    struct pvr_hwrt_dataset *hwrt,
			    struct drm_pvr_ioctl_create_hwrt_dataset_args *args,
			    struct drm_pvr_create_hwrt_rt_data_args *rt_data_args,
			    struct pvr_hwrt_data *hwrt_data)
{
	struct drm_pvr_create_hwrt_geom_data_args *geom_data_args = &args->geom_data_args;
	struct pvr_device *pvr_dev = pvr_file->pvr_dev;
	struct rogue_fwif_rta_ctl *rta_ctl;
	int free_list_i;
	int err;

	hwrt_data->fw_data = pvr_gem_create_and_map_fw_object(pvr_dev, sizeof(*hwrt_data->fw_data),
							      PVR_BO_FW_FLAGS_DEVICE_UNCACHED |
							      DRM_PVR_BO_CREATE_ZEROED,
							      &hwrt_data->fw_obj);
	if (IS_ERR(hwrt_data->fw_data)) {
		err = PTR_ERR(hwrt_data->fw_data);
		goto err_out;
	}

	pvr_gem_get_fw_addr(hwrt->common_fw_obj, &hwrt_data->fw_data->hwrt_data_common_fw_addr);

	for (free_list_i = 0; free_list_i < ARRAY_SIZE(hwrt->free_lists); free_list_i++) {
		pvr_gem_get_fw_addr(hwrt->free_lists[free_list_i]->fw_obj,
				    &hwrt_data->fw_data->freelists_fw_addr[free_list_i]);
	}

	hwrt_data->fw_data->tail_ptrs_dev_addr = geom_data_args->tpc_dev_addr;
	hwrt_data->fw_data->vheap_table_dev_addr = geom_data_args->vheap_table_dev_addr;
	hwrt_data->fw_data->rtc_dev_addr = geom_data_args->rtc_dev_addr;

	hwrt_data->fw_data->pm_mlist_dev_addr = rt_data_args->pm_mlist_dev_addr;
	hwrt_data->fw_data->macrotile_array_dev_addr = rt_data_args->macrotile_array_dev_addr;
	hwrt_data->fw_data->rgn_header_dev_addr = rt_data_args->region_header_dev_addr;

	rta_ctl = &hwrt_data->fw_data->rta_ctl;

	rta_ctl->render_target_index = 0;
	rta_ctl->active_render_targets = 0;
	rta_ctl->valid_render_targets_fw_addr = 0;
	rta_ctl->rta_num_partial_renders_fw_addr = 0;
	rta_ctl->max_rts = args->layers;

	if (args->layers > 1) {
		err = pvr_gem_create_fw_object(pvr_dev, args->layers * SRTC_ENTRY_SIZE,
					       PVR_BO_FW_FLAGS_DEVICE_UNCACHED |
					       DRM_PVR_BO_CREATE_ZEROED,
					       &hwrt_data->srtc_obj);
		if (err)
			goto err_put_fw_obj;
		pvr_gem_get_fw_addr(hwrt_data->srtc_obj, &rta_ctl->valid_render_targets_fw_addr);

		err = pvr_gem_create_fw_object(pvr_dev, args->layers * RAA_ENTRY_SIZE,
					       PVR_BO_FW_FLAGS_DEVICE_UNCACHED |
					       DRM_PVR_BO_CREATE_ZEROED,
					       &hwrt_data->raa_obj);
		if (err)
			goto err_put_shadow_rt_cache;
		pvr_gem_get_fw_addr(hwrt_data->raa_obj, &rta_ctl->rta_num_partial_renders_fw_addr);
	}

	pvr_free_list_add_hwrt(hwrt->free_lists[0], hwrt_data);

	return 0;

err_put_shadow_rt_cache:
	pvr_fw_object_release(hwrt_data->srtc_obj);

err_put_fw_obj:
	pvr_fw_object_vunmap(hwrt_data->fw_obj, false);
	pvr_fw_object_release(hwrt_data->fw_obj);

err_out:
	return err;
}

static void
hwrt_data_fini_fw_structure(struct pvr_hwrt_dataset *hwrt, int hwrt_nr)
{
	struct pvr_hwrt_data *hwrt_data = &hwrt->data[hwrt_nr];

	pvr_free_list_remove_hwrt(hwrt->free_lists[0], hwrt_data);

	if (hwrt->max_rts > 1) {
		pvr_fw_object_release(hwrt_data->raa_obj);
		pvr_fw_object_release(hwrt_data->srtc_obj);
	}

	pvr_fw_object_vunmap(hwrt_data->fw_obj, false);
	pvr_fw_object_release(hwrt_data->fw_obj);
}

/**
 * pvr_hwrt_dataset_create() - Create a new HWRT dataset
 * @pvr_file: Pointer to pvr_file structure.
 * @args: Creation arguments from userspace.
 *
 * Return:
 *  * Pointer to new HWRT, or
 *  * ERR_PTR(-%ENOMEM) on out of memory.
 */
struct pvr_hwrt_dataset *
pvr_hwrt_dataset_create(struct pvr_file *pvr_file,
			struct drm_pvr_ioctl_create_hwrt_dataset_args *args)
{
	struct pvr_hwrt_dataset *hwrt;
	int err;

	/* Create and fill out the kernel structure */
	hwrt = kzalloc(sizeof(*hwrt), GFP_KERNEL);

	if (!hwrt)
		return ERR_PTR(-ENOMEM);

	kref_init(&hwrt->ref_count);

	err = hwrt_init_kernel_structure(pvr_file, args, hwrt);
	if (err < 0)
		goto err_free;

	err = hwrt_init_common_fw_structure(pvr_file, args, hwrt);
	if (err < 0)
		goto err_free;

	for (int i = 0; i < ARRAY_SIZE(hwrt->data); i++) {
		err = hwrt_data_init_fw_structure(pvr_file, hwrt, args,
						  &args->rt_data_args[i],
						  &hwrt->data[i]);
		if (err < 0) {
			i--;
			/* Destroy already created structures. */
			for (; i >= 0; i--)
				hwrt_data_fini_fw_structure(hwrt, i);
			goto err_free;
		}

		hwrt->data[i].hwrt_dataset = hwrt;
	}

	return hwrt;

err_free:
	pvr_hwrt_dataset_put(hwrt);

	return ERR_PTR(err);
}

static void
pvr_hwrt_dataset_release(struct kref *ref_count)
{
	struct pvr_hwrt_dataset *hwrt =
		container_of(ref_count, struct pvr_hwrt_dataset, ref_count);

	for (int i = ARRAY_SIZE(hwrt->data) - 1; i >= 0; i--) {
		WARN_ON(pvr_fw_structure_cleanup(hwrt->pvr_dev, ROGUE_FWIF_CLEANUP_HWRTDATA,
						 hwrt->data[i].fw_obj, 0));
		hwrt_data_fini_fw_structure(hwrt, i);
	}

	hwrt_fini_common_fw_structure(hwrt);
	hwrt_fini_kernel_structure(hwrt);

	kfree(hwrt);
}

/**
 * pvr_destroy_hwrt_datasets_for_file: Destroy any HWRT datasets associated
 * with the given file.
 * @pvr_file: Pointer to pvr_file structure.
 *
 * Removes all HWRT datasets associated with @pvr_file from the device
 * hwrt_dataset list and drops initial references. HWRT datasets will then be
 * destroyed once all outstanding references are dropped.
 */
void pvr_destroy_hwrt_datasets_for_file(struct pvr_file *pvr_file)
{
	struct pvr_hwrt_dataset *hwrt;
	unsigned long handle;

	xa_for_each(&pvr_file->hwrt_handles, handle, hwrt) {
		(void)hwrt;
		pvr_hwrt_dataset_put(xa_erase(&pvr_file->hwrt_handles, handle));
	}
}

/**
 * pvr_hwrt_dataset_put() - Release reference on HWRT dataset
 * @hwrt: Pointer to HWRT dataset to release reference on
 */
void
pvr_hwrt_dataset_put(struct pvr_hwrt_dataset *hwrt)
{
	if (hwrt)
		kref_put(&hwrt->ref_count, pvr_hwrt_dataset_release);
}
