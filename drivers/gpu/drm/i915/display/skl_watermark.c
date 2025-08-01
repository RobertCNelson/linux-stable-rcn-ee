// SPDX-License-Identifier: MIT
/*
 * Copyright © 2022 Intel Corporation
 */

#include <linux/debugfs.h>

#include <drm/drm_blend.h>
#include <drm/drm_file.h>
#include <drm/drm_print.h>

#include "soc/intel_dram.h"
#include "i915_reg.h"
#include "i915_utils.h"
#include "i9xx_wm.h"
#include "intel_atomic.h"
#include "intel_bw.h"
#include "intel_cdclk.h"
#include "intel_crtc.h"
#include "intel_cursor_regs.h"
#include "intel_de.h"
#include "intel_display.h"
#include "intel_display_power.h"
#include "intel_display_regs.h"
#include "intel_display_rpm.h"
#include "intel_display_types.h"
#include "intel_fb.h"
#include "intel_fixed.h"
#include "intel_flipq.h"
#include "intel_pcode.h"
#include "intel_plane.h"
#include "intel_wm.h"
#include "skl_universal_plane_regs.h"
#include "skl_watermark.h"
#include "skl_watermark_regs.h"

struct intel_dbuf_state {
	struct intel_global_state base;

	struct skl_ddb_entry ddb[I915_MAX_PIPES];
	unsigned int weight[I915_MAX_PIPES];
	u8 slices[I915_MAX_PIPES];
	u8 enabled_slices;
	u8 active_pipes;
	u8 mdclk_cdclk_ratio;
	bool joined_mbus;
};

#define to_intel_dbuf_state(global_state) \
	container_of_const((global_state), struct intel_dbuf_state, base)

#define intel_atomic_get_old_dbuf_state(state) \
	to_intel_dbuf_state(intel_atomic_get_old_global_obj_state(state, &to_intel_display(state)->dbuf.obj))
#define intel_atomic_get_new_dbuf_state(state) \
	to_intel_dbuf_state(intel_atomic_get_new_global_obj_state(state, &to_intel_display(state)->dbuf.obj))

static void skl_sagv_disable(struct intel_display *display);

/* Stores plane specific WM parameters */
struct skl_wm_params {
	bool x_tiled, y_tiled;
	bool rc_surface;
	bool is_planar;
	u32 width;
	u8 cpp;
	u32 plane_pixel_rate;
	u32 y_min_scanlines;
	u32 plane_bytes_per_line;
	uint_fixed_16_16_t plane_blocks_per_line;
	uint_fixed_16_16_t y_tile_minimum;
	u32 linetime_us;
	u32 dbuf_block_size;
};

u8 intel_enabled_dbuf_slices_mask(struct intel_display *display)
{
	u8 enabled_slices = 0;
	enum dbuf_slice slice;

	for_each_dbuf_slice(display, slice) {
		if (intel_de_read(display, DBUF_CTL_S(slice)) & DBUF_POWER_STATE)
			enabled_slices |= BIT(slice);
	}

	return enabled_slices;
}

/*
 * FIXME: We still don't have the proper code detect if we need to apply the WA,
 * so assume we'll always need it in order to avoid underruns.
 */
static bool skl_needs_memory_bw_wa(struct intel_display *display)
{
	return DISPLAY_VER(display) == 9;
}

bool
intel_has_sagv(struct intel_display *display)
{
	return HAS_SAGV(display) && display->sagv.status != I915_SAGV_NOT_CONTROLLED;
}

static u32
intel_sagv_block_time(struct intel_display *display)
{
	if (DISPLAY_VER(display) >= 14) {
		u32 val;

		val = intel_de_read(display, MTL_LATENCY_SAGV);

		return REG_FIELD_GET(MTL_LATENCY_QCLK_SAGV, val);
	} else if (DISPLAY_VER(display) >= 12) {
		u32 val = 0;
		int ret;

		ret = intel_pcode_read(display->drm,
				       GEN12_PCODE_READ_SAGV_BLOCK_TIME_US,
				       &val, NULL);
		if (ret) {
			drm_dbg_kms(display->drm, "Couldn't read SAGV block time!\n");
			return 0;
		}

		return val;
	} else if (DISPLAY_VER(display) == 11) {
		return 10;
	} else if (HAS_SAGV(display)) {
		return 30;
	} else {
		return 0;
	}
}

static void intel_sagv_init(struct intel_display *display)
{
	if (!HAS_SAGV(display))
		display->sagv.status = I915_SAGV_NOT_CONTROLLED;

	/*
	 * Probe to see if we have working SAGV control.
	 * For icl+ this was already determined by intel_bw_init_hw().
	 */
	if (DISPLAY_VER(display) < 11)
		skl_sagv_disable(display);

	drm_WARN_ON(display->drm, display->sagv.status == I915_SAGV_UNKNOWN);

	display->sagv.block_time_us = intel_sagv_block_time(display);

	drm_dbg_kms(display->drm, "SAGV supported: %s, original SAGV block time: %u us\n",
		    str_yes_no(intel_has_sagv(display)), display->sagv.block_time_us);

	/* avoid overflow when adding with wm0 latency/etc. */
	if (drm_WARN(display->drm, display->sagv.block_time_us > U16_MAX,
		     "Excessive SAGV block time %u, ignoring\n",
		     display->sagv.block_time_us))
		display->sagv.block_time_us = 0;

	if (!intel_has_sagv(display))
		display->sagv.block_time_us = 0;
}

/*
 * SAGV dynamically adjusts the system agent voltage and clock frequencies
 * depending on power and performance requirements. The display engine access
 * to system memory is blocked during the adjustment time. Because of the
 * blocking time, having this enabled can cause full system hangs and/or pipe
 * underruns if we don't meet all of the following requirements:
 *
 *  - <= 1 pipe enabled
 *  - All planes can enable watermarks for latencies >= SAGV engine block time
 *  - We're not using an interlaced display configuration
 */
static void skl_sagv_enable(struct intel_display *display)
{
	int ret;

	if (!intel_has_sagv(display))
		return;

	if (display->sagv.status == I915_SAGV_ENABLED)
		return;

	drm_dbg_kms(display->drm, "Enabling SAGV\n");
	ret = intel_pcode_write(display->drm, GEN9_PCODE_SAGV_CONTROL,
				GEN9_SAGV_ENABLE);

	/* We don't need to wait for SAGV when enabling */

	/*
	 * Some skl systems, pre-release machines in particular,
	 * don't actually have SAGV.
	 */
	if (display->platform.skylake && ret == -ENXIO) {
		drm_dbg(display->drm, "No SAGV found on system, ignoring\n");
		display->sagv.status = I915_SAGV_NOT_CONTROLLED;
		return;
	} else if (ret < 0) {
		drm_err(display->drm, "Failed to enable SAGV\n");
		return;
	}

	display->sagv.status = I915_SAGV_ENABLED;
}

static void skl_sagv_disable(struct intel_display *display)
{
	int ret;

	if (!intel_has_sagv(display))
		return;

	if (display->sagv.status == I915_SAGV_DISABLED)
		return;

	drm_dbg_kms(display->drm, "Disabling SAGV\n");
	/* bspec says to keep retrying for at least 1 ms */
	ret = intel_pcode_request(display->drm, GEN9_PCODE_SAGV_CONTROL,
				  GEN9_SAGV_DISABLE,
				  GEN9_SAGV_IS_DISABLED, GEN9_SAGV_IS_DISABLED, 1);
	/*
	 * Some skl systems, pre-release machines in particular,
	 * don't actually have SAGV.
	 */
	if (display->platform.skylake && ret == -ENXIO) {
		drm_dbg(display->drm, "No SAGV found on system, ignoring\n");
		display->sagv.status = I915_SAGV_NOT_CONTROLLED;
		return;
	} else if (ret < 0) {
		drm_err(display->drm, "Failed to disable SAGV (%d)\n", ret);
		return;
	}

	display->sagv.status = I915_SAGV_DISABLED;
}

static void skl_sagv_pre_plane_update(struct intel_atomic_state *state)
{
	struct intel_display *display = to_intel_display(state);
	const struct intel_bw_state *new_bw_state =
		intel_atomic_get_new_bw_state(state);

	if (!new_bw_state)
		return;

	if (!intel_bw_can_enable_sagv(display, new_bw_state))
		skl_sagv_disable(display);
}

static void skl_sagv_post_plane_update(struct intel_atomic_state *state)
{
	struct intel_display *display = to_intel_display(state);
	const struct intel_bw_state *new_bw_state =
		intel_atomic_get_new_bw_state(state);

	if (!new_bw_state)
		return;

	if (intel_bw_can_enable_sagv(display, new_bw_state))
		skl_sagv_enable(display);
}

void intel_sagv_pre_plane_update(struct intel_atomic_state *state)
{
	struct intel_display *display = to_intel_display(state);

	/*
	 * Just return if we can't control SAGV or don't have it.
	 * This is different from situation when we have SAGV but just can't
	 * afford it due to DBuf limitation - in case if SAGV is completely
	 * disabled in a BIOS, we are not even allowed to send a PCode request,
	 * as it will throw an error. So have to check it here.
	 */
	if (!intel_has_sagv(display))
		return;

	if (DISPLAY_VER(display) >= 11)
		icl_sagv_pre_plane_update(state);
	else
		skl_sagv_pre_plane_update(state);
}

void intel_sagv_post_plane_update(struct intel_atomic_state *state)
{
	struct intel_display *display = to_intel_display(state);

	/*
	 * Just return if we can't control SAGV or don't have it.
	 * This is different from situation when we have SAGV but just can't
	 * afford it due to DBuf limitation - in case if SAGV is completely
	 * disabled in a BIOS, we are not even allowed to send a PCode request,
	 * as it will throw an error. So have to check it here.
	 */
	if (!intel_has_sagv(display))
		return;

	if (DISPLAY_VER(display) >= 11)
		icl_sagv_post_plane_update(state);
	else
		skl_sagv_post_plane_update(state);
}

static bool skl_crtc_can_enable_sagv(const struct intel_crtc_state *crtc_state)
{
	struct intel_display *display = to_intel_display(crtc_state);
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	enum plane_id plane_id;
	int max_level = INT_MAX;

	if (!intel_has_sagv(display))
		return false;

	if (!crtc_state->hw.active)
		return true;

	if (crtc_state->hw.pipe_mode.flags & DRM_MODE_FLAG_INTERLACE)
		return false;

	for_each_plane_id_on_crtc(crtc, plane_id) {
		const struct skl_plane_wm *wm =
			&crtc_state->wm.skl.optimal.planes[plane_id];
		int level;

		/* Skip this plane if it's not enabled */
		if (!wm->wm[0].enable)
			continue;

		/* Find the highest enabled wm level for this plane */
		for (level = display->wm.num_levels - 1;
		     !wm->wm[level].enable; --level)
		     { }

		/* Highest common enabled wm level for all planes */
		max_level = min(level, max_level);
	}

	/* No enabled planes? */
	if (max_level == INT_MAX)
		return true;

	for_each_plane_id_on_crtc(crtc, plane_id) {
		const struct skl_plane_wm *wm =
			&crtc_state->wm.skl.optimal.planes[plane_id];

		/*
		 * All enabled planes must have enabled a common wm level that
		 * can tolerate memory latencies higher than sagv_block_time_us
		 */
		if (wm->wm[0].enable && !wm->wm[max_level].can_sagv)
			return false;
	}

	return true;
}

static bool tgl_crtc_can_enable_sagv(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	enum plane_id plane_id;

	if (!crtc_state->hw.active)
		return true;

	for_each_plane_id_on_crtc(crtc, plane_id) {
		const struct skl_plane_wm *wm =
			&crtc_state->wm.skl.optimal.planes[plane_id];

		if (wm->wm[0].enable && !wm->sagv.wm0.enable)
			return false;
	}

	return true;
}

bool intel_crtc_can_enable_sagv(const struct intel_crtc_state *crtc_state)
{
	struct intel_display *display = to_intel_display(crtc_state);

	if (!display->params.enable_sagv)
		return false;

	/*
	 * SAGV is initially forced off because its current
	 * state can't be queried from pcode. Allow SAGV to
	 * be enabled upon the first real commit.
	 */
	if (crtc_state->inherited)
		return false;

	if (DISPLAY_VER(display) >= 12)
		return tgl_crtc_can_enable_sagv(crtc_state);
	else
		return skl_crtc_can_enable_sagv(crtc_state);
}

static u16 skl_ddb_entry_init(struct skl_ddb_entry *entry,
			      u16 start, u16 end)
{
	entry->start = start;
	entry->end = end;

	return end;
}

static int intel_dbuf_slice_size(struct intel_display *display)
{
	return DISPLAY_INFO(display)->dbuf.size /
		hweight8(DISPLAY_INFO(display)->dbuf.slice_mask);
}

static void
skl_ddb_entry_for_slices(struct intel_display *display, u8 slice_mask,
			 struct skl_ddb_entry *ddb)
{
	int slice_size = intel_dbuf_slice_size(display);

	if (!slice_mask) {
		ddb->start = 0;
		ddb->end = 0;
		return;
	}

	ddb->start = (ffs(slice_mask) - 1) * slice_size;
	ddb->end = fls(slice_mask) * slice_size;

	WARN_ON(ddb->start >= ddb->end);
	WARN_ON(ddb->end > DISPLAY_INFO(display)->dbuf.size);
}

static unsigned int mbus_ddb_offset(struct intel_display *display, u8 slice_mask)
{
	struct skl_ddb_entry ddb;

	if (slice_mask & (BIT(DBUF_S1) | BIT(DBUF_S2)))
		slice_mask = BIT(DBUF_S1);
	else if (slice_mask & (BIT(DBUF_S3) | BIT(DBUF_S4)))
		slice_mask = BIT(DBUF_S3);

	skl_ddb_entry_for_slices(display, slice_mask, &ddb);

	return ddb.start;
}

u32 skl_ddb_dbuf_slice_mask(struct intel_display *display,
			    const struct skl_ddb_entry *entry)
{
	int slice_size = intel_dbuf_slice_size(display);
	enum dbuf_slice start_slice, end_slice;
	u8 slice_mask = 0;

	if (!skl_ddb_entry_size(entry))
		return 0;

	start_slice = entry->start / slice_size;
	end_slice = (entry->end - 1) / slice_size;

	/*
	 * Per plane DDB entry can in a really worst case be on multiple slices
	 * but single entry is anyway contiguous.
	 */
	while (start_slice <= end_slice) {
		slice_mask |= BIT(start_slice);
		start_slice++;
	}

	return slice_mask;
}

static unsigned int intel_crtc_ddb_weight(const struct intel_crtc_state *crtc_state)
{
	const struct drm_display_mode *pipe_mode = &crtc_state->hw.pipe_mode;
	int hdisplay, vdisplay;

	if (!crtc_state->hw.active)
		return 0;

	/*
	 * Watermark/ddb requirement highly depends upon width of the
	 * framebuffer, So instead of allocating DDB equally among pipes
	 * distribute DDB based on resolution/width of the display.
	 */
	drm_mode_get_hv_timing(pipe_mode, &hdisplay, &vdisplay);

	return hdisplay;
}

static void intel_crtc_dbuf_weights(const struct intel_dbuf_state *dbuf_state,
				    enum pipe for_pipe,
				    unsigned int *weight_start,
				    unsigned int *weight_end,
				    unsigned int *weight_total)
{
	struct intel_display *display = to_intel_display(dbuf_state->base.state->base.dev);
	enum pipe pipe;

	*weight_start = 0;
	*weight_end = 0;
	*weight_total = 0;

	for_each_pipe(display, pipe) {
		int weight = dbuf_state->weight[pipe];

		/*
		 * Do not account pipes using other slice sets
		 * luckily as of current BSpec slice sets do not partially
		 * intersect(pipes share either same one slice or same slice set
		 * i.e no partial intersection), so it is enough to check for
		 * equality for now.
		 */
		if (dbuf_state->slices[pipe] != dbuf_state->slices[for_pipe])
			continue;

		*weight_total += weight;
		if (pipe < for_pipe) {
			*weight_start += weight;
			*weight_end += weight;
		} else if (pipe == for_pipe) {
			*weight_end += weight;
		}
	}
}

static int
skl_crtc_allocate_ddb(struct intel_atomic_state *state, struct intel_crtc *crtc)
{
	struct intel_display *display = to_intel_display(crtc);
	unsigned int weight_total, weight_start, weight_end;
	const struct intel_dbuf_state *old_dbuf_state =
		intel_atomic_get_old_dbuf_state(state);
	struct intel_dbuf_state *new_dbuf_state =
		intel_atomic_get_new_dbuf_state(state);
	struct intel_crtc_state *crtc_state;
	struct skl_ddb_entry ddb_slices;
	enum pipe pipe = crtc->pipe;
	unsigned int mbus_offset = 0;
	u32 ddb_range_size;
	u32 dbuf_slice_mask;
	u32 start, end;
	int ret;

	if (new_dbuf_state->weight[pipe] == 0) {
		skl_ddb_entry_init(&new_dbuf_state->ddb[pipe], 0, 0);
		goto out;
	}

	dbuf_slice_mask = new_dbuf_state->slices[pipe];

	skl_ddb_entry_for_slices(display, dbuf_slice_mask, &ddb_slices);
	mbus_offset = mbus_ddb_offset(display, dbuf_slice_mask);
	ddb_range_size = skl_ddb_entry_size(&ddb_slices);

	intel_crtc_dbuf_weights(new_dbuf_state, pipe,
				&weight_start, &weight_end, &weight_total);

	start = ddb_range_size * weight_start / weight_total;
	end = ddb_range_size * weight_end / weight_total;

	skl_ddb_entry_init(&new_dbuf_state->ddb[pipe],
			   ddb_slices.start - mbus_offset + start,
			   ddb_slices.start - mbus_offset + end);

out:
	if (old_dbuf_state->slices[pipe] == new_dbuf_state->slices[pipe] &&
	    skl_ddb_entry_equal(&old_dbuf_state->ddb[pipe],
				&new_dbuf_state->ddb[pipe]))
		return 0;

	ret = intel_atomic_lock_global_state(&new_dbuf_state->base);
	if (ret)
		return ret;

	crtc_state = intel_atomic_get_crtc_state(&state->base, crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	/*
	 * Used for checking overlaps, so we need absolute
	 * offsets instead of MBUS relative offsets.
	 */
	crtc_state->wm.skl.ddb.start = mbus_offset + new_dbuf_state->ddb[pipe].start;
	crtc_state->wm.skl.ddb.end = mbus_offset + new_dbuf_state->ddb[pipe].end;

	drm_dbg_kms(display->drm,
		    "[CRTC:%d:%s] dbuf slices 0x%x -> 0x%x, ddb (%d - %d) -> (%d - %d), active pipes 0x%x -> 0x%x\n",
		    crtc->base.base.id, crtc->base.name,
		    old_dbuf_state->slices[pipe], new_dbuf_state->slices[pipe],
		    old_dbuf_state->ddb[pipe].start, old_dbuf_state->ddb[pipe].end,
		    new_dbuf_state->ddb[pipe].start, new_dbuf_state->ddb[pipe].end,
		    old_dbuf_state->active_pipes, new_dbuf_state->active_pipes);

	return 0;
}

static int skl_compute_wm_params(const struct intel_crtc_state *crtc_state,
				 int width, const struct drm_format_info *format,
				 u64 modifier, unsigned int rotation,
				 u32 plane_pixel_rate, struct skl_wm_params *wp,
				 int color_plane, unsigned int pan_x);

static void skl_compute_plane_wm(const struct intel_crtc_state *crtc_state,
				 struct intel_plane *plane,
				 int level,
				 unsigned int latency,
				 const struct skl_wm_params *wp,
				 const struct skl_wm_level *result_prev,
				 struct skl_wm_level *result /* out */);

static unsigned int skl_wm_latency(struct intel_display *display, int level,
				   const struct skl_wm_params *wp)
{
	unsigned int latency = display->wm.skl_latency[level];

	if (latency == 0)
		return 0;

	/*
	 * WaIncreaseLatencyIPCEnabled: kbl,cfl
	 * Display WA #1141: kbl,cfl
	 */
	if ((display->platform.kabylake || display->platform.coffeelake ||
	     display->platform.cometlake) && skl_watermark_ipc_enabled(display))
		latency += 4;

	if (skl_needs_memory_bw_wa(display) && wp && wp->x_tiled)
		latency += 15;

	return latency;
}

static unsigned int
skl_cursor_allocation(const struct intel_crtc_state *crtc_state,
		      int num_active)
{
	struct intel_display *display = to_intel_display(crtc_state);
	struct intel_plane *plane = to_intel_plane(crtc_state->uapi.crtc->cursor);
	struct skl_wm_level wm = {};
	int ret, min_ddb_alloc = 0;
	struct skl_wm_params wp;
	int level;

	ret = skl_compute_wm_params(crtc_state, 256,
				    drm_format_info(DRM_FORMAT_ARGB8888),
				    DRM_FORMAT_MOD_LINEAR,
				    DRM_MODE_ROTATE_0,
				    crtc_state->pixel_rate, &wp, 0, 0);
	drm_WARN_ON(display->drm, ret);

	for (level = 0; level < display->wm.num_levels; level++) {
		unsigned int latency = skl_wm_latency(display, level, &wp);

		skl_compute_plane_wm(crtc_state, plane, level, latency, &wp, &wm, &wm);
		if (wm.min_ddb_alloc == U16_MAX)
			break;

		min_ddb_alloc = wm.min_ddb_alloc;
	}

	return max(num_active == 1 ? 32 : 8, min_ddb_alloc);
}

static void skl_ddb_entry_init_from_hw(struct skl_ddb_entry *entry, u32 reg)
{
	skl_ddb_entry_init(entry,
			   REG_FIELD_GET(PLANE_BUF_START_MASK, reg),
			   REG_FIELD_GET(PLANE_BUF_END_MASK, reg));
	if (entry->end)
		entry->end++;
}

static void
skl_ddb_get_hw_plane_state(struct intel_display *display,
			   const enum pipe pipe,
			   const enum plane_id plane_id,
			   struct skl_ddb_entry *ddb,
			   struct skl_ddb_entry *ddb_y,
			   u16 *min_ddb, u16 *interim_ddb)
{
	u32 val;

	/* Cursor doesn't support NV12/planar, so no extra calculation needed */
	if (plane_id == PLANE_CURSOR) {
		val = intel_de_read(display, CUR_BUF_CFG(pipe));
		skl_ddb_entry_init_from_hw(ddb, val);
		return;
	}

	val = intel_de_read(display, PLANE_BUF_CFG(pipe, plane_id));
	skl_ddb_entry_init_from_hw(ddb, val);

	if (DISPLAY_VER(display) >= 30) {
		val = intel_de_read(display, PLANE_MIN_BUF_CFG(pipe, plane_id));

		*min_ddb = REG_FIELD_GET(PLANE_MIN_DBUF_BLOCKS_MASK, val);
		*interim_ddb = REG_FIELD_GET(PLANE_INTERIM_DBUF_BLOCKS_MASK, val);
	}

	if (DISPLAY_VER(display) >= 11)
		return;

	val = intel_de_read(display, PLANE_NV12_BUF_CFG(pipe, plane_id));
	skl_ddb_entry_init_from_hw(ddb_y, val);
}

static void skl_pipe_ddb_get_hw_state(struct intel_crtc *crtc,
				      struct skl_ddb_entry *ddb,
				      struct skl_ddb_entry *ddb_y,
				      u16 *min_ddb, u16 *interim_ddb)
{
	struct intel_display *display = to_intel_display(crtc);
	enum intel_display_power_domain power_domain;
	enum pipe pipe = crtc->pipe;
	intel_wakeref_t wakeref;
	enum plane_id plane_id;

	power_domain = POWER_DOMAIN_PIPE(pipe);
	wakeref = intel_display_power_get_if_enabled(display, power_domain);
	if (!wakeref)
		return;

	for_each_plane_id_on_crtc(crtc, plane_id)
		skl_ddb_get_hw_plane_state(display, pipe,
					   plane_id,
					   &ddb[plane_id],
					   &ddb_y[plane_id],
					   &min_ddb[plane_id],
					   &interim_ddb[plane_id]);

	intel_display_power_put(display, power_domain, wakeref);
}

struct dbuf_slice_conf_entry {
	u8 active_pipes;
	u8 dbuf_mask[I915_MAX_PIPES];
	bool join_mbus;
};

/*
 * Table taken from Bspec 12716
 * Pipes do have some preferred DBuf slice affinity,
 * plus there are some hardcoded requirements on how
 * those should be distributed for multipipe scenarios.
 * For more DBuf slices algorithm can get even more messy
 * and less readable, so decided to use a table almost
 * as is from BSpec itself - that way it is at least easier
 * to compare, change and check.
 */
static const struct dbuf_slice_conf_entry icl_allowed_dbufs[] =
/* Autogenerated with igt/tools/intel_dbuf_map tool: */
{
	{
		.active_pipes = BIT(PIPE_A),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1),
		},
	},
	{
		.active_pipes = BIT(PIPE_B),
		.dbuf_mask = {
			[PIPE_B] = BIT(DBUF_S1),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_B),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1),
			[PIPE_B] = BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_C),
		.dbuf_mask = {
			[PIPE_C] = BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_C),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1),
			[PIPE_C] = BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_B) | BIT(PIPE_C),
		.dbuf_mask = {
			[PIPE_B] = BIT(DBUF_S1),
			[PIPE_C] = BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_C),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1),
			[PIPE_B] = BIT(DBUF_S1),
			[PIPE_C] = BIT(DBUF_S2),
		},
	},
	{}
};

/*
 * Table taken from Bspec 49255
 * Pipes do have some preferred DBuf slice affinity,
 * plus there are some hardcoded requirements on how
 * those should be distributed for multipipe scenarios.
 * For more DBuf slices algorithm can get even more messy
 * and less readable, so decided to use a table almost
 * as is from BSpec itself - that way it is at least easier
 * to compare, change and check.
 */
static const struct dbuf_slice_conf_entry tgl_allowed_dbufs[] =
/* Autogenerated with igt/tools/intel_dbuf_map tool: */
{
	{
		.active_pipes = BIT(PIPE_A),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1) | BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_B),
		.dbuf_mask = {
			[PIPE_B] = BIT(DBUF_S1) | BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_B),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S2),
			[PIPE_B] = BIT(DBUF_S1),
		},
	},
	{
		.active_pipes = BIT(PIPE_C),
		.dbuf_mask = {
			[PIPE_C] = BIT(DBUF_S2) | BIT(DBUF_S1),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_C),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1),
			[PIPE_C] = BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_B) | BIT(PIPE_C),
		.dbuf_mask = {
			[PIPE_B] = BIT(DBUF_S1),
			[PIPE_C] = BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_C),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1),
			[PIPE_B] = BIT(DBUF_S1),
			[PIPE_C] = BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_D] = BIT(DBUF_S2) | BIT(DBUF_S1),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1),
			[PIPE_D] = BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_B) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_B] = BIT(DBUF_S1),
			[PIPE_D] = BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1),
			[PIPE_B] = BIT(DBUF_S1),
			[PIPE_D] = BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_C) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_C] = BIT(DBUF_S1),
			[PIPE_D] = BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_C) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1),
			[PIPE_C] = BIT(DBUF_S2),
			[PIPE_D] = BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_B) | BIT(PIPE_C) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_B] = BIT(DBUF_S1),
			[PIPE_C] = BIT(DBUF_S2),
			[PIPE_D] = BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_C) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1),
			[PIPE_B] = BIT(DBUF_S1),
			[PIPE_C] = BIT(DBUF_S2),
			[PIPE_D] = BIT(DBUF_S2),
		},
	},
	{}
};

static const struct dbuf_slice_conf_entry dg2_allowed_dbufs[] = {
	{
		.active_pipes = BIT(PIPE_A),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1) | BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_B),
		.dbuf_mask = {
			[PIPE_B] = BIT(DBUF_S1) | BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_B),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1),
			[PIPE_B] = BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_C),
		.dbuf_mask = {
			[PIPE_C] = BIT(DBUF_S3) | BIT(DBUF_S4),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_C),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1) | BIT(DBUF_S2),
			[PIPE_C] = BIT(DBUF_S3) | BIT(DBUF_S4),
		},
	},
	{
		.active_pipes = BIT(PIPE_B) | BIT(PIPE_C),
		.dbuf_mask = {
			[PIPE_B] = BIT(DBUF_S1) | BIT(DBUF_S2),
			[PIPE_C] = BIT(DBUF_S3) | BIT(DBUF_S4),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_C),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1),
			[PIPE_B] = BIT(DBUF_S2),
			[PIPE_C] = BIT(DBUF_S3) | BIT(DBUF_S4),
		},
	},
	{
		.active_pipes = BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_D] = BIT(DBUF_S3) | BIT(DBUF_S4),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1) | BIT(DBUF_S2),
			[PIPE_D] = BIT(DBUF_S3) | BIT(DBUF_S4),
		},
	},
	{
		.active_pipes = BIT(PIPE_B) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_B] = BIT(DBUF_S1) | BIT(DBUF_S2),
			[PIPE_D] = BIT(DBUF_S3) | BIT(DBUF_S4),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1),
			[PIPE_B] = BIT(DBUF_S2),
			[PIPE_D] = BIT(DBUF_S3) | BIT(DBUF_S4),
		},
	},
	{
		.active_pipes = BIT(PIPE_C) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_C] = BIT(DBUF_S3),
			[PIPE_D] = BIT(DBUF_S4),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_C) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1) | BIT(DBUF_S2),
			[PIPE_C] = BIT(DBUF_S3),
			[PIPE_D] = BIT(DBUF_S4),
		},
	},
	{
		.active_pipes = BIT(PIPE_B) | BIT(PIPE_C) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_B] = BIT(DBUF_S1) | BIT(DBUF_S2),
			[PIPE_C] = BIT(DBUF_S3),
			[PIPE_D] = BIT(DBUF_S4),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_C) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1),
			[PIPE_B] = BIT(DBUF_S2),
			[PIPE_C] = BIT(DBUF_S3),
			[PIPE_D] = BIT(DBUF_S4),
		},
	},
	{}
};

static const struct dbuf_slice_conf_entry adlp_allowed_dbufs[] = {
	/*
	 * Keep the join_mbus cases first so check_mbus_joined()
	 * will prefer them over the !join_mbus cases.
	 */
	{
		.active_pipes = BIT(PIPE_A),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1) | BIT(DBUF_S2) | BIT(DBUF_S3) | BIT(DBUF_S4),
		},
		.join_mbus = true,
	},
	{
		.active_pipes = BIT(PIPE_B),
		.dbuf_mask = {
			[PIPE_B] = BIT(DBUF_S1) | BIT(DBUF_S2) | BIT(DBUF_S3) | BIT(DBUF_S4),
		},
		.join_mbus = true,
	},
	{
		.active_pipes = BIT(PIPE_A),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1) | BIT(DBUF_S2),
		},
		.join_mbus = false,
	},
	{
		.active_pipes = BIT(PIPE_B),
		.dbuf_mask = {
			[PIPE_B] = BIT(DBUF_S3) | BIT(DBUF_S4),
		},
		.join_mbus = false,
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_B),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1) | BIT(DBUF_S2),
			[PIPE_B] = BIT(DBUF_S3) | BIT(DBUF_S4),
		},
	},
	{
		.active_pipes = BIT(PIPE_C),
		.dbuf_mask = {
			[PIPE_C] = BIT(DBUF_S3) | BIT(DBUF_S4),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_C),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1) | BIT(DBUF_S2),
			[PIPE_C] = BIT(DBUF_S3) | BIT(DBUF_S4),
		},
	},
	{
		.active_pipes = BIT(PIPE_B) | BIT(PIPE_C),
		.dbuf_mask = {
			[PIPE_B] = BIT(DBUF_S3) | BIT(DBUF_S4),
			[PIPE_C] = BIT(DBUF_S3) | BIT(DBUF_S4),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_C),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1) | BIT(DBUF_S2),
			[PIPE_B] = BIT(DBUF_S3) | BIT(DBUF_S4),
			[PIPE_C] = BIT(DBUF_S3) | BIT(DBUF_S4),
		},
	},
	{
		.active_pipes = BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_D] = BIT(DBUF_S1) | BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1) | BIT(DBUF_S2),
			[PIPE_D] = BIT(DBUF_S1) | BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_B) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_B] = BIT(DBUF_S3) | BIT(DBUF_S4),
			[PIPE_D] = BIT(DBUF_S1) | BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1) | BIT(DBUF_S2),
			[PIPE_B] = BIT(DBUF_S3) | BIT(DBUF_S4),
			[PIPE_D] = BIT(DBUF_S1) | BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_C) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_C] = BIT(DBUF_S3) | BIT(DBUF_S4),
			[PIPE_D] = BIT(DBUF_S1) | BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_C) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1) | BIT(DBUF_S2),
			[PIPE_C] = BIT(DBUF_S3) | BIT(DBUF_S4),
			[PIPE_D] = BIT(DBUF_S1) | BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_B) | BIT(PIPE_C) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_B] = BIT(DBUF_S3) | BIT(DBUF_S4),
			[PIPE_C] = BIT(DBUF_S3) | BIT(DBUF_S4),
			[PIPE_D] = BIT(DBUF_S1) | BIT(DBUF_S2),
		},
	},
	{
		.active_pipes = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_C) | BIT(PIPE_D),
		.dbuf_mask = {
			[PIPE_A] = BIT(DBUF_S1) | BIT(DBUF_S2),
			[PIPE_B] = BIT(DBUF_S3) | BIT(DBUF_S4),
			[PIPE_C] = BIT(DBUF_S3) | BIT(DBUF_S4),
			[PIPE_D] = BIT(DBUF_S1) | BIT(DBUF_S2),
		},
	},
	{}

};

static bool check_mbus_joined(u8 active_pipes,
			      const struct dbuf_slice_conf_entry *dbuf_slices)
{
	int i;

	for (i = 0; dbuf_slices[i].active_pipes != 0; i++) {
		if (dbuf_slices[i].active_pipes == active_pipes)
			return dbuf_slices[i].join_mbus;
	}
	return false;
}

static bool adlp_check_mbus_joined(u8 active_pipes)
{
	return check_mbus_joined(active_pipes, adlp_allowed_dbufs);
}

static u8 compute_dbuf_slices(enum pipe pipe, u8 active_pipes, bool join_mbus,
			      const struct dbuf_slice_conf_entry *dbuf_slices)
{
	int i;

	for (i = 0; dbuf_slices[i].active_pipes != 0; i++) {
		if (dbuf_slices[i].active_pipes == active_pipes &&
		    dbuf_slices[i].join_mbus == join_mbus)
			return dbuf_slices[i].dbuf_mask[pipe];
	}
	return 0;
}

/*
 * This function finds an entry with same enabled pipe configuration and
 * returns correspondent DBuf slice mask as stated in BSpec for particular
 * platform.
 */
static u8 icl_compute_dbuf_slices(enum pipe pipe, u8 active_pipes, bool join_mbus)
{
	/*
	 * FIXME: For ICL this is still a bit unclear as prev BSpec revision
	 * required calculating "pipe ratio" in order to determine
	 * if one or two slices can be used for single pipe configurations
	 * as additional constraint to the existing table.
	 * However based on recent info, it should be not "pipe ratio"
	 * but rather ratio between pixel_rate and cdclk with additional
	 * constants, so for now we are using only table until this is
	 * clarified. Also this is the reason why crtc_state param is
	 * still here - we will need it once those additional constraints
	 * pop up.
	 */
	return compute_dbuf_slices(pipe, active_pipes, join_mbus,
				   icl_allowed_dbufs);
}

static u8 tgl_compute_dbuf_slices(enum pipe pipe, u8 active_pipes, bool join_mbus)
{
	return compute_dbuf_slices(pipe, active_pipes, join_mbus,
				   tgl_allowed_dbufs);
}

static u8 adlp_compute_dbuf_slices(enum pipe pipe, u8 active_pipes, bool join_mbus)
{
	return compute_dbuf_slices(pipe, active_pipes, join_mbus,
				   adlp_allowed_dbufs);
}

static u8 dg2_compute_dbuf_slices(enum pipe pipe, u8 active_pipes, bool join_mbus)
{
	return compute_dbuf_slices(pipe, active_pipes, join_mbus,
				   dg2_allowed_dbufs);
}

static u8 skl_compute_dbuf_slices(struct intel_crtc *crtc, u8 active_pipes, bool join_mbus)
{
	struct intel_display *display = to_intel_display(crtc);
	enum pipe pipe = crtc->pipe;

	if (display->platform.dg2)
		return dg2_compute_dbuf_slices(pipe, active_pipes, join_mbus);
	else if (DISPLAY_VER(display) >= 13)
		return adlp_compute_dbuf_slices(pipe, active_pipes, join_mbus);
	else if (DISPLAY_VER(display) == 12)
		return tgl_compute_dbuf_slices(pipe, active_pipes, join_mbus);
	else if (DISPLAY_VER(display) == 11)
		return icl_compute_dbuf_slices(pipe, active_pipes, join_mbus);
	/*
	 * For anything else just return one slice yet.
	 * Should be extended for other platforms.
	 */
	return active_pipes & BIT(pipe) ? BIT(DBUF_S1) : 0;
}

static bool
use_minimal_wm0_only(const struct intel_crtc_state *crtc_state,
		     struct intel_plane *plane)
{
	struct intel_display *display = to_intel_display(plane);

	/* Xe3+ are auto minimum DDB capble. So don't force minimal wm0 */
	return IS_DISPLAY_VER(display, 13, 20) &&
	       crtc_state->uapi.async_flip &&
	       plane->async_flip;
}

unsigned int
skl_plane_relative_data_rate(const struct intel_crtc_state *crtc_state,
			     struct intel_plane *plane, int width, int height,
			     int cpp)
{
	/*
	 * We calculate extra ddb based on ratio plane rate/total data rate
	 * in case, in some cases we should not allocate extra ddb for the plane,
	 * so do not count its data rate, if this is the case.
	 */
	if (use_minimal_wm0_only(crtc_state, plane))
		return 0;

	return width * height * cpp;
}

static u64
skl_total_relative_data_rate(const struct intel_crtc_state *crtc_state)
{
	struct intel_display *display = to_intel_display(crtc_state);
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	enum plane_id plane_id;
	u64 data_rate = 0;

	for_each_plane_id_on_crtc(crtc, plane_id) {
		if (plane_id == PLANE_CURSOR)
			continue;

		data_rate += crtc_state->rel_data_rate[plane_id];

		if (DISPLAY_VER(display) < 11)
			data_rate += crtc_state->rel_data_rate_y[plane_id];
	}

	return data_rate;
}

const struct skl_wm_level *
skl_plane_wm_level(const struct skl_pipe_wm *pipe_wm,
		   enum plane_id plane_id,
		   int level)
{
	const struct skl_plane_wm *wm = &pipe_wm->planes[plane_id];

	if (level == 0 && pipe_wm->use_sagv_wm)
		return &wm->sagv.wm0;

	return &wm->wm[level];
}

const struct skl_wm_level *
skl_plane_trans_wm(const struct skl_pipe_wm *pipe_wm,
		   enum plane_id plane_id)
{
	const struct skl_plane_wm *wm = &pipe_wm->planes[plane_id];

	if (pipe_wm->use_sagv_wm)
		return &wm->sagv.trans_wm;

	return &wm->trans_wm;
}

/*
 * We only disable the watermarks for each plane if
 * they exceed the ddb allocation of said plane. This
 * is done so that we don't end up touching cursor
 * watermarks needlessly when some other plane reduces
 * our max possible watermark level.
 *
 * Bspec has this to say about the PLANE_WM enable bit:
 * "All the watermarks at this level for all enabled
 *  planes must be enabled before the level will be used."
 * So this is actually safe to do.
 */
static void
skl_check_wm_level(struct skl_wm_level *wm, const struct skl_ddb_entry *ddb)
{
	if (wm->min_ddb_alloc > skl_ddb_entry_size(ddb))
		memset(wm, 0, sizeof(*wm));
}

static void
skl_check_nv12_wm_level(struct skl_wm_level *wm, struct skl_wm_level *uv_wm,
			const struct skl_ddb_entry *ddb_y, const struct skl_ddb_entry *ddb)
{
	if (wm->min_ddb_alloc > skl_ddb_entry_size(ddb_y) ||
	    uv_wm->min_ddb_alloc > skl_ddb_entry_size(ddb)) {
		memset(wm, 0, sizeof(*wm));
		memset(uv_wm, 0, sizeof(*uv_wm));
	}
}

static bool skl_need_wm_copy_wa(struct intel_display *display, int level,
				const struct skl_plane_wm *wm)
{
	/*
	 * Wa_1408961008:icl, ehl
	 * Wa_14012656716:tgl, adl
	 * Wa_14017887344:icl
	 * Wa_14017868169:adl, tgl
	 * Due to some power saving optimizations, different subsystems
	 * like PSR, might still use even disabled wm level registers,
	 * for "reference", so lets keep at least the values sane.
	 * Considering amount of WA requiring us to do similar things, was
	 * decided to simply do it for all of the platforms, as those wm
	 * levels are disabled, this isn't going to do harm anyway.
	 */
	return level > 0 && !wm->wm[level].enable;
}

struct skl_plane_ddb_iter {
	u64 data_rate;
	u16 start, size;
};

static void
skl_allocate_plane_ddb(struct skl_plane_ddb_iter *iter,
		       struct skl_ddb_entry *ddb,
		       const struct skl_wm_level *wm,
		       u64 data_rate)
{
	u16 size, extra = 0;

	if (data_rate) {
		extra = min_t(u16, iter->size,
			      DIV64_U64_ROUND_UP(iter->size * data_rate,
						 iter->data_rate));
		iter->size -= extra;
		iter->data_rate -= data_rate;
	}

	/*
	 * Keep ddb entry of all disabled planes explicitly zeroed
	 * to avoid skl_ddb_add_affected_planes() adding them to
	 * the state when other planes change their allocations.
	 */
	size = wm->min_ddb_alloc + extra;
	if (size)
		iter->start = skl_ddb_entry_init(ddb, iter->start,
						 iter->start + size);
}

static int
skl_crtc_allocate_plane_ddb(struct intel_atomic_state *state,
			    struct intel_crtc *crtc)
{
	struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	const struct intel_dbuf_state *dbuf_state =
		intel_atomic_get_new_dbuf_state(state);
	const struct skl_ddb_entry *alloc = &dbuf_state->ddb[crtc->pipe];
	struct intel_display *display = to_intel_display(state);
	int num_active = hweight8(dbuf_state->active_pipes);
	struct skl_plane_ddb_iter iter;
	enum plane_id plane_id;
	u16 cursor_size;
	u32 blocks;
	int level;

	/* Clear the partitioning for disabled planes. */
	memset(crtc_state->wm.skl.plane_ddb, 0, sizeof(crtc_state->wm.skl.plane_ddb));
	memset(crtc_state->wm.skl.plane_ddb_y, 0, sizeof(crtc_state->wm.skl.plane_ddb_y));
	memset(crtc_state->wm.skl.plane_min_ddb, 0,
	       sizeof(crtc_state->wm.skl.plane_min_ddb));
	memset(crtc_state->wm.skl.plane_interim_ddb, 0,
	       sizeof(crtc_state->wm.skl.plane_interim_ddb));

	if (!crtc_state->hw.active)
		return 0;

	iter.start = alloc->start;
	iter.size = skl_ddb_entry_size(alloc);
	if (iter.size == 0)
		return 0;

	/* Allocate fixed number of blocks for cursor. */
	cursor_size = skl_cursor_allocation(crtc_state, num_active);
	iter.size -= cursor_size;
	skl_ddb_entry_init(&crtc_state->wm.skl.plane_ddb[PLANE_CURSOR],
			   alloc->end - cursor_size, alloc->end);

	iter.data_rate = skl_total_relative_data_rate(crtc_state);

	/*
	 * Find the highest watermark level for which we can satisfy the block
	 * requirement of active planes.
	 */
	for (level = display->wm.num_levels - 1; level >= 0; level--) {
		blocks = 0;
		for_each_plane_id_on_crtc(crtc, plane_id) {
			const struct skl_plane_wm *wm =
				&crtc_state->wm.skl.optimal.planes[plane_id];

			if (plane_id == PLANE_CURSOR) {
				const struct skl_ddb_entry *ddb =
					&crtc_state->wm.skl.plane_ddb[plane_id];

				if (wm->wm[level].min_ddb_alloc > skl_ddb_entry_size(ddb)) {
					drm_WARN_ON(display->drm,
						    wm->wm[level].min_ddb_alloc != U16_MAX);
					blocks = U32_MAX;
					break;
				}
				continue;
			}

			blocks += wm->wm[level].min_ddb_alloc;
			blocks += wm->uv_wm[level].min_ddb_alloc;
		}

		if (blocks <= iter.size) {
			iter.size -= blocks;
			break;
		}
	}

	if (level < 0) {
		drm_dbg_kms(display->drm,
			    "Requested display configuration exceeds system DDB limitations");
		drm_dbg_kms(display->drm, "minimum required %d/%d\n",
			    blocks, iter.size);
		return -EINVAL;
	}

	/* avoid the WARN later when we don't allocate any extra DDB */
	if (iter.data_rate == 0)
		iter.size = 0;

	/*
	 * Grant each plane the blocks it requires at the highest achievable
	 * watermark level, plus an extra share of the leftover blocks
	 * proportional to its relative data rate.
	 */
	for_each_plane_id_on_crtc(crtc, plane_id) {
		struct skl_ddb_entry *ddb =
			&crtc_state->wm.skl.plane_ddb[plane_id];
		struct skl_ddb_entry *ddb_y =
			&crtc_state->wm.skl.plane_ddb_y[plane_id];
		u16 *min_ddb = &crtc_state->wm.skl.plane_min_ddb[plane_id];
		u16 *interim_ddb =
			&crtc_state->wm.skl.plane_interim_ddb[plane_id];
		const struct skl_plane_wm *wm =
			&crtc_state->wm.skl.optimal.planes[plane_id];

		if (plane_id == PLANE_CURSOR)
			continue;

		if (DISPLAY_VER(display) < 11 &&
		    crtc_state->nv12_planes & BIT(plane_id)) {
			skl_allocate_plane_ddb(&iter, ddb_y, &wm->wm[level],
					       crtc_state->rel_data_rate_y[plane_id]);
			skl_allocate_plane_ddb(&iter, ddb, &wm->uv_wm[level],
					       crtc_state->rel_data_rate[plane_id]);
		} else {
			skl_allocate_plane_ddb(&iter, ddb, &wm->wm[level],
					       crtc_state->rel_data_rate[plane_id]);
		}

		if (DISPLAY_VER(display) >= 30) {
			*min_ddb = wm->wm[0].min_ddb_alloc;
			*interim_ddb = wm->sagv.wm0.min_ddb_alloc;
		}
	}
	drm_WARN_ON(display->drm, iter.size != 0 || iter.data_rate != 0);

	/*
	 * When we calculated watermark values we didn't know how high
	 * of a level we'd actually be able to hit, so we just marked
	 * all levels as "enabled."  Go back now and disable the ones
	 * that aren't actually possible.
	 */
	for (level++; level < display->wm.num_levels; level++) {
		for_each_plane_id_on_crtc(crtc, plane_id) {
			const struct skl_ddb_entry *ddb =
				&crtc_state->wm.skl.plane_ddb[plane_id];
			const struct skl_ddb_entry *ddb_y =
				&crtc_state->wm.skl.plane_ddb_y[plane_id];
			struct skl_plane_wm *wm =
				&crtc_state->wm.skl.optimal.planes[plane_id];

			if (DISPLAY_VER(display) < 11 &&
			    crtc_state->nv12_planes & BIT(plane_id))
				skl_check_nv12_wm_level(&wm->wm[level],
							&wm->uv_wm[level],
							ddb_y, ddb);
			else
				skl_check_wm_level(&wm->wm[level], ddb);

			if (skl_need_wm_copy_wa(display, level, wm)) {
				wm->wm[level].blocks = wm->wm[level - 1].blocks;
				wm->wm[level].lines = wm->wm[level - 1].lines;
				wm->wm[level].ignore_lines = wm->wm[level - 1].ignore_lines;
			}
		}
	}

	/*
	 * Go back and disable the transition and SAGV watermarks
	 * if it turns out we don't have enough DDB blocks for them.
	 */
	for_each_plane_id_on_crtc(crtc, plane_id) {
		const struct skl_ddb_entry *ddb =
			&crtc_state->wm.skl.plane_ddb[plane_id];
		const struct skl_ddb_entry *ddb_y =
			&crtc_state->wm.skl.plane_ddb_y[plane_id];
		u16 *interim_ddb =
			&crtc_state->wm.skl.plane_interim_ddb[plane_id];
		struct skl_plane_wm *wm =
			&crtc_state->wm.skl.optimal.planes[plane_id];

		if (DISPLAY_VER(display) < 11 &&
		    crtc_state->nv12_planes & BIT(plane_id)) {
			skl_check_wm_level(&wm->trans_wm, ddb_y);
		} else {
			WARN_ON(skl_ddb_entry_size(ddb_y));

			skl_check_wm_level(&wm->trans_wm, ddb);
		}

		skl_check_wm_level(&wm->sagv.wm0, ddb);
		if (DISPLAY_VER(display) >= 30)
			*interim_ddb = wm->sagv.wm0.min_ddb_alloc;

		skl_check_wm_level(&wm->sagv.trans_wm, ddb);
	}

	return 0;
}

/*
 * The max latency should be 257 (max the punit can code is 255 and we add 2us
 * for the read latency) and cpp should always be <= 8, so that
 * should allow pixel_rate up to ~2 GHz which seems sufficient since max
 * 2xcdclk is 1350 MHz and the pixel rate should never exceed that.
 */
static uint_fixed_16_16_t
skl_wm_method1(struct intel_display *display, u32 pixel_rate,
	       u8 cpp, u32 latency, u32 dbuf_block_size)
{
	u32 wm_intermediate_val;
	uint_fixed_16_16_t ret;

	if (latency == 0)
		return FP_16_16_MAX;

	wm_intermediate_val = latency * pixel_rate * cpp;
	ret = div_fixed16(wm_intermediate_val, 1000 * dbuf_block_size);

	if (DISPLAY_VER(display) >= 10)
		ret = add_fixed16_u32(ret, 1);

	return ret;
}

static uint_fixed_16_16_t
skl_wm_method2(u32 pixel_rate, u32 pipe_htotal, u32 latency,
	       uint_fixed_16_16_t plane_blocks_per_line)
{
	u32 wm_intermediate_val;
	uint_fixed_16_16_t ret;

	if (latency == 0)
		return FP_16_16_MAX;

	wm_intermediate_val = latency * pixel_rate;
	wm_intermediate_val = DIV_ROUND_UP(wm_intermediate_val,
					   pipe_htotal * 1000);
	ret = mul_u32_fixed16(wm_intermediate_val, plane_blocks_per_line);
	return ret;
}

static uint_fixed_16_16_t
intel_get_linetime_us(const struct intel_crtc_state *crtc_state)
{
	struct intel_display *display = to_intel_display(crtc_state);
	u32 pixel_rate;
	u32 crtc_htotal;
	uint_fixed_16_16_t linetime_us;

	if (!crtc_state->hw.active)
		return u32_to_fixed16(0);

	pixel_rate = crtc_state->pixel_rate;

	if (drm_WARN_ON(display->drm, pixel_rate == 0))
		return u32_to_fixed16(0);

	crtc_htotal = crtc_state->hw.pipe_mode.crtc_htotal;
	linetime_us = div_fixed16(crtc_htotal * 1000, pixel_rate);

	return linetime_us;
}

static int
skl_compute_wm_params(const struct intel_crtc_state *crtc_state,
		      int width, const struct drm_format_info *format,
		      u64 modifier, unsigned int rotation,
		      u32 plane_pixel_rate, struct skl_wm_params *wp,
		      int color_plane, unsigned int pan_x)
{
	struct intel_display *display = to_intel_display(crtc_state);
	u32 interm_pbpl;

	/* only planar format has two planes */
	if (color_plane == 1 &&
	    !intel_format_info_is_yuv_semiplanar(format, modifier)) {
		drm_dbg_kms(display->drm,
			    "Non planar format have single plane\n");
		return -EINVAL;
	}

	wp->x_tiled = modifier == I915_FORMAT_MOD_X_TILED;
	wp->y_tiled = modifier != I915_FORMAT_MOD_X_TILED &&
		intel_fb_is_tiled_modifier(modifier);
	wp->rc_surface = intel_fb_is_ccs_modifier(modifier);
	wp->is_planar = intel_format_info_is_yuv_semiplanar(format, modifier);

	wp->width = width;
	if (color_plane == 1 && wp->is_planar)
		wp->width /= 2;

	wp->cpp = format->cpp[color_plane];
	wp->plane_pixel_rate = plane_pixel_rate;

	if (DISPLAY_VER(display) >= 11 &&
	    modifier == I915_FORMAT_MOD_Yf_TILED  && wp->cpp == 1)
		wp->dbuf_block_size = 256;
	else
		wp->dbuf_block_size = 512;

	if (drm_rotation_90_or_270(rotation)) {
		switch (wp->cpp) {
		case 1:
			wp->y_min_scanlines = 16;
			break;
		case 2:
			wp->y_min_scanlines = 8;
			break;
		case 4:
			wp->y_min_scanlines = 4;
			break;
		default:
			MISSING_CASE(wp->cpp);
			return -EINVAL;
		}
	} else {
		wp->y_min_scanlines = 4;
	}

	if (skl_needs_memory_bw_wa(display))
		wp->y_min_scanlines *= 2;

	wp->plane_bytes_per_line = wp->width * wp->cpp;
	if (wp->y_tiled) {
		interm_pbpl = DIV_ROUND_UP(wp->plane_bytes_per_line *
					   wp->y_min_scanlines,
					   wp->dbuf_block_size);

		if (DISPLAY_VER(display) >= 30)
			interm_pbpl += (pan_x != 0);
		else if (DISPLAY_VER(display) >= 10)
			interm_pbpl++;

		wp->plane_blocks_per_line = div_fixed16(interm_pbpl,
							wp->y_min_scanlines);
	} else {
		interm_pbpl = DIV_ROUND_UP(wp->plane_bytes_per_line,
					   wp->dbuf_block_size);

		if (!wp->x_tiled || DISPLAY_VER(display) >= 10)
			interm_pbpl++;

		wp->plane_blocks_per_line = u32_to_fixed16(interm_pbpl);
	}

	wp->y_tile_minimum = mul_u32_fixed16(wp->y_min_scanlines,
					     wp->plane_blocks_per_line);

	wp->linetime_us = fixed16_to_u32_round_up(intel_get_linetime_us(crtc_state));

	return 0;
}

static int
skl_compute_plane_wm_params(const struct intel_crtc_state *crtc_state,
			    const struct intel_plane_state *plane_state,
			    struct skl_wm_params *wp, int color_plane)
{
	const struct drm_framebuffer *fb = plane_state->hw.fb;
	int width;

	/*
	 * Src coordinates are already rotated by 270 degrees for
	 * the 90/270 degree plane rotation cases (to match the
	 * GTT mapping), hence no need to account for rotation here.
	 */
	width = drm_rect_width(&plane_state->uapi.src) >> 16;

	return skl_compute_wm_params(crtc_state, width,
				     fb->format, fb->modifier,
				     plane_state->hw.rotation,
				     intel_plane_pixel_rate(crtc_state, plane_state),
				     wp, color_plane,
				     plane_state->uapi.src.x1);
}

static bool skl_wm_has_lines(struct intel_display *display, int level)
{
	if (DISPLAY_VER(display) >= 10)
		return true;

	/* The number of lines are ignored for the level 0 watermark. */
	return level > 0;
}

static int skl_wm_max_lines(struct intel_display *display)
{
	if (DISPLAY_VER(display) >= 13)
		return 255;
	else
		return 31;
}

static bool xe3_auto_min_alloc_capable(struct intel_plane *plane, int level)
{
	struct intel_display *display = to_intel_display(plane);

	return DISPLAY_VER(display) >= 30 && level == 0 && plane->id != PLANE_CURSOR;
}

static void skl_compute_plane_wm(const struct intel_crtc_state *crtc_state,
				 struct intel_plane *plane,
				 int level,
				 unsigned int latency,
				 const struct skl_wm_params *wp,
				 const struct skl_wm_level *result_prev,
				 struct skl_wm_level *result /* out */)
{
	struct intel_display *display = to_intel_display(crtc_state);
	uint_fixed_16_16_t method1, method2;
	uint_fixed_16_16_t selected_result;
	u32 blocks, lines, min_ddb_alloc = 0;

	if (latency == 0 ||
	    (use_minimal_wm0_only(crtc_state, plane) && level > 0)) {
		/* reject it */
		result->min_ddb_alloc = U16_MAX;
		return;
	}

	method1 = skl_wm_method1(display, wp->plane_pixel_rate,
				 wp->cpp, latency, wp->dbuf_block_size);
	method2 = skl_wm_method2(wp->plane_pixel_rate,
				 crtc_state->hw.pipe_mode.crtc_htotal,
				 latency,
				 wp->plane_blocks_per_line);

	if (wp->y_tiled) {
		selected_result = max_fixed16(method2, wp->y_tile_minimum);
	} else {
		if ((wp->cpp * crtc_state->hw.pipe_mode.crtc_htotal /
		     wp->dbuf_block_size < 1) &&
		     (wp->plane_bytes_per_line / wp->dbuf_block_size < 1)) {
			selected_result = method2;
		} else if (latency >= wp->linetime_us) {
			if (DISPLAY_VER(display) == 9)
				selected_result = min_fixed16(method1, method2);
			else
				selected_result = method2;
		} else {
			selected_result = method1;
		}
	}

	blocks = fixed16_to_u32_round_up(selected_result);
	if (DISPLAY_VER(display) < 30)
		blocks++;

	/*
	 * Lets have blocks at minimum equivalent to plane_blocks_per_line
	 * as there will be at minimum one line for lines configuration. This
	 * is a work around for FIFO underruns observed with resolutions like
	 * 4k 60 Hz in single channel DRAM configurations.
	 *
	 * As per the Bspec 49325, if the ddb allocation can hold at least
	 * one plane_blocks_per_line, we should have selected method2 in
	 * the above logic. Assuming that modern versions have enough dbuf
	 * and method2 guarantees blocks equivalent to at least 1 line,
	 * select the blocks as plane_blocks_per_line.
	 *
	 * TODO: Revisit the logic when we have better understanding on DRAM
	 * channels' impact on the level 0 memory latency and the relevant
	 * wm calculations.
	 */
	if (skl_wm_has_lines(display, level))
		blocks = max(blocks,
			     fixed16_to_u32_round_up(wp->plane_blocks_per_line));
	lines = div_round_up_fixed16(selected_result,
				     wp->plane_blocks_per_line);

	if (DISPLAY_VER(display) == 9) {
		/* Display WA #1125: skl,bxt,kbl */
		if (level == 0 && wp->rc_surface)
			blocks += fixed16_to_u32_round_up(wp->y_tile_minimum);

		/* Display WA #1126: skl,bxt,kbl */
		if (level >= 1 && level <= 7) {
			if (wp->y_tiled) {
				blocks += fixed16_to_u32_round_up(wp->y_tile_minimum);
				lines += wp->y_min_scanlines;
			} else {
				blocks++;
			}

			/*
			 * Make sure result blocks for higher latency levels are
			 * at least as high as level below the current level.
			 * Assumption in DDB algorithm optimization for special
			 * cases. Also covers Display WA #1125 for RC.
			 */
			if (result_prev->blocks > blocks)
				blocks = result_prev->blocks;
		}
	}

	if (DISPLAY_VER(display) >= 11) {
		if (wp->y_tiled) {
			int extra_lines;

			if (lines % wp->y_min_scanlines == 0)
				extra_lines = wp->y_min_scanlines;
			else
				extra_lines = wp->y_min_scanlines * 2 -
					lines % wp->y_min_scanlines;

			min_ddb_alloc = mul_round_up_u32_fixed16(lines + extra_lines,
								 wp->plane_blocks_per_line);
		} else {
			min_ddb_alloc = blocks + DIV_ROUND_UP(blocks, 10);
		}
	}

	if (!skl_wm_has_lines(display, level))
		lines = 0;

	if (lines > skl_wm_max_lines(display)) {
		/* reject it */
		result->min_ddb_alloc = U16_MAX;
		return;
	}

	/*
	 * If lines is valid, assume we can use this watermark level
	 * for now.  We'll come back and disable it after we calculate the
	 * DDB allocation if it turns out we don't actually have enough
	 * blocks to satisfy it.
	 */
	result->blocks = blocks;
	result->lines = lines;
	/* Bspec says: value >= plane ddb allocation -> invalid, hence the +1 here */
	result->min_ddb_alloc = max(min_ddb_alloc, blocks) + 1;
	result->enable = true;
	result->auto_min_alloc_wm_enable = xe3_auto_min_alloc_capable(plane, level);

	if (DISPLAY_VER(display) < 12 && display->sagv.block_time_us)
		result->can_sagv = latency >= display->sagv.block_time_us;
}

static void
skl_compute_wm_levels(const struct intel_crtc_state *crtc_state,
		      struct intel_plane *plane,
		      const struct skl_wm_params *wm_params,
		      struct skl_wm_level *levels)
{
	struct intel_display *display = to_intel_display(crtc_state);
	struct skl_wm_level *result_prev = &levels[0];
	int level;

	for (level = 0; level < display->wm.num_levels; level++) {
		struct skl_wm_level *result = &levels[level];
		unsigned int latency = skl_wm_latency(display, level, wm_params);

		skl_compute_plane_wm(crtc_state, plane, level, latency,
				     wm_params, result_prev, result);

		result_prev = result;
	}
}

static void tgl_compute_sagv_wm(const struct intel_crtc_state *crtc_state,
				struct intel_plane *plane,
				const struct skl_wm_params *wm_params,
				struct skl_plane_wm *plane_wm)
{
	struct intel_display *display = to_intel_display(crtc_state);
	struct skl_wm_level *sagv_wm = &plane_wm->sagv.wm0;
	struct skl_wm_level *levels = plane_wm->wm;
	unsigned int latency = 0;

	if (display->sagv.block_time_us)
		latency = display->sagv.block_time_us +
			skl_wm_latency(display, 0, wm_params);

	skl_compute_plane_wm(crtc_state, plane, 0, latency,
			     wm_params, &levels[0],
			     sagv_wm);
}

static void skl_compute_transition_wm(struct intel_display *display,
				      struct skl_wm_level *trans_wm,
				      const struct skl_wm_level *wm0,
				      const struct skl_wm_params *wp)
{
	u16 trans_min, trans_amount, trans_y_tile_min;
	u16 wm0_blocks, trans_offset, blocks;

	/* Transition WM don't make any sense if ipc is disabled */
	if (!skl_watermark_ipc_enabled(display))
		return;

	/*
	 * WaDisableTWM:skl,kbl,cfl,bxt
	 * Transition WM are not recommended by HW team for GEN9
	 */
	if (DISPLAY_VER(display) == 9)
		return;

	if (DISPLAY_VER(display) >= 11)
		trans_min = 4;
	else
		trans_min = 14;

	/* Display WA #1140: glk,cnl */
	if (DISPLAY_VER(display) == 10)
		trans_amount = 0;
	else
		trans_amount = 10; /* This is configurable amount */

	trans_offset = trans_min + trans_amount;

	/*
	 * The spec asks for Selected Result Blocks for wm0 (the real value),
	 * not Result Blocks (the integer value). Pay attention to the capital
	 * letters. The value wm_l0->blocks is actually Result Blocks, but
	 * since Result Blocks is the ceiling of Selected Result Blocks plus 1,
	 * and since we later will have to get the ceiling of the sum in the
	 * transition watermarks calculation, we can just pretend Selected
	 * Result Blocks is Result Blocks minus 1 and it should work for the
	 * current platforms.
	 */
	wm0_blocks = wm0->blocks - 1;

	if (wp->y_tiled) {
		trans_y_tile_min =
			(u16)mul_round_up_u32_fixed16(2, wp->y_tile_minimum);
		blocks = max(wm0_blocks, trans_y_tile_min) + trans_offset;
	} else {
		blocks = wm0_blocks + trans_offset;
	}
	blocks++;

	/*
	 * Just assume we can enable the transition watermark.  After
	 * computing the DDB we'll come back and disable it if that
	 * assumption turns out to be false.
	 */
	trans_wm->blocks = blocks;
	trans_wm->min_ddb_alloc = max_t(u16, wm0->min_ddb_alloc, blocks + 1);
	trans_wm->enable = true;
}

static int skl_build_plane_wm_single(struct intel_crtc_state *crtc_state,
				     const struct intel_plane_state *plane_state,
				     struct intel_plane *plane, int color_plane)
{
	struct intel_display *display = to_intel_display(crtc_state);
	struct skl_plane_wm *wm = &crtc_state->wm.skl.raw.planes[plane->id];
	struct skl_wm_params wm_params;
	int ret;

	ret = skl_compute_plane_wm_params(crtc_state, plane_state,
					  &wm_params, color_plane);
	if (ret)
		return ret;

	skl_compute_wm_levels(crtc_state, plane, &wm_params, wm->wm);

	skl_compute_transition_wm(display, &wm->trans_wm,
				  &wm->wm[0], &wm_params);

	if (DISPLAY_VER(display) >= 12) {
		tgl_compute_sagv_wm(crtc_state, plane, &wm_params, wm);

		skl_compute_transition_wm(display, &wm->sagv.trans_wm,
					  &wm->sagv.wm0, &wm_params);
	}

	return 0;
}

static int skl_build_plane_wm_uv(struct intel_crtc_state *crtc_state,
				 const struct intel_plane_state *plane_state,
				 struct intel_plane *plane)
{
	struct skl_plane_wm *wm = &crtc_state->wm.skl.raw.planes[plane->id];
	struct skl_wm_params wm_params;
	int ret;

	wm->is_planar = true;

	/* uv plane watermarks must also be validated for NV12/Planar */
	ret = skl_compute_plane_wm_params(crtc_state, plane_state,
					  &wm_params, 1);
	if (ret)
		return ret;

	skl_compute_wm_levels(crtc_state, plane, &wm_params, wm->uv_wm);

	return 0;
}

static int skl_build_plane_wm(struct intel_crtc_state *crtc_state,
			      const struct intel_plane_state *plane_state)
{
	struct intel_plane *plane = to_intel_plane(plane_state->uapi.plane);
	enum plane_id plane_id = plane->id;
	struct skl_plane_wm *wm = &crtc_state->wm.skl.raw.planes[plane_id];
	const struct drm_framebuffer *fb = plane_state->hw.fb;
	int ret;

	memset(wm, 0, sizeof(*wm));

	if (!intel_wm_plane_visible(crtc_state, plane_state))
		return 0;

	ret = skl_build_plane_wm_single(crtc_state, plane_state,
					plane, 0);
	if (ret)
		return ret;

	if (fb->format->is_yuv && fb->format->num_planes > 1) {
		ret = skl_build_plane_wm_uv(crtc_state, plane_state,
					    plane);
		if (ret)
			return ret;
	}

	return 0;
}

static int icl_build_plane_wm(struct intel_crtc_state *crtc_state,
			      const struct intel_plane_state *plane_state)
{
	struct intel_display *display = to_intel_display(plane_state);
	struct intel_plane *plane = to_intel_plane(plane_state->uapi.plane);
	enum plane_id plane_id = plane->id;
	struct skl_plane_wm *wm = &crtc_state->wm.skl.raw.planes[plane_id];
	int ret;

	/* Watermarks calculated on UV plane */
	if (plane_state->is_y_plane)
		return 0;

	memset(wm, 0, sizeof(*wm));

	if (plane_state->planar_linked_plane) {
		const struct drm_framebuffer *fb = plane_state->hw.fb;

		drm_WARN_ON(display->drm,
			    !intel_wm_plane_visible(crtc_state, plane_state));
		drm_WARN_ON(display->drm, !fb->format->is_yuv ||
			    fb->format->num_planes == 1);

		ret = skl_build_plane_wm_single(crtc_state, plane_state,
						plane_state->planar_linked_plane, 0);
		if (ret)
			return ret;

		ret = skl_build_plane_wm_single(crtc_state, plane_state,
						plane, 1);
		if (ret)
			return ret;
	} else if (intel_wm_plane_visible(crtc_state, plane_state)) {
		ret = skl_build_plane_wm_single(crtc_state, plane_state,
						plane, 0);
		if (ret)
			return ret;
	}

	return 0;
}

static int
cdclk_prefill_adjustment(const struct intel_crtc_state *crtc_state)
{
	struct intel_display *display = to_intel_display(crtc_state);
	struct intel_atomic_state *state =
		to_intel_atomic_state(crtc_state->uapi.state);
	const struct intel_cdclk_state *cdclk_state;

	cdclk_state = intel_atomic_get_cdclk_state(state);
	if (IS_ERR(cdclk_state)) {
		drm_WARN_ON(display->drm, PTR_ERR(cdclk_state));
		return 1;
	}

	return min(1, DIV_ROUND_UP(crtc_state->pixel_rate,
				   2 * intel_cdclk_logical(cdclk_state)));
}

static int
dsc_prefill_latency(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	const struct intel_crtc_scaler_state *scaler_state =
					&crtc_state->scaler_state;
	int linetime = DIV_ROUND_UP(1000 * crtc_state->hw.adjusted_mode.htotal,
				    crtc_state->hw.adjusted_mode.clock);
	int num_scaler_users = hweight32(scaler_state->scaler_users);
	int chroma_downscaling_factor =
		crtc_state->output_format == INTEL_OUTPUT_FORMAT_YCBCR420 ? 2 : 1;
	u32 dsc_prefill_latency = 0;

	if (!crtc_state->dsc.compression_enable ||
	    !num_scaler_users ||
	    num_scaler_users > crtc->num_scalers)
		return dsc_prefill_latency;

	dsc_prefill_latency = DIV_ROUND_UP(15 * linetime * chroma_downscaling_factor, 10);

	for (int i = 0; i < num_scaler_users; i++) {
		u64 hscale_k, vscale_k;

		hscale_k = max(1000, mul_u32_u32(scaler_state->scalers[i].hscale, 1000) >> 16);
		vscale_k = max(1000, mul_u32_u32(scaler_state->scalers[i].vscale, 1000) >> 16);
		dsc_prefill_latency = DIV_ROUND_UP_ULL(dsc_prefill_latency * hscale_k * vscale_k,
						       1000000);
	}

	dsc_prefill_latency *= cdclk_prefill_adjustment(crtc_state);

	return intel_usecs_to_scanlines(&crtc_state->hw.adjusted_mode, dsc_prefill_latency);
}

static int
scaler_prefill_latency(const struct intel_crtc_state *crtc_state)
{
	const struct intel_crtc_scaler_state *scaler_state =
					&crtc_state->scaler_state;
	int num_scaler_users = hweight32(scaler_state->scaler_users);
	int scaler_prefill_latency = 0;
	int linetime = DIV_ROUND_UP(1000 * crtc_state->hw.adjusted_mode.htotal,
				    crtc_state->hw.adjusted_mode.clock);

	if (!num_scaler_users)
		return scaler_prefill_latency;

	scaler_prefill_latency = 4 * linetime;

	if (num_scaler_users > 1) {
		u64 hscale_k = max(1000, mul_u32_u32(scaler_state->scalers[0].hscale, 1000) >> 16);
		u64 vscale_k = max(1000, mul_u32_u32(scaler_state->scalers[0].vscale, 1000) >> 16);
		int chroma_downscaling_factor =
			crtc_state->output_format == INTEL_OUTPUT_FORMAT_YCBCR420 ? 2 : 1;
		int latency;

		latency = DIV_ROUND_UP_ULL((4 * linetime * hscale_k * vscale_k *
					    chroma_downscaling_factor), 1000000);
		scaler_prefill_latency += latency;
	}

	scaler_prefill_latency *= cdclk_prefill_adjustment(crtc_state);

	return intel_usecs_to_scanlines(&crtc_state->hw.adjusted_mode, scaler_prefill_latency);
}

static bool
skl_is_vblank_too_short(const struct intel_crtc_state *crtc_state,
			int wm0_lines, int latency)
{
	const struct drm_display_mode *adjusted_mode =
		&crtc_state->hw.adjusted_mode;

	return crtc_state->framestart_delay +
		intel_usecs_to_scanlines(adjusted_mode, latency) +
		scaler_prefill_latency(crtc_state) +
		dsc_prefill_latency(crtc_state) +
		wm0_lines >
		adjusted_mode->crtc_vtotal - adjusted_mode->crtc_vblank_start;
}

static int skl_max_wm0_lines(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	enum plane_id plane_id;
	int wm0_lines = 0;

	for_each_plane_id_on_crtc(crtc, plane_id) {
		const struct skl_plane_wm *wm = &crtc_state->wm.skl.optimal.planes[plane_id];

		/* FIXME what about !skl_wm_has_lines() platforms? */
		wm0_lines = max_t(int, wm0_lines, wm->wm[0].lines);
	}

	return wm0_lines;
}

static int skl_max_wm_level_for_vblank(struct intel_crtc_state *crtc_state,
				       int wm0_lines)
{
	struct intel_display *display = to_intel_display(crtc_state);
	int level;

	for (level = display->wm.num_levels - 1; level >= 0; level--) {
		int latency;

		/* FIXME should we care about the latency w/a's? */
		latency = skl_wm_latency(display, level, NULL);
		if (latency == 0)
			continue;

		/* FIXME is it correct to use 0 latency for wm0 here? */
		if (level == 0)
			latency = 0;

		if (!skl_is_vblank_too_short(crtc_state, wm0_lines, latency))
			return level;
	}

	return -EINVAL;
}

static int skl_wm_check_vblank(struct intel_crtc_state *crtc_state)
{
	struct intel_display *display = to_intel_display(crtc_state);
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	int wm0_lines, level;

	if (!crtc_state->hw.active)
		return 0;

	wm0_lines = skl_max_wm0_lines(crtc_state);

	level = skl_max_wm_level_for_vblank(crtc_state, wm0_lines);
	if (level < 0)
		return level;

	/*
	 * PSR needs to toggle LATENCY_REPORTING_REMOVED_PIPE_*
	 * based on whether we're limited by the vblank duration.
	 */
	crtc_state->wm_level_disabled = level < display->wm.num_levels - 1;

	for (level++; level < display->wm.num_levels; level++) {
		enum plane_id plane_id;

		for_each_plane_id_on_crtc(crtc, plane_id) {
			struct skl_plane_wm *wm =
				&crtc_state->wm.skl.optimal.planes[plane_id];

			/*
			 * FIXME just clear enable or flag the entire
			 * thing as bad via min_ddb_alloc=U16_MAX?
			 */
			wm->wm[level].enable = false;
			wm->uv_wm[level].enable = false;
		}
	}

	if (DISPLAY_VER(display) >= 12 &&
	    display->sagv.block_time_us &&
	    skl_is_vblank_too_short(crtc_state, wm0_lines,
				    display->sagv.block_time_us)) {
		enum plane_id plane_id;

		for_each_plane_id_on_crtc(crtc, plane_id) {
			struct skl_plane_wm *wm =
				&crtc_state->wm.skl.optimal.planes[plane_id];

			wm->sagv.wm0.enable = false;
			wm->sagv.trans_wm.enable = false;
		}
	}

	return 0;
}

static int skl_build_pipe_wm(struct intel_atomic_state *state,
			     struct intel_crtc *crtc)
{
	struct intel_display *display = to_intel_display(crtc);
	struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	const struct intel_plane_state *plane_state;
	struct intel_plane *plane;
	int ret, i;

	for_each_new_intel_plane_in_state(state, plane, plane_state, i) {
		/*
		 * FIXME should perhaps check {old,new}_plane_crtc->hw.crtc
		 * instead but we don't populate that correctly for NV12 Y
		 * planes so for now hack this.
		 */
		if (plane->pipe != crtc->pipe)
			continue;

		if (DISPLAY_VER(display) >= 11)
			ret = icl_build_plane_wm(crtc_state, plane_state);
		else
			ret = skl_build_plane_wm(crtc_state, plane_state);
		if (ret)
			return ret;
	}

	crtc_state->wm.skl.optimal = crtc_state->wm.skl.raw;

	return skl_wm_check_vblank(crtc_state);
}

static bool skl_wm_level_equals(const struct skl_wm_level *l1,
				const struct skl_wm_level *l2)
{
	return l1->enable == l2->enable &&
		l1->ignore_lines == l2->ignore_lines &&
		l1->lines == l2->lines &&
		l1->blocks == l2->blocks &&
		l1->auto_min_alloc_wm_enable == l2->auto_min_alloc_wm_enable;
}

static bool skl_plane_wm_equals(struct intel_display *display,
				const struct skl_plane_wm *wm1,
				const struct skl_plane_wm *wm2)
{
	int level;

	for (level = 0; level < display->wm.num_levels; level++) {
		/*
		 * We don't check uv_wm as the hardware doesn't actually
		 * use it. It only gets used for calculating the required
		 * ddb allocation.
		 */
		if (!skl_wm_level_equals(&wm1->wm[level], &wm2->wm[level]))
			return false;
	}

	return skl_wm_level_equals(&wm1->trans_wm, &wm2->trans_wm) &&
		skl_wm_level_equals(&wm1->sagv.wm0, &wm2->sagv.wm0) &&
		skl_wm_level_equals(&wm1->sagv.trans_wm, &wm2->sagv.trans_wm);
}

static bool skl_ddb_entries_overlap(const struct skl_ddb_entry *a,
				    const struct skl_ddb_entry *b)
{
	return a->start < b->end && b->start < a->end;
}

static void skl_ddb_entry_union(struct skl_ddb_entry *a,
				const struct skl_ddb_entry *b)
{
	if (a->end && b->end) {
		a->start = min(a->start, b->start);
		a->end = max(a->end, b->end);
	} else if (b->end) {
		a->start = b->start;
		a->end = b->end;
	}
}

bool skl_ddb_allocation_overlaps(const struct skl_ddb_entry *ddb,
				 const struct skl_ddb_entry *entries,
				 int num_entries, int ignore_idx)
{
	int i;

	for (i = 0; i < num_entries; i++) {
		if (i != ignore_idx &&
		    skl_ddb_entries_overlap(ddb, &entries[i]))
			return true;
	}

	return false;
}

static int
skl_ddb_add_affected_planes(struct intel_atomic_state *state,
			    struct intel_crtc *crtc)
{
	struct intel_display *display = to_intel_display(state);
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	struct intel_plane *plane;

	for_each_intel_plane_on_crtc(display->drm, crtc, plane) {
		struct intel_plane_state *plane_state;
		enum plane_id plane_id = plane->id;

		if (skl_ddb_entry_equal(&old_crtc_state->wm.skl.plane_ddb[plane_id],
					&new_crtc_state->wm.skl.plane_ddb[plane_id]) &&
		    skl_ddb_entry_equal(&old_crtc_state->wm.skl.plane_ddb_y[plane_id],
					&new_crtc_state->wm.skl.plane_ddb_y[plane_id]))
			continue;

		if (new_crtc_state->do_async_flip) {
			drm_dbg_kms(display->drm, "[PLANE:%d:%s] Can't change DDB during async flip\n",
				    plane->base.base.id, plane->base.name);
			return -EINVAL;
		}

		plane_state = intel_atomic_get_plane_state(state, plane);
		if (IS_ERR(plane_state))
			return PTR_ERR(plane_state);

		new_crtc_state->update_planes |= BIT(plane_id);
		new_crtc_state->async_flip_planes = 0;
		new_crtc_state->do_async_flip = false;
	}

	return 0;
}

static u8 intel_dbuf_enabled_slices(const struct intel_dbuf_state *dbuf_state)
{
	struct intel_display *display = to_intel_display(dbuf_state->base.state->base.dev);
	u8 enabled_slices;
	enum pipe pipe;

	/*
	 * FIXME: For now we always enable slice S1 as per
	 * the Bspec display initialization sequence.
	 */
	enabled_slices = BIT(DBUF_S1);

	for_each_pipe(display, pipe)
		enabled_slices |= dbuf_state->slices[pipe];

	return enabled_slices;
}

static int
skl_compute_ddb(struct intel_atomic_state *state)
{
	struct intel_display *display = to_intel_display(state);
	const struct intel_dbuf_state *old_dbuf_state;
	struct intel_dbuf_state *new_dbuf_state = NULL;
	struct intel_crtc_state *new_crtc_state;
	struct intel_crtc *crtc;
	int ret, i;

	for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i) {
		new_dbuf_state = intel_atomic_get_dbuf_state(state);
		if (IS_ERR(new_dbuf_state))
			return PTR_ERR(new_dbuf_state);

		old_dbuf_state = intel_atomic_get_old_dbuf_state(state);
		break;
	}

	if (!new_dbuf_state)
		return 0;

	new_dbuf_state->active_pipes =
		intel_calc_active_pipes(state, old_dbuf_state->active_pipes);

	if (old_dbuf_state->active_pipes != new_dbuf_state->active_pipes) {
		ret = intel_atomic_lock_global_state(&new_dbuf_state->base);
		if (ret)
			return ret;
	}

	if (HAS_MBUS_JOINING(display)) {
		new_dbuf_state->joined_mbus =
			adlp_check_mbus_joined(new_dbuf_state->active_pipes);

		if (old_dbuf_state->joined_mbus != new_dbuf_state->joined_mbus) {
			ret = intel_cdclk_state_set_joined_mbus(state, new_dbuf_state->joined_mbus);
			if (ret)
				return ret;
		}
	}

	for_each_intel_crtc(display->drm, crtc) {
		enum pipe pipe = crtc->pipe;

		new_dbuf_state->slices[pipe] =
			skl_compute_dbuf_slices(crtc, new_dbuf_state->active_pipes,
						new_dbuf_state->joined_mbus);

		if (old_dbuf_state->slices[pipe] == new_dbuf_state->slices[pipe])
			continue;

		ret = intel_atomic_lock_global_state(&new_dbuf_state->base);
		if (ret)
			return ret;
	}

	new_dbuf_state->enabled_slices = intel_dbuf_enabled_slices(new_dbuf_state);

	if (old_dbuf_state->enabled_slices != new_dbuf_state->enabled_slices ||
	    old_dbuf_state->joined_mbus != new_dbuf_state->joined_mbus) {
		ret = intel_atomic_serialize_global_state(&new_dbuf_state->base);
		if (ret)
			return ret;

		drm_dbg_kms(display->drm,
			    "Enabled dbuf slices 0x%x -> 0x%x (total dbuf slices 0x%x), mbus joined? %s->%s\n",
			    old_dbuf_state->enabled_slices,
			    new_dbuf_state->enabled_slices,
			    DISPLAY_INFO(display)->dbuf.slice_mask,
			    str_yes_no(old_dbuf_state->joined_mbus),
			    str_yes_no(new_dbuf_state->joined_mbus));
	}

	for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i) {
		enum pipe pipe = crtc->pipe;

		new_dbuf_state->weight[pipe] = intel_crtc_ddb_weight(new_crtc_state);

		if (old_dbuf_state->weight[pipe] == new_dbuf_state->weight[pipe])
			continue;

		ret = intel_atomic_lock_global_state(&new_dbuf_state->base);
		if (ret)
			return ret;
	}

	for_each_intel_crtc(display->drm, crtc) {
		ret = skl_crtc_allocate_ddb(state, crtc);
		if (ret)
			return ret;
	}

	for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i) {
		ret = skl_crtc_allocate_plane_ddb(state, crtc);
		if (ret)
			return ret;

		ret = skl_ddb_add_affected_planes(state, crtc);
		if (ret)
			return ret;
	}

	return 0;
}

static char enast(bool enable)
{
	return enable ? '*' : ' ';
}

static noinline_for_stack void
skl_print_plane_changes(struct intel_display *display,
			struct intel_plane *plane,
			const struct skl_plane_wm *old_wm,
			const struct skl_plane_wm *new_wm)
{
	drm_dbg_kms(display->drm,
		    "[PLANE:%d:%s]   level %cwm0,%cwm1,%cwm2,%cwm3,%cwm4,%cwm5,%cwm6,%cwm7,%ctwm,%cswm,%cstwm"
		    " -> %cwm0,%cwm1,%cwm2,%cwm3,%cwm4,%cwm5,%cwm6,%cwm7,%ctwm,%cswm,%cstwm\n",
		    plane->base.base.id, plane->base.name,
		    enast(old_wm->wm[0].enable), enast(old_wm->wm[1].enable),
		    enast(old_wm->wm[2].enable), enast(old_wm->wm[3].enable),
		    enast(old_wm->wm[4].enable), enast(old_wm->wm[5].enable),
		    enast(old_wm->wm[6].enable), enast(old_wm->wm[7].enable),
		    enast(old_wm->trans_wm.enable),
		    enast(old_wm->sagv.wm0.enable),
		    enast(old_wm->sagv.trans_wm.enable),
		    enast(new_wm->wm[0].enable), enast(new_wm->wm[1].enable),
		    enast(new_wm->wm[2].enable), enast(new_wm->wm[3].enable),
		    enast(new_wm->wm[4].enable), enast(new_wm->wm[5].enable),
		    enast(new_wm->wm[6].enable), enast(new_wm->wm[7].enable),
		    enast(new_wm->trans_wm.enable),
		    enast(new_wm->sagv.wm0.enable),
		    enast(new_wm->sagv.trans_wm.enable));

	drm_dbg_kms(display->drm,
		    "[PLANE:%d:%s]   lines %c%3d,%c%3d,%c%3d,%c%3d,%c%3d,%c%3d,%c%3d,%c%3d,%c%3d,%c%3d,%c%4d"
		      " -> %c%3d,%c%3d,%c%3d,%c%3d,%c%3d,%c%3d,%c%3d,%c%3d,%c%3d,%c%3d,%c%4d\n",
		    plane->base.base.id, plane->base.name,
		    enast(old_wm->wm[0].ignore_lines), old_wm->wm[0].lines,
		    enast(old_wm->wm[1].ignore_lines), old_wm->wm[1].lines,
		    enast(old_wm->wm[2].ignore_lines), old_wm->wm[2].lines,
		    enast(old_wm->wm[3].ignore_lines), old_wm->wm[3].lines,
		    enast(old_wm->wm[4].ignore_lines), old_wm->wm[4].lines,
		    enast(old_wm->wm[5].ignore_lines), old_wm->wm[5].lines,
		    enast(old_wm->wm[6].ignore_lines), old_wm->wm[6].lines,
		    enast(old_wm->wm[7].ignore_lines), old_wm->wm[7].lines,
		    enast(old_wm->trans_wm.ignore_lines), old_wm->trans_wm.lines,
		    enast(old_wm->sagv.wm0.ignore_lines), old_wm->sagv.wm0.lines,
		    enast(old_wm->sagv.trans_wm.ignore_lines), old_wm->sagv.trans_wm.lines,
		    enast(new_wm->wm[0].ignore_lines), new_wm->wm[0].lines,
		    enast(new_wm->wm[1].ignore_lines), new_wm->wm[1].lines,
		    enast(new_wm->wm[2].ignore_lines), new_wm->wm[2].lines,
		    enast(new_wm->wm[3].ignore_lines), new_wm->wm[3].lines,
		    enast(new_wm->wm[4].ignore_lines), new_wm->wm[4].lines,
		    enast(new_wm->wm[5].ignore_lines), new_wm->wm[5].lines,
		    enast(new_wm->wm[6].ignore_lines), new_wm->wm[6].lines,
		    enast(new_wm->wm[7].ignore_lines), new_wm->wm[7].lines,
		    enast(new_wm->trans_wm.ignore_lines), new_wm->trans_wm.lines,
		    enast(new_wm->sagv.wm0.ignore_lines), new_wm->sagv.wm0.lines,
		    enast(new_wm->sagv.trans_wm.ignore_lines), new_wm->sagv.trans_wm.lines);

	drm_dbg_kms(display->drm,
		    "[PLANE:%d:%s]  blocks %4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%5d"
		    " -> %4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%5d\n",
		    plane->base.base.id, plane->base.name,
		    old_wm->wm[0].blocks, old_wm->wm[1].blocks,
		    old_wm->wm[2].blocks, old_wm->wm[3].blocks,
		    old_wm->wm[4].blocks, old_wm->wm[5].blocks,
		    old_wm->wm[6].blocks, old_wm->wm[7].blocks,
		    old_wm->trans_wm.blocks,
		    old_wm->sagv.wm0.blocks,
		    old_wm->sagv.trans_wm.blocks,
		    new_wm->wm[0].blocks, new_wm->wm[1].blocks,
		    new_wm->wm[2].blocks, new_wm->wm[3].blocks,
		    new_wm->wm[4].blocks, new_wm->wm[5].blocks,
		    new_wm->wm[6].blocks, new_wm->wm[7].blocks,
		    new_wm->trans_wm.blocks,
		    new_wm->sagv.wm0.blocks,
		    new_wm->sagv.trans_wm.blocks);

	drm_dbg_kms(display->drm,
		    "[PLANE:%d:%s] min_ddb %4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%5d"
		    " -> %4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%5d\n",
		    plane->base.base.id, plane->base.name,
		    old_wm->wm[0].min_ddb_alloc, old_wm->wm[1].min_ddb_alloc,
		    old_wm->wm[2].min_ddb_alloc, old_wm->wm[3].min_ddb_alloc,
		    old_wm->wm[4].min_ddb_alloc, old_wm->wm[5].min_ddb_alloc,
		    old_wm->wm[6].min_ddb_alloc, old_wm->wm[7].min_ddb_alloc,
		    old_wm->trans_wm.min_ddb_alloc,
		    old_wm->sagv.wm0.min_ddb_alloc,
		    old_wm->sagv.trans_wm.min_ddb_alloc,
		    new_wm->wm[0].min_ddb_alloc, new_wm->wm[1].min_ddb_alloc,
		    new_wm->wm[2].min_ddb_alloc, new_wm->wm[3].min_ddb_alloc,
		    new_wm->wm[4].min_ddb_alloc, new_wm->wm[5].min_ddb_alloc,
		    new_wm->wm[6].min_ddb_alloc, new_wm->wm[7].min_ddb_alloc,
		    new_wm->trans_wm.min_ddb_alloc,
		    new_wm->sagv.wm0.min_ddb_alloc,
		    new_wm->sagv.trans_wm.min_ddb_alloc);
}

static void
skl_print_wm_changes(struct intel_atomic_state *state)
{
	struct intel_display *display = to_intel_display(state);
	const struct intel_crtc_state *old_crtc_state;
	const struct intel_crtc_state *new_crtc_state;
	struct intel_plane *plane;
	struct intel_crtc *crtc;
	int i;

	if (!drm_debug_enabled(DRM_UT_KMS))
		return;

	for_each_oldnew_intel_crtc_in_state(state, crtc, old_crtc_state,
					    new_crtc_state, i) {
		const struct skl_pipe_wm *old_pipe_wm, *new_pipe_wm;

		old_pipe_wm = &old_crtc_state->wm.skl.optimal;
		new_pipe_wm = &new_crtc_state->wm.skl.optimal;

		for_each_intel_plane_on_crtc(display->drm, crtc, plane) {
			enum plane_id plane_id = plane->id;
			const struct skl_ddb_entry *old, *new;

			old = &old_crtc_state->wm.skl.plane_ddb[plane_id];
			new = &new_crtc_state->wm.skl.plane_ddb[plane_id];

			if (skl_ddb_entry_equal(old, new))
				continue;
			drm_dbg_kms(display->drm,
				    "[PLANE:%d:%s] ddb (%4d - %4d) -> (%4d - %4d), size %4d -> %4d\n",
				    plane->base.base.id, plane->base.name,
				    old->start, old->end, new->start, new->end,
				    skl_ddb_entry_size(old), skl_ddb_entry_size(new));
		}

		for_each_intel_plane_on_crtc(display->drm, crtc, plane) {
			enum plane_id plane_id = plane->id;
			const struct skl_plane_wm *old_wm, *new_wm;

			old_wm = &old_pipe_wm->planes[plane_id];
			new_wm = &new_pipe_wm->planes[plane_id];

			if (skl_plane_wm_equals(display, old_wm, new_wm))
				continue;

			skl_print_plane_changes(display, plane, old_wm, new_wm);
		}
	}
}

static bool skl_plane_selected_wm_equals(struct intel_plane *plane,
					 const struct skl_pipe_wm *old_pipe_wm,
					 const struct skl_pipe_wm *new_pipe_wm)
{
	struct intel_display *display = to_intel_display(plane);
	int level;

	for (level = 0; level < display->wm.num_levels; level++) {
		/*
		 * We don't check uv_wm as the hardware doesn't actually
		 * use it. It only gets used for calculating the required
		 * ddb allocation.
		 */
		if (!skl_wm_level_equals(skl_plane_wm_level(old_pipe_wm, plane->id, level),
					 skl_plane_wm_level(new_pipe_wm, plane->id, level)))
			return false;
	}

	if (HAS_HW_SAGV_WM(display)) {
		const struct skl_plane_wm *old_wm = &old_pipe_wm->planes[plane->id];
		const struct skl_plane_wm *new_wm = &new_pipe_wm->planes[plane->id];

		if (!skl_wm_level_equals(&old_wm->sagv.wm0, &new_wm->sagv.wm0) ||
		    !skl_wm_level_equals(&old_wm->sagv.trans_wm, &new_wm->sagv.trans_wm))
			return false;
	}

	return skl_wm_level_equals(skl_plane_trans_wm(old_pipe_wm, plane->id),
				   skl_plane_trans_wm(new_pipe_wm, plane->id));
}

/*
 * To make sure the cursor watermark registers are always consistent
 * with our computed state the following scenario needs special
 * treatment:
 *
 * 1. enable cursor
 * 2. move cursor entirely offscreen
 * 3. disable cursor
 *
 * Step 2. does call .disable_plane() but does not zero the watermarks
 * (since we consider an offscreen cursor still active for the purposes
 * of watermarks). Step 3. would not normally call .disable_plane()
 * because the actual plane visibility isn't changing, and we don't
 * deallocate the cursor ddb until the pipe gets disabled. So we must
 * force step 3. to call .disable_plane() to update the watermark
 * registers properly.
 *
 * Other planes do not suffer from this issues as their watermarks are
 * calculated based on the actual plane visibility. The only time this
 * can trigger for the other planes is during the initial readout as the
 * default value of the watermarks registers is not zero.
 */
static int skl_wm_add_affected_planes(struct intel_atomic_state *state,
				      struct intel_crtc *crtc)
{
	struct intel_display *display = to_intel_display(state);
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	struct intel_plane *plane;

	for_each_intel_plane_on_crtc(display->drm, crtc, plane) {
		struct intel_plane_state *plane_state;
		enum plane_id plane_id = plane->id;

		/*
		 * Force a full wm update for every plane on modeset.
		 * Required because the reset value of the wm registers
		 * is non-zero, whereas we want all disabled planes to
		 * have zero watermarks. So if we turn off the relevant
		 * power well the hardware state will go out of sync
		 * with the software state.
		 */
		if (!intel_crtc_needs_modeset(new_crtc_state) &&
		    skl_plane_selected_wm_equals(plane,
						 &old_crtc_state->wm.skl.optimal,
						 &new_crtc_state->wm.skl.optimal))
			continue;

		if (new_crtc_state->do_async_flip) {
			drm_dbg_kms(display->drm, "[PLANE:%d:%s] Can't change watermarks during async flip\n",
				    plane->base.base.id, plane->base.name);
			return -EINVAL;
		}

		plane_state = intel_atomic_get_plane_state(state, plane);
		if (IS_ERR(plane_state))
			return PTR_ERR(plane_state);

		new_crtc_state->update_planes |= BIT(plane_id);
		new_crtc_state->async_flip_planes = 0;
		new_crtc_state->do_async_flip = false;
	}

	return 0;
}

static int pkgc_max_linetime(struct intel_atomic_state *state)
{
	struct intel_display *display = to_intel_display(state);
	const struct intel_crtc_state *crtc_state;
	struct intel_crtc *crtc;
	int i, max_linetime;

	/*
	 * Apparenty the hardware uses WM_LINETIME internally for
	 * this stuff, compute everything based on that.
	 */
	for_each_new_intel_crtc_in_state(state, crtc, crtc_state, i) {
		display->pkgc.disable[crtc->pipe] = crtc_state->vrr.enable;
		display->pkgc.linetime[crtc->pipe] = DIV_ROUND_UP(crtc_state->linetime, 8);
	}

	max_linetime = 0;
	for_each_intel_crtc(display->drm, crtc) {
		if (display->pkgc.disable[crtc->pipe])
			return 0;

		max_linetime = max(display->pkgc.linetime[crtc->pipe], max_linetime);
	}

	return max_linetime;
}

void
intel_program_dpkgc_latency(struct intel_atomic_state *state)
{
	struct intel_display *display = to_intel_display(state);
	int max_linetime, latency, added_wake_time = 0;

	if (DISPLAY_VER(display) < 20)
		return;

	mutex_lock(&display->wm.wm_mutex);

	latency = skl_watermark_max_latency(display, 1);

	/* FIXME runtime changes to enable_flipq are racy */
	if (display->params.enable_flipq)
		added_wake_time = intel_flipq_exec_time_us(display);

	/*
	 * Wa_22020432604
	 * "PKG_C_LATENCY Added Wake Time field is not working"
	 */
	if (latency && IS_DISPLAY_VER(display, 20, 30)) {
		latency += added_wake_time;
		added_wake_time = 0;
	}

	max_linetime = pkgc_max_linetime(state);

	if (max_linetime == 0 || latency == 0) {
		latency = REG_FIELD_GET(LNL_PKG_C_LATENCY_MASK,
					LNL_PKG_C_LATENCY_MASK);
		added_wake_time = 0;
	} else {
		/*
		 * Wa_22020299601
		 * "Increase the latency programmed in PKG_C_LATENCY Pkg C Latency to be a
		 *  multiple of the pipeline time from WM_LINETIME"
		 */
		latency = roundup(latency, max_linetime);
	}

	intel_de_write(display, LNL_PKG_C_LATENCY,
		       REG_FIELD_PREP(LNL_ADDED_WAKE_TIME_MASK, added_wake_time) |
		       REG_FIELD_PREP(LNL_PKG_C_LATENCY_MASK, latency));

	mutex_unlock(&display->wm.wm_mutex);
}

static int
skl_compute_wm(struct intel_atomic_state *state)
{
	struct intel_display *display = to_intel_display(state);
	struct intel_crtc *crtc;
	struct intel_crtc_state __maybe_unused *new_crtc_state;
	int ret, i;

	for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i) {
		ret = skl_build_pipe_wm(state, crtc);
		if (ret)
			return ret;
	}

	ret = skl_compute_ddb(state);
	if (ret)
		return ret;

	/*
	 * skl_compute_ddb() will have adjusted the final watermarks
	 * based on how much ddb is available. Now we can actually
	 * check if the final watermarks changed.
	 */
	for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i) {
		struct skl_pipe_wm *pipe_wm = &new_crtc_state->wm.skl.optimal;

		/*
		 * We store use_sagv_wm in the crtc state rather than relying on
		 * that bw state since we have no convenient way to get at the
		 * latter from the plane commit hooks (especially in the legacy
		 * cursor case).
		 *
		 * drm_atomic_check_only() gets upset if we pull more crtcs
		 * into the state, so we have to calculate this based on the
		 * individual intel_crtc_can_enable_sagv() rather than
		 * the overall intel_bw_can_enable_sagv(). Otherwise the
		 * crtcs not included in the commit would not switch to the
		 * SAGV watermarks when we are about to enable SAGV, and that
		 * would lead to underruns. This does mean extra power draw
		 * when only a subset of the crtcs are blocking SAGV as the
		 * other crtcs can't be allowed to use the more optimal
		 * normal (ie. non-SAGV) watermarks.
		 */
		pipe_wm->use_sagv_wm = !HAS_HW_SAGV_WM(display) &&
			DISPLAY_VER(display) >= 12 &&
			intel_crtc_can_enable_sagv(new_crtc_state);

		ret = skl_wm_add_affected_planes(state, crtc);
		if (ret)
			return ret;
	}

	skl_print_wm_changes(state);

	return 0;
}

static void skl_wm_level_from_reg_val(struct intel_display *display,
				      u32 val, struct skl_wm_level *level)
{
	level->enable = val & PLANE_WM_EN;
	level->ignore_lines = val & PLANE_WM_IGNORE_LINES;
	level->blocks = REG_FIELD_GET(PLANE_WM_BLOCKS_MASK, val);
	level->lines = REG_FIELD_GET(PLANE_WM_LINES_MASK, val);
	level->auto_min_alloc_wm_enable = DISPLAY_VER(display) >= 30 ?
					   val & PLANE_WM_AUTO_MIN_ALLOC_EN : 0;
}

static void skl_pipe_wm_get_hw_state(struct intel_crtc *crtc,
				     struct skl_pipe_wm *out)
{
	struct intel_display *display = to_intel_display(crtc);
	enum pipe pipe = crtc->pipe;
	enum plane_id plane_id;
	int level;
	u32 val;

	for_each_plane_id_on_crtc(crtc, plane_id) {
		struct skl_plane_wm *wm = &out->planes[plane_id];

		for (level = 0; level < display->wm.num_levels; level++) {
			if (plane_id != PLANE_CURSOR)
				val = intel_de_read(display, PLANE_WM(pipe, plane_id, level));
			else
				val = intel_de_read(display, CUR_WM(pipe, level));

			skl_wm_level_from_reg_val(display, val, &wm->wm[level]);
		}

		if (plane_id != PLANE_CURSOR)
			val = intel_de_read(display, PLANE_WM_TRANS(pipe, plane_id));
		else
			val = intel_de_read(display, CUR_WM_TRANS(pipe));

		skl_wm_level_from_reg_val(display, val, &wm->trans_wm);

		if (HAS_HW_SAGV_WM(display)) {
			if (plane_id != PLANE_CURSOR)
				val = intel_de_read(display, PLANE_WM_SAGV(pipe, plane_id));
			else
				val = intel_de_read(display, CUR_WM_SAGV(pipe));

			skl_wm_level_from_reg_val(display, val, &wm->sagv.wm0);

			if (plane_id != PLANE_CURSOR)
				val = intel_de_read(display, PLANE_WM_SAGV_TRANS(pipe, plane_id));
			else
				val = intel_de_read(display, CUR_WM_SAGV_TRANS(pipe));

			skl_wm_level_from_reg_val(display, val, &wm->sagv.trans_wm);
		} else if (DISPLAY_VER(display) >= 12) {
			wm->sagv.wm0 = wm->wm[0];
			wm->sagv.trans_wm = wm->trans_wm;
		}
	}
}

static void skl_wm_get_hw_state(struct intel_display *display)
{
	struct intel_dbuf_state *dbuf_state =
		to_intel_dbuf_state(display->dbuf.obj.state);
	struct intel_crtc *crtc;

	if (HAS_MBUS_JOINING(display))
		dbuf_state->joined_mbus = intel_de_read(display, MBUS_CTL) & MBUS_JOIN;

	dbuf_state->mdclk_cdclk_ratio = intel_mdclk_cdclk_ratio(display, &display->cdclk.hw);
	dbuf_state->active_pipes = 0;

	for_each_intel_crtc(display->drm, crtc) {
		struct intel_crtc_state *crtc_state =
			to_intel_crtc_state(crtc->base.state);
		enum pipe pipe = crtc->pipe;
		unsigned int mbus_offset;
		enum plane_id plane_id;
		u8 slices;

		memset(&crtc_state->wm.skl.optimal, 0,
		       sizeof(crtc_state->wm.skl.optimal));
		if (crtc_state->hw.active) {
			skl_pipe_wm_get_hw_state(crtc, &crtc_state->wm.skl.optimal);
			dbuf_state->active_pipes |= BIT(pipe);
		}
		crtc_state->wm.skl.raw = crtc_state->wm.skl.optimal;

		memset(&dbuf_state->ddb[pipe], 0, sizeof(dbuf_state->ddb[pipe]));

		for_each_plane_id_on_crtc(crtc, plane_id) {
			struct skl_ddb_entry *ddb =
				&crtc_state->wm.skl.plane_ddb[plane_id];
			struct skl_ddb_entry *ddb_y =
				&crtc_state->wm.skl.plane_ddb_y[plane_id];
			u16 *min_ddb =
				&crtc_state->wm.skl.plane_min_ddb[plane_id];
			u16 *interim_ddb =
				&crtc_state->wm.skl.plane_interim_ddb[plane_id];

			if (!crtc_state->hw.active)
				continue;

			skl_ddb_get_hw_plane_state(display, crtc->pipe,
						   plane_id, ddb, ddb_y,
						   min_ddb, interim_ddb);

			skl_ddb_entry_union(&dbuf_state->ddb[pipe], ddb);
			skl_ddb_entry_union(&dbuf_state->ddb[pipe], ddb_y);
		}

		dbuf_state->weight[pipe] = intel_crtc_ddb_weight(crtc_state);

		/*
		 * Used for checking overlaps, so we need absolute
		 * offsets instead of MBUS relative offsets.
		 */
		slices = skl_compute_dbuf_slices(crtc, dbuf_state->active_pipes,
						 dbuf_state->joined_mbus);
		mbus_offset = mbus_ddb_offset(display, slices);
		crtc_state->wm.skl.ddb.start = mbus_offset + dbuf_state->ddb[pipe].start;
		crtc_state->wm.skl.ddb.end = mbus_offset + dbuf_state->ddb[pipe].end;

		/* The slices actually used by the planes on the pipe */
		dbuf_state->slices[pipe] =
			skl_ddb_dbuf_slice_mask(display, &crtc_state->wm.skl.ddb);

		drm_dbg_kms(display->drm,
			    "[CRTC:%d:%s] dbuf slices 0x%x, ddb (%d - %d), active pipes 0x%x, mbus joined: %s\n",
			    crtc->base.base.id, crtc->base.name,
			    dbuf_state->slices[pipe], dbuf_state->ddb[pipe].start,
			    dbuf_state->ddb[pipe].end, dbuf_state->active_pipes,
			    str_yes_no(dbuf_state->joined_mbus));
	}

	dbuf_state->enabled_slices = display->dbuf.enabled_slices;
}

bool skl_watermark_ipc_enabled(struct intel_display *display)
{
	return display->wm.ipc_enabled;
}

void skl_watermark_ipc_update(struct intel_display *display)
{
	if (!HAS_IPC(display))
		return;

	intel_de_rmw(display, DISP_ARB_CTL2, DISP_IPC_ENABLE,
		     skl_watermark_ipc_enabled(display) ? DISP_IPC_ENABLE : 0);
}

static bool skl_watermark_ipc_can_enable(struct intel_display *display)
{
	/* Display WA #0477 WaDisableIPC: skl */
	if (display->platform.skylake)
		return false;

	/* Display WA #1141: SKL:all KBL:all CFL */
	if (display->platform.kabylake ||
	    display->platform.coffeelake ||
	    display->platform.cometlake) {
		const struct dram_info *dram_info = intel_dram_info(display->drm);

		return dram_info->symmetric_memory;
	}

	return true;
}

void skl_watermark_ipc_init(struct intel_display *display)
{
	if (!HAS_IPC(display))
		return;

	display->wm.ipc_enabled = skl_watermark_ipc_can_enable(display);

	skl_watermark_ipc_update(display);
}

static void
adjust_wm_latency(struct intel_display *display,
		  u16 wm[], int num_levels, int read_latency)
{
	const struct dram_info *dram_info = intel_dram_info(display->drm);
	int i, level;

	/*
	 * If a level n (n > 1) has a 0us latency, all levels m (m >= n)
	 * need to be disabled. We make sure to sanitize the values out
	 * of the punit to satisfy this requirement.
	 */
	for (level = 1; level < num_levels; level++) {
		if (wm[level] == 0) {
			for (i = level + 1; i < num_levels; i++)
				wm[i] = 0;

			num_levels = level;
			break;
		}
	}

	/*
	 * WaWmMemoryReadLatency
	 *
	 * punit doesn't take into account the read latency so we need
	 * to add proper adjustment to each valid level we retrieve
	 * from the punit when level 0 response data is 0us.
	 */
	if (wm[0] == 0) {
		for (level = 0; level < num_levels; level++)
			wm[level] += read_latency;
	}

	/*
	 * WA Level-0 adjustment for 16GB DIMMs: SKL+
	 * If we could not get dimm info enable this WA to prevent from
	 * any underrun. If not able to get Dimm info assume 16GB dimm
	 * to avoid any underrun.
	 */
	if (!display->platform.dg2 && dram_info->wm_lv_0_adjust_needed)
		wm[0] += 1;
}

static void mtl_read_wm_latency(struct intel_display *display, u16 wm[])
{
	int num_levels = display->wm.num_levels;
	u32 val;

	val = intel_de_read(display, MTL_LATENCY_LP0_LP1);
	wm[0] = REG_FIELD_GET(MTL_LATENCY_LEVEL_EVEN_MASK, val);
	wm[1] = REG_FIELD_GET(MTL_LATENCY_LEVEL_ODD_MASK, val);

	val = intel_de_read(display, MTL_LATENCY_LP2_LP3);
	wm[2] = REG_FIELD_GET(MTL_LATENCY_LEVEL_EVEN_MASK, val);
	wm[3] = REG_FIELD_GET(MTL_LATENCY_LEVEL_ODD_MASK, val);

	val = intel_de_read(display, MTL_LATENCY_LP4_LP5);
	wm[4] = REG_FIELD_GET(MTL_LATENCY_LEVEL_EVEN_MASK, val);
	wm[5] = REG_FIELD_GET(MTL_LATENCY_LEVEL_ODD_MASK, val);

	adjust_wm_latency(display, wm, num_levels, 6);
}

static void skl_read_wm_latency(struct intel_display *display, u16 wm[])
{
	int num_levels = display->wm.num_levels;
	int read_latency = DISPLAY_VER(display) >= 12 ? 3 : 2;
	int mult = display->platform.dg2 ? 2 : 1;
	u32 val;
	int ret;

	/* read the first set of memory latencies[0:3] */
	val = 0; /* data0 to be programmed to 0 for first set */
	ret = intel_pcode_read(display->drm, GEN9_PCODE_READ_MEM_LATENCY, &val, NULL);
	if (ret) {
		drm_err(display->drm, "SKL Mailbox read error = %d\n", ret);
		return;
	}

	wm[0] = REG_FIELD_GET(GEN9_MEM_LATENCY_LEVEL_0_4_MASK, val) * mult;
	wm[1] = REG_FIELD_GET(GEN9_MEM_LATENCY_LEVEL_1_5_MASK, val) * mult;
	wm[2] = REG_FIELD_GET(GEN9_MEM_LATENCY_LEVEL_2_6_MASK, val) * mult;
	wm[3] = REG_FIELD_GET(GEN9_MEM_LATENCY_LEVEL_3_7_MASK, val) * mult;

	/* read the second set of memory latencies[4:7] */
	val = 1; /* data0 to be programmed to 1 for second set */
	ret = intel_pcode_read(display->drm, GEN9_PCODE_READ_MEM_LATENCY, &val, NULL);
	if (ret) {
		drm_err(display->drm, "SKL Mailbox read error = %d\n", ret);
		return;
	}

	wm[4] = REG_FIELD_GET(GEN9_MEM_LATENCY_LEVEL_0_4_MASK, val) * mult;
	wm[5] = REG_FIELD_GET(GEN9_MEM_LATENCY_LEVEL_1_5_MASK, val) * mult;
	wm[6] = REG_FIELD_GET(GEN9_MEM_LATENCY_LEVEL_2_6_MASK, val) * mult;
	wm[7] = REG_FIELD_GET(GEN9_MEM_LATENCY_LEVEL_3_7_MASK, val) * mult;

	adjust_wm_latency(display, wm, num_levels, read_latency);
}

static void skl_setup_wm_latency(struct intel_display *display)
{
	if (HAS_HW_SAGV_WM(display))
		display->wm.num_levels = 6;
	else
		display->wm.num_levels = 8;

	if (DISPLAY_VER(display) >= 14)
		mtl_read_wm_latency(display, display->wm.skl_latency);
	else
		skl_read_wm_latency(display, display->wm.skl_latency);

	intel_print_wm_latency(display, "Gen9 Plane", display->wm.skl_latency);
}

static struct intel_global_state *intel_dbuf_duplicate_state(struct intel_global_obj *obj)
{
	struct intel_dbuf_state *dbuf_state;

	dbuf_state = kmemdup(obj->state, sizeof(*dbuf_state), GFP_KERNEL);
	if (!dbuf_state)
		return NULL;

	return &dbuf_state->base;
}

static void intel_dbuf_destroy_state(struct intel_global_obj *obj,
				     struct intel_global_state *state)
{
	kfree(state);
}

static const struct intel_global_state_funcs intel_dbuf_funcs = {
	.atomic_duplicate_state = intel_dbuf_duplicate_state,
	.atomic_destroy_state = intel_dbuf_destroy_state,
};

struct intel_dbuf_state *
intel_atomic_get_dbuf_state(struct intel_atomic_state *state)
{
	struct intel_display *display = to_intel_display(state);
	struct intel_global_state *dbuf_state;

	dbuf_state = intel_atomic_get_global_obj_state(state, &display->dbuf.obj);
	if (IS_ERR(dbuf_state))
		return ERR_CAST(dbuf_state);

	return to_intel_dbuf_state(dbuf_state);
}

int intel_dbuf_init(struct intel_display *display)
{
	struct intel_dbuf_state *dbuf_state;

	dbuf_state = kzalloc(sizeof(*dbuf_state), GFP_KERNEL);
	if (!dbuf_state)
		return -ENOMEM;

	intel_atomic_global_obj_init(display, &display->dbuf.obj,
				     &dbuf_state->base, &intel_dbuf_funcs);

	return 0;
}

static bool xelpdp_is_only_pipe_per_dbuf_bank(enum pipe pipe, u8 active_pipes)
{
	switch (pipe) {
	case PIPE_A:
	case PIPE_D:
		active_pipes &= BIT(PIPE_A) | BIT(PIPE_D);
		break;
	case PIPE_B:
	case PIPE_C:
		active_pipes &= BIT(PIPE_B) | BIT(PIPE_C);
		break;
	default: /* to suppress compiler warning */
		MISSING_CASE(pipe);
		return false;
	}

	return is_power_of_2(active_pipes);
}

static u32 pipe_mbus_dbox_ctl(const struct intel_crtc *crtc,
			      const struct intel_dbuf_state *dbuf_state)
{
	struct intel_display *display = to_intel_display(crtc);
	u32 val = 0;

	if (DISPLAY_VER(display) >= 14)
		val |= MBUS_DBOX_I_CREDIT(2);

	if (DISPLAY_VER(display) >= 12) {
		val |= MBUS_DBOX_B2B_TRANSACTIONS_MAX(16);
		val |= MBUS_DBOX_B2B_TRANSACTIONS_DELAY(1);
		val |= MBUS_DBOX_REGULATE_B2B_TRANSACTIONS_EN;
	}

	if (DISPLAY_VER(display) >= 14)
		val |= dbuf_state->joined_mbus ?
			MBUS_DBOX_A_CREDIT(12) : MBUS_DBOX_A_CREDIT(8);
	else if (display->platform.alderlake_p)
		/* Wa_22010947358:adl-p */
		val |= dbuf_state->joined_mbus ?
			MBUS_DBOX_A_CREDIT(6) : MBUS_DBOX_A_CREDIT(4);
	else
		val |= MBUS_DBOX_A_CREDIT(2);

	if (DISPLAY_VER(display) >= 14) {
		val |= MBUS_DBOX_B_CREDIT(0xA);
	} else if (display->platform.alderlake_p) {
		val |= MBUS_DBOX_BW_CREDIT(2);
		val |= MBUS_DBOX_B_CREDIT(8);
	} else if (DISPLAY_VER(display) >= 12) {
		val |= MBUS_DBOX_BW_CREDIT(2);
		val |= MBUS_DBOX_B_CREDIT(12);
	} else {
		val |= MBUS_DBOX_BW_CREDIT(1);
		val |= MBUS_DBOX_B_CREDIT(8);
	}

	if (DISPLAY_VERx100(display) == 1400) {
		if (xelpdp_is_only_pipe_per_dbuf_bank(crtc->pipe, dbuf_state->active_pipes))
			val |= MBUS_DBOX_BW_8CREDITS_MTL;
		else
			val |= MBUS_DBOX_BW_4CREDITS_MTL;
	}

	return val;
}

static void pipe_mbus_dbox_ctl_update(struct intel_display *display,
				      const struct intel_dbuf_state *dbuf_state)
{
	struct intel_crtc *crtc;

	for_each_intel_crtc_in_pipe_mask(display->drm, crtc, dbuf_state->active_pipes)
		intel_de_write(display, PIPE_MBUS_DBOX_CTL(crtc->pipe),
			       pipe_mbus_dbox_ctl(crtc, dbuf_state));
}

static void intel_mbus_dbox_update(struct intel_atomic_state *state)
{
	struct intel_display *display = to_intel_display(state);
	const struct intel_dbuf_state *new_dbuf_state, *old_dbuf_state;

	if (DISPLAY_VER(display) < 11)
		return;

	new_dbuf_state = intel_atomic_get_new_dbuf_state(state);
	old_dbuf_state = intel_atomic_get_old_dbuf_state(state);
	if (!new_dbuf_state ||
	    (new_dbuf_state->joined_mbus == old_dbuf_state->joined_mbus &&
	     new_dbuf_state->active_pipes == old_dbuf_state->active_pipes))
		return;

	pipe_mbus_dbox_ctl_update(display, new_dbuf_state);
}

int intel_dbuf_state_set_mdclk_cdclk_ratio(struct intel_atomic_state *state,
					   int ratio)
{
	struct intel_dbuf_state *dbuf_state;

	dbuf_state = intel_atomic_get_dbuf_state(state);
	if (IS_ERR(dbuf_state))
		return PTR_ERR(dbuf_state);

	dbuf_state->mdclk_cdclk_ratio = ratio;

	return intel_atomic_lock_global_state(&dbuf_state->base);
}

void intel_dbuf_mdclk_cdclk_ratio_update(struct intel_display *display,
					 int ratio, bool joined_mbus)
{
	enum dbuf_slice slice;

	if (!HAS_MBUS_JOINING(display))
		return;

	if (DISPLAY_VER(display) >= 20)
		intel_de_rmw(display, MBUS_CTL, MBUS_TRANSLATION_THROTTLE_MIN_MASK,
			     MBUS_TRANSLATION_THROTTLE_MIN(ratio - 1));

	if (joined_mbus)
		ratio *= 2;

	drm_dbg_kms(display->drm, "Updating dbuf ratio to %d (mbus joined: %s)\n",
		    ratio, str_yes_no(joined_mbus));

	for_each_dbuf_slice(display, slice)
		intel_de_rmw(display, DBUF_CTL_S(slice),
			     DBUF_MIN_TRACKER_STATE_SERVICE_MASK,
			     DBUF_MIN_TRACKER_STATE_SERVICE(ratio - 1));
}

static void intel_dbuf_mdclk_min_tracker_update(struct intel_atomic_state *state)
{
	struct intel_display *display = to_intel_display(state);
	const struct intel_dbuf_state *old_dbuf_state =
		intel_atomic_get_old_dbuf_state(state);
	const struct intel_dbuf_state *new_dbuf_state =
		intel_atomic_get_new_dbuf_state(state);
	int mdclk_cdclk_ratio;

	if (intel_cdclk_is_decreasing_later(state)) {
		/* cdclk/mdclk will be changed later by intel_set_cdclk_post_plane_update() */
		mdclk_cdclk_ratio = old_dbuf_state->mdclk_cdclk_ratio;
	} else {
		/* cdclk/mdclk already changed by intel_set_cdclk_pre_plane_update() */
		mdclk_cdclk_ratio = new_dbuf_state->mdclk_cdclk_ratio;
	}

	intel_dbuf_mdclk_cdclk_ratio_update(display, mdclk_cdclk_ratio,
					    new_dbuf_state->joined_mbus);
}

static enum pipe intel_mbus_joined_pipe(struct intel_atomic_state *state,
					const struct intel_dbuf_state *dbuf_state)
{
	struct intel_display *display = to_intel_display(state);
	enum pipe pipe = ffs(dbuf_state->active_pipes) - 1;
	const struct intel_crtc_state *new_crtc_state;
	struct intel_crtc *crtc;

	drm_WARN_ON(display->drm, !dbuf_state->joined_mbus);
	drm_WARN_ON(display->drm, !is_power_of_2(dbuf_state->active_pipes));

	crtc = intel_crtc_for_pipe(display, pipe);
	new_crtc_state = intel_atomic_get_new_crtc_state(state, crtc);

	if (new_crtc_state && !intel_crtc_needs_modeset(new_crtc_state))
		return pipe;
	else
		return INVALID_PIPE;
}

static void mbus_ctl_join_update(struct intel_display *display,
				 const struct intel_dbuf_state *dbuf_state,
				 enum pipe pipe)
{
	u32 mbus_ctl;

	if (dbuf_state->joined_mbus)
		mbus_ctl = MBUS_HASHING_MODE_1x4 | MBUS_JOIN;
	else
		mbus_ctl = MBUS_HASHING_MODE_2x2;

	if (pipe != INVALID_PIPE)
		mbus_ctl |= MBUS_JOIN_PIPE_SELECT(pipe);
	else
		mbus_ctl |= MBUS_JOIN_PIPE_SELECT_NONE;

	intel_de_rmw(display, MBUS_CTL,
		     MBUS_HASHING_MODE_MASK | MBUS_JOIN |
		     MBUS_JOIN_PIPE_SELECT_MASK, mbus_ctl);
}

static void intel_dbuf_mbus_join_update(struct intel_atomic_state *state,
					enum pipe pipe)
{
	struct intel_display *display = to_intel_display(state);
	const struct intel_dbuf_state *old_dbuf_state =
		intel_atomic_get_old_dbuf_state(state);
	const struct intel_dbuf_state *new_dbuf_state =
		intel_atomic_get_new_dbuf_state(state);

	drm_dbg_kms(display->drm, "Changing mbus joined: %s -> %s (pipe: %c)\n",
		    str_yes_no(old_dbuf_state->joined_mbus),
		    str_yes_no(new_dbuf_state->joined_mbus),
		    pipe != INVALID_PIPE ? pipe_name(pipe) : '*');

	mbus_ctl_join_update(display, new_dbuf_state, pipe);
}

void intel_dbuf_mbus_pre_ddb_update(struct intel_atomic_state *state)
{
	const struct intel_dbuf_state *new_dbuf_state =
		intel_atomic_get_new_dbuf_state(state);
	const struct intel_dbuf_state *old_dbuf_state =
		intel_atomic_get_old_dbuf_state(state);

	if (!new_dbuf_state)
		return;

	if (!old_dbuf_state->joined_mbus && new_dbuf_state->joined_mbus) {
		enum pipe pipe = intel_mbus_joined_pipe(state, new_dbuf_state);

		WARN_ON(!new_dbuf_state->base.changed);

		intel_dbuf_mbus_join_update(state, pipe);
		intel_mbus_dbox_update(state);
		intel_dbuf_mdclk_min_tracker_update(state);
	}
}

void intel_dbuf_mbus_post_ddb_update(struct intel_atomic_state *state)
{
	struct intel_display *display = to_intel_display(state);
	const struct intel_dbuf_state *new_dbuf_state =
		intel_atomic_get_new_dbuf_state(state);
	const struct intel_dbuf_state *old_dbuf_state =
		intel_atomic_get_old_dbuf_state(state);

	if (!new_dbuf_state)
		return;

	if (old_dbuf_state->joined_mbus && !new_dbuf_state->joined_mbus) {
		enum pipe pipe = intel_mbus_joined_pipe(state, old_dbuf_state);

		WARN_ON(!new_dbuf_state->base.changed);

		intel_dbuf_mdclk_min_tracker_update(state);
		intel_mbus_dbox_update(state);
		intel_dbuf_mbus_join_update(state, pipe);

		if (pipe != INVALID_PIPE) {
			struct intel_crtc *crtc = intel_crtc_for_pipe(display, pipe);

			intel_crtc_wait_for_next_vblank(crtc);
		}
	} else if (old_dbuf_state->joined_mbus == new_dbuf_state->joined_mbus &&
		   old_dbuf_state->active_pipes != new_dbuf_state->active_pipes) {
		WARN_ON(!new_dbuf_state->base.changed);

		intel_dbuf_mdclk_min_tracker_update(state);
		intel_mbus_dbox_update(state);
	}

}

void intel_dbuf_pre_plane_update(struct intel_atomic_state *state)
{
	struct intel_display *display = to_intel_display(state);
	const struct intel_dbuf_state *new_dbuf_state =
		intel_atomic_get_new_dbuf_state(state);
	const struct intel_dbuf_state *old_dbuf_state =
		intel_atomic_get_old_dbuf_state(state);
	u8 old_slices, new_slices;

	if (!new_dbuf_state)
		return;

	old_slices = old_dbuf_state->enabled_slices;
	new_slices = old_dbuf_state->enabled_slices | new_dbuf_state->enabled_slices;

	if (old_slices == new_slices)
		return;

	WARN_ON(!new_dbuf_state->base.changed);

	gen9_dbuf_slices_update(display, new_slices);
}

void intel_dbuf_post_plane_update(struct intel_atomic_state *state)
{
	struct intel_display *display = to_intel_display(state);
	const struct intel_dbuf_state *new_dbuf_state =
		intel_atomic_get_new_dbuf_state(state);
	const struct intel_dbuf_state *old_dbuf_state =
		intel_atomic_get_old_dbuf_state(state);
	u8 old_slices, new_slices;

	if (!new_dbuf_state)
		return;

	old_slices = old_dbuf_state->enabled_slices | new_dbuf_state->enabled_slices;
	new_slices = new_dbuf_state->enabled_slices;

	if (old_slices == new_slices)
		return;

	WARN_ON(!new_dbuf_state->base.changed);

	gen9_dbuf_slices_update(display, new_slices);
}

int intel_dbuf_num_enabled_slices(const struct intel_dbuf_state *dbuf_state)
{
	return hweight8(dbuf_state->enabled_slices);
}

int intel_dbuf_num_active_pipes(const struct intel_dbuf_state *dbuf_state)
{
	return hweight8(dbuf_state->active_pipes);
}

bool intel_dbuf_pmdemand_needs_update(struct intel_atomic_state *state)
{
	struct intel_display *display = to_intel_display(state);
	const struct intel_dbuf_state *new_dbuf_state, *old_dbuf_state;

	new_dbuf_state = intel_atomic_get_new_dbuf_state(state);
	old_dbuf_state = intel_atomic_get_old_dbuf_state(state);

	if (new_dbuf_state &&
	    new_dbuf_state->active_pipes != old_dbuf_state->active_pipes)
		return true;

	if (DISPLAY_VER(display) < 30) {
		if (new_dbuf_state &&
		    new_dbuf_state->enabled_slices !=
		    old_dbuf_state->enabled_slices)
			return true;
	}

	return false;
}

static void skl_mbus_sanitize(struct intel_display *display)
{
	struct intel_dbuf_state *dbuf_state =
		to_intel_dbuf_state(display->dbuf.obj.state);

	if (!HAS_MBUS_JOINING(display))
		return;

	if (!dbuf_state->joined_mbus ||
	    adlp_check_mbus_joined(dbuf_state->active_pipes))
		return;

	drm_dbg_kms(display->drm, "Disabling redundant MBUS joining (active pipes 0x%x)\n",
		    dbuf_state->active_pipes);

	dbuf_state->joined_mbus = false;
	intel_dbuf_mdclk_cdclk_ratio_update(display,
					    dbuf_state->mdclk_cdclk_ratio,
					    dbuf_state->joined_mbus);
	pipe_mbus_dbox_ctl_update(display, dbuf_state);
	mbus_ctl_join_update(display, dbuf_state, INVALID_PIPE);
}

static bool skl_dbuf_is_misconfigured(struct intel_display *display)
{
	const struct intel_dbuf_state *dbuf_state =
		to_intel_dbuf_state(display->dbuf.obj.state);
	struct skl_ddb_entry entries[I915_MAX_PIPES] = {};
	struct intel_crtc *crtc;

	for_each_intel_crtc(display->drm, crtc) {
		const struct intel_crtc_state *crtc_state =
			to_intel_crtc_state(crtc->base.state);

		entries[crtc->pipe] = crtc_state->wm.skl.ddb;
	}

	for_each_intel_crtc(display->drm, crtc) {
		const struct intel_crtc_state *crtc_state =
			to_intel_crtc_state(crtc->base.state);
		u8 slices;

		slices = skl_compute_dbuf_slices(crtc, dbuf_state->active_pipes,
						 dbuf_state->joined_mbus);
		if (dbuf_state->slices[crtc->pipe] & ~slices)
			return true;

		if (skl_ddb_allocation_overlaps(&crtc_state->wm.skl.ddb, entries,
						I915_MAX_PIPES, crtc->pipe))
			return true;
	}

	return false;
}

static void skl_dbuf_sanitize(struct intel_display *display)
{
	struct intel_crtc *crtc;

	/*
	 * On TGL/RKL (at least) the BIOS likes to assign the planes
	 * to the wrong DBUF slices. This will cause an infinite loop
	 * in skl_commit_modeset_enables() as it can't find a way to
	 * transition between the old bogus DBUF layout to the new
	 * proper DBUF layout without DBUF allocation overlaps between
	 * the planes (which cannot be allowed or else the hardware
	 * may hang). If we detect a bogus DBUF layout just turn off
	 * all the planes so that skl_commit_modeset_enables() can
	 * simply ignore them.
	 */
	if (!skl_dbuf_is_misconfigured(display))
		return;

	drm_dbg_kms(display->drm, "BIOS has misprogrammed the DBUF, disabling all planes\n");

	for_each_intel_crtc(display->drm, crtc) {
		struct intel_plane *plane = to_intel_plane(crtc->base.primary);
		const struct intel_plane_state *plane_state =
			to_intel_plane_state(plane->base.state);
		struct intel_crtc_state *crtc_state =
			to_intel_crtc_state(crtc->base.state);

		if (plane_state->uapi.visible)
			intel_plane_disable_noatomic(crtc, plane);

		drm_WARN_ON(display->drm, crtc_state->active_planes != 0);

		memset(&crtc_state->wm.skl.ddb, 0, sizeof(crtc_state->wm.skl.ddb));
	}
}

static void skl_wm_sanitize(struct intel_display *display)
{
	skl_mbus_sanitize(display);
	skl_dbuf_sanitize(display);
}

void skl_wm_crtc_disable_noatomic(struct intel_crtc *crtc)
{
	struct intel_display *display = to_intel_display(crtc);
	struct intel_crtc_state *crtc_state =
		to_intel_crtc_state(crtc->base.state);
	struct intel_dbuf_state *dbuf_state =
		to_intel_dbuf_state(display->dbuf.obj.state);
	enum pipe pipe = crtc->pipe;

	if (DISPLAY_VER(display) < 9)
		return;

	dbuf_state->active_pipes &= ~BIT(pipe);

	dbuf_state->weight[pipe] = 0;
	dbuf_state->slices[pipe] = 0;

	memset(&dbuf_state->ddb[pipe], 0, sizeof(dbuf_state->ddb[pipe]));

	memset(&crtc_state->wm.skl.ddb, 0, sizeof(crtc_state->wm.skl.ddb));
}

void skl_wm_plane_disable_noatomic(struct intel_crtc *crtc,
				   struct intel_plane *plane)
{
	struct intel_display *display = to_intel_display(crtc);
	struct intel_crtc_state *crtc_state =
		to_intel_crtc_state(crtc->base.state);

	if (DISPLAY_VER(display) < 9)
		return;

	skl_ddb_entry_init(&crtc_state->wm.skl.plane_ddb[plane->id], 0, 0);
	skl_ddb_entry_init(&crtc_state->wm.skl.plane_ddb[plane->id], 0, 0);

	crtc_state->wm.skl.plane_min_ddb[plane->id] = 0;
	crtc_state->wm.skl.plane_interim_ddb[plane->id] = 0;

	memset(&crtc_state->wm.skl.raw.planes[plane->id], 0,
	       sizeof(crtc_state->wm.skl.raw.planes[plane->id]));
	memset(&crtc_state->wm.skl.optimal.planes[plane->id], 0,
	       sizeof(crtc_state->wm.skl.optimal.planes[plane->id]));
}

void intel_wm_state_verify(struct intel_atomic_state *state,
			   struct intel_crtc *crtc)
{
	struct intel_display *display = to_intel_display(state);
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	struct skl_hw_state {
		struct skl_ddb_entry ddb[I915_MAX_PLANES];
		struct skl_ddb_entry ddb_y[I915_MAX_PLANES];
		u16 min_ddb[I915_MAX_PLANES];
		u16 interim_ddb[I915_MAX_PLANES];
		struct skl_pipe_wm wm;
	} *hw;
	const struct skl_pipe_wm *sw_wm = &new_crtc_state->wm.skl.optimal;
	struct intel_plane *plane;
	u8 hw_enabled_slices;
	int level;

	if (DISPLAY_VER(display) < 9 || !new_crtc_state->hw.active)
		return;

	hw = kzalloc(sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return;

	skl_pipe_wm_get_hw_state(crtc, &hw->wm);

	skl_pipe_ddb_get_hw_state(crtc, hw->ddb, hw->ddb_y, hw->min_ddb, hw->interim_ddb);

	hw_enabled_slices = intel_enabled_dbuf_slices_mask(display);

	if (DISPLAY_VER(display) >= 11 &&
	    hw_enabled_slices != display->dbuf.enabled_slices)
		drm_err(display->drm,
			"mismatch in DBUF Slices (expected 0x%x, got 0x%x)\n",
			display->dbuf.enabled_slices,
			hw_enabled_slices);

	for_each_intel_plane_on_crtc(display->drm, crtc, plane) {
		const struct skl_ddb_entry *hw_ddb_entry, *sw_ddb_entry;
		const struct skl_wm_level *hw_wm_level, *sw_wm_level;

		/* Watermarks */
		for (level = 0; level < display->wm.num_levels; level++) {
			hw_wm_level = &hw->wm.planes[plane->id].wm[level];
			sw_wm_level = skl_plane_wm_level(sw_wm, plane->id, level);

			if (skl_wm_level_equals(hw_wm_level, sw_wm_level))
				continue;

			drm_err(display->drm,
				"[PLANE:%d:%s] mismatch in WM%d (expected e=%d b=%u l=%u, got e=%d b=%u l=%u)\n",
				plane->base.base.id, plane->base.name, level,
				sw_wm_level->enable,
				sw_wm_level->blocks,
				sw_wm_level->lines,
				hw_wm_level->enable,
				hw_wm_level->blocks,
				hw_wm_level->lines);
		}

		hw_wm_level = &hw->wm.planes[plane->id].trans_wm;
		sw_wm_level = skl_plane_trans_wm(sw_wm, plane->id);

		if (!skl_wm_level_equals(hw_wm_level, sw_wm_level)) {
			drm_err(display->drm,
				"[PLANE:%d:%s] mismatch in trans WM (expected e=%d b=%u l=%u, got e=%d b=%u l=%u)\n",
				plane->base.base.id, plane->base.name,
				sw_wm_level->enable,
				sw_wm_level->blocks,
				sw_wm_level->lines,
				hw_wm_level->enable,
				hw_wm_level->blocks,
				hw_wm_level->lines);
		}

		hw_wm_level = &hw->wm.planes[plane->id].sagv.wm0;
		sw_wm_level = &sw_wm->planes[plane->id].sagv.wm0;

		if (HAS_HW_SAGV_WM(display) &&
		    !skl_wm_level_equals(hw_wm_level, sw_wm_level)) {
			drm_err(display->drm,
				"[PLANE:%d:%s] mismatch in SAGV WM (expected e=%d b=%u l=%u, got e=%d b=%u l=%u)\n",
				plane->base.base.id, plane->base.name,
				sw_wm_level->enable,
				sw_wm_level->blocks,
				sw_wm_level->lines,
				hw_wm_level->enable,
				hw_wm_level->blocks,
				hw_wm_level->lines);
		}

		hw_wm_level = &hw->wm.planes[plane->id].sagv.trans_wm;
		sw_wm_level = &sw_wm->planes[plane->id].sagv.trans_wm;

		if (HAS_HW_SAGV_WM(display) &&
		    !skl_wm_level_equals(hw_wm_level, sw_wm_level)) {
			drm_err(display->drm,
				"[PLANE:%d:%s] mismatch in SAGV trans WM (expected e=%d b=%u l=%u, got e=%d b=%u l=%u)\n",
				plane->base.base.id, plane->base.name,
				sw_wm_level->enable,
				sw_wm_level->blocks,
				sw_wm_level->lines,
				hw_wm_level->enable,
				hw_wm_level->blocks,
				hw_wm_level->lines);
		}

		/* DDB */
		hw_ddb_entry = &hw->ddb[PLANE_CURSOR];
		sw_ddb_entry = &new_crtc_state->wm.skl.plane_ddb[PLANE_CURSOR];

		if (!skl_ddb_entry_equal(hw_ddb_entry, sw_ddb_entry)) {
			drm_err(display->drm,
				"[PLANE:%d:%s] mismatch in DDB (expected (%u,%u), found (%u,%u))\n",
				plane->base.base.id, plane->base.name,
				sw_ddb_entry->start, sw_ddb_entry->end,
				hw_ddb_entry->start, hw_ddb_entry->end);
		}
	}

	kfree(hw);
}

static const struct intel_wm_funcs skl_wm_funcs = {
	.compute_global_watermarks = skl_compute_wm,
	.get_hw_state = skl_wm_get_hw_state,
	.sanitize = skl_wm_sanitize,
};

void skl_wm_init(struct intel_display *display)
{
	intel_sagv_init(display);

	skl_setup_wm_latency(display);

	display->funcs.wm = &skl_wm_funcs;
}

static int skl_watermark_ipc_status_show(struct seq_file *m, void *data)
{
	struct intel_display *display = m->private;

	seq_printf(m, "Isochronous Priority Control: %s\n",
		   str_yes_no(skl_watermark_ipc_enabled(display)));
	return 0;
}

static int skl_watermark_ipc_status_open(struct inode *inode, struct file *file)
{
	struct intel_display *display = inode->i_private;

	return single_open(file, skl_watermark_ipc_status_show, display);
}

static ssize_t skl_watermark_ipc_status_write(struct file *file,
					      const char __user *ubuf,
					      size_t len, loff_t *offp)
{
	struct seq_file *m = file->private_data;
	struct intel_display *display = m->private;
	bool enable;
	int ret;

	ret = kstrtobool_from_user(ubuf, len, &enable);
	if (ret < 0)
		return ret;

	with_intel_display_rpm(display) {
		if (!skl_watermark_ipc_enabled(display) && enable)
			drm_info(display->drm,
				 "Enabling IPC: WM will be proper only after next commit\n");
		display->wm.ipc_enabled = enable;
		skl_watermark_ipc_update(display);
	}

	return len;
}

static const struct file_operations skl_watermark_ipc_status_fops = {
	.owner = THIS_MODULE,
	.open = skl_watermark_ipc_status_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = skl_watermark_ipc_status_write
};

static int intel_sagv_status_show(struct seq_file *m, void *unused)
{
	struct intel_display *display = m->private;
	static const char * const sagv_status[] = {
		[I915_SAGV_UNKNOWN] = "unknown",
		[I915_SAGV_DISABLED] = "disabled",
		[I915_SAGV_ENABLED] = "enabled",
		[I915_SAGV_NOT_CONTROLLED] = "not controlled",
	};

	seq_printf(m, "SAGV available: %s\n", str_yes_no(intel_has_sagv(display)));
	seq_printf(m, "SAGV modparam: %s\n",
		   str_enabled_disabled(display->params.enable_sagv));
	seq_printf(m, "SAGV status: %s\n", sagv_status[display->sagv.status]);
	seq_printf(m, "SAGV block time: %d usec\n", display->sagv.block_time_us);

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(intel_sagv_status);

void skl_watermark_debugfs_register(struct intel_display *display)
{
	struct drm_minor *minor = display->drm->primary;

	if (HAS_IPC(display))
		debugfs_create_file("i915_ipc_status", 0644, minor->debugfs_root,
				    display, &skl_watermark_ipc_status_fops);

	if (HAS_SAGV(display))
		debugfs_create_file("i915_sagv_status", 0444, minor->debugfs_root,
				    display, &intel_sagv_status_fops);
}

unsigned int skl_watermark_max_latency(struct intel_display *display, int initial_wm_level)
{
	int level;

	for (level = display->wm.num_levels - 1; level >= initial_wm_level; level--) {
		unsigned int latency = skl_wm_latency(display, level, NULL);

		if (latency)
			return latency;
	}

	return 0;
}
