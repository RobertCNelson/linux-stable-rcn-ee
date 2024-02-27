/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2022 Microchip Technology Inc. and its subsidiaries
 * Author: Shravan Chippa <shavan.chippa@microchip.com>
 *
 */

#ifndef __MCHP_COMMAN_H__
#define __MCHP_COMMAN_H__

#include <linux/bitops.h>
#include <linux/io.h>
#include <media/v4l2-subdev.h>

#define MVC_PAD_SINK			0
#define MVC_PAD_SOURCE			1

/**
 * struct mvc_device - Microchip Video Capture device structure
 * @subdev: V4L2 subdevice
 * @dev: (OF) device
 * @iomem: device I/O register space remapped to kernel virtual memory
 * @clk: video core clock
 */
struct mvc_device {
	struct v4l2_subdev subdev;
	struct device *dev;
	void __iomem *iomem;
	struct clk *clk;
};

/**
 * struct mvideo_format - Microchip video format description
 * @vf_code: AXI4 video format code
 * @width: AXI4 format width in bits per component
 * @pattern: CFA pattern for Mono/Sensor formats
 * @code: media bus format code
 * @bpl_factor: Bytes per line factor
 * @bpp: bits per pixel
 * @fourcc: V4L2 pixel format FCC identifier
 * @num_planes: number of planes w.r.t. color format
 * @buffers: number of buffers per format
 * @hsub: Horizontal sampling factor of Chroma
 * @vsub: Vertical sampling factor of Chroma
 */
struct mvideo_format {
	unsigned int vf_code;
	unsigned int width;
	const char *pattern;
	unsigned int code;
	unsigned int bpp;
	u32 fourcc;
	u8 num_planes;
	u8 buffers;
	u8 hsub;
	u8 vsub;
};

const struct mvideo_format *mvc_get_format_by_code(unsigned int code);
const struct mvideo_format *mvc_get_format_by_fourcc(u32 fourcc);
const struct mvideo_format *mvc_of_get_format(struct device_node *node);
void mvc_set_format_size(struct v4l2_mbus_framefmt *format,
			 const struct v4l2_subdev_format *fmt);
int mvc_enum_mbus_code(struct v4l2_subdev *subdev,
		       struct v4l2_subdev_state *sd_state,
		       struct v4l2_subdev_mbus_code_enum *code);
int mvc_enum_frame_size(struct v4l2_subdev *subdev,
			struct v4l2_subdev_state *sd_state,
			struct v4l2_subdev_frame_size_enum *fse);

int mvc_init_resources(struct mvc_device *mvc);

#endif /* __MICROCHIP_COMMAN_H__ */

