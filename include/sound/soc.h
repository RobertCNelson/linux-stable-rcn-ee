/* SPDX-License-Identifier: GPL-2.0
 *
 * linux/sound/soc.h -- ALSA SoC Layer
 *
 * Author:	Liam Girdwood
 * Created:	Aug 11th 2005
 * Copyright:	Wolfson Microelectronics. PLC.
 */

#ifndef __LINUX_SND_SOC_H
#define __LINUX_SND_SOC_H

#include <linux/args.h>
#include <linux/array_size.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/lockdep.h>
#include <linux/log2.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <sound/ac97_codec.h>
#include <sound/compress_driver.h>
#include <sound/control.h>
#include <sound/core.h>
#include <sound/pcm.h>

struct module;
struct platform_device;

/* For the current users of sound/soc.h to avoid build issues */
#include <linux/platform_device.h>
#include <linux/regmap.h>

/*
 * Convenience kcontrol builders
 */
#define SOC_DOUBLE_S_VALUE(xreg, shift_left, shift_right, xmin, xmax, xsign_bit, \
			   xinvert, xautodisable) \
	((unsigned long)&(struct soc_mixer_control) \
	{.reg = xreg, .rreg = xreg, .shift = shift_left, \
	.rshift = shift_right, .min = xmin, .max = xmax, \
	.sign_bit = xsign_bit, .invert = xinvert, .autodisable = xautodisable})
#define SOC_DOUBLE_VALUE(xreg, shift_left, shift_right, xmin, xmax, xinvert, xautodisable) \
	SOC_DOUBLE_S_VALUE(xreg, shift_left, shift_right, xmin, xmax, 0, xinvert, \
			   xautodisable)
#define SOC_SINGLE_VALUE(xreg, xshift, xmin, xmax, xinvert, xautodisable) \
	SOC_DOUBLE_VALUE(xreg, xshift, xshift, xmin, xmax, xinvert, xautodisable)
#define SOC_DOUBLE_R_S_VALUE(xlreg, xrreg, xshift, xmin, xmax, xsign_bit, xinvert) \
	((unsigned long)&(struct soc_mixer_control) \
	{.reg = xlreg, .rreg = xrreg, .shift = xshift, .rshift = xshift, \
	.max = xmax, .min = xmin, .sign_bit = xsign_bit, \
	.invert = xinvert})
#define SOC_DOUBLE_R_VALUE(xlreg, xrreg, xshift, xmin, xmax, xinvert) \
	SOC_DOUBLE_R_S_VALUE(xlreg, xrreg, xshift, xmin, xmax, 0, xinvert)

#define SOC_SINGLE(xname, reg, shift, max, invert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,\
	.put = snd_soc_put_volsw, \
	.private_value = SOC_SINGLE_VALUE(reg, shift, 0, max, invert, 0) }
#define SOC_SINGLE_RANGE(xname, xreg, xshift, xmin, xmax, xinvert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw, \
	.put = snd_soc_put_volsw, \
	.private_value = SOC_SINGLE_VALUE(xreg, xshift, xmin, xmax, xinvert, 0) }
#define SOC_SINGLE_TLV(xname, reg, shift, max, invert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,\
	.put = snd_soc_put_volsw, \
	.private_value = SOC_SINGLE_VALUE(reg, shift, 0, max, invert, 0) }
#define SOC_SINGLE_SX_TLV(xname, xreg, xshift, xmin, xmax, tlv_array) \
{       .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
	SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.tlv.p  = (tlv_array),\
	.info = snd_soc_info_volsw_sx, \
	.get = snd_soc_get_volsw_sx,\
	.put = snd_soc_put_volsw_sx, \
	.private_value = SOC_SINGLE_VALUE(xreg, xshift, xmin, xmax, 0, 0) }
#define SOC_SINGLE_RANGE_TLV(xname, xreg, xshift, xmin, xmax, xinvert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_get_volsw, .put = snd_soc_put_volsw, \
	.private_value = SOC_SINGLE_VALUE(xreg, xshift, xmin, xmax, xinvert, 0) }
#define SOC_DOUBLE(xname, reg, shift_left, shift_right, max, invert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw, \
	.put = snd_soc_put_volsw, \
	.private_value = SOC_DOUBLE_VALUE(reg, shift_left, shift_right, \
					  0, max, invert, 0) }
#define SOC_DOUBLE_STS(xname, reg, shift_left, shift_right, max, invert) \
{									\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),		\
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,		\
	.access = SNDRV_CTL_ELEM_ACCESS_READ |				\
		SNDRV_CTL_ELEM_ACCESS_VOLATILE,				\
	.private_value = SOC_DOUBLE_VALUE(reg, shift_left, shift_right,	\
					  0, max, invert, 0) }
#define SOC_DOUBLE_R(xname, reg_left, reg_right, xshift, xmax, xinvert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_get_volsw, .put = snd_soc_put_volsw, \
	.private_value = SOC_DOUBLE_R_VALUE(reg_left, reg_right, xshift, \
					    0, xmax, xinvert) }
#define SOC_DOUBLE_R_RANGE(xname, reg_left, reg_right, xshift, xmin, \
			   xmax, xinvert)		\
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.info = snd_soc_info_volsw, \
	.get = snd_soc_get_volsw, .put = snd_soc_put_volsw, \
	.private_value = SOC_DOUBLE_R_VALUE(reg_left, reg_right, \
					    xshift, xmin, xmax, xinvert) }
#define SOC_DOUBLE_TLV(xname, reg, shift_left, shift_right, max, invert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw, \
	.put = snd_soc_put_volsw, \
	.private_value = SOC_DOUBLE_VALUE(reg, shift_left, shift_right, \
					  0, max, invert, 0) }
#define SOC_DOUBLE_SX_TLV(xname, xreg, shift_left, shift_right, xmin, xmax, tlv_array) \
{       .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
	SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.tlv.p  = (tlv_array), \
	.info = snd_soc_info_volsw_sx, \
	.get = snd_soc_get_volsw_sx, \
	.put = snd_soc_put_volsw_sx, \
	.private_value = SOC_DOUBLE_VALUE(xreg, shift_left, shift_right, \
					  xmin, xmax, 0, 0) }
#define SOC_DOUBLE_RANGE_TLV(xname, xreg, xshift_left, xshift_right, xmin, xmax, \
			     xinvert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		  SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_get_volsw, .put = snd_soc_put_volsw, \
	.private_value = SOC_DOUBLE_VALUE(xreg, xshift_left, xshift_right, \
					  xmin, xmax, xinvert, 0) }
#define SOC_DOUBLE_R_TLV(xname, reg_left, reg_right, xshift, xmax, xinvert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_get_volsw, .put = snd_soc_put_volsw, \
	.private_value = SOC_DOUBLE_R_VALUE(reg_left, reg_right, xshift, \
					    0, xmax, xinvert) }
#define SOC_DOUBLE_R_RANGE_TLV(xname, reg_left, reg_right, xshift, xmin, \
			       xmax, xinvert, tlv_array)		\
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_get_volsw, .put = snd_soc_put_volsw, \
	.private_value = SOC_DOUBLE_R_VALUE(reg_left, reg_right, \
					    xshift, xmin, xmax, xinvert) }
#define SOC_DOUBLE_R_SX_TLV(xname, xreg, xrreg, xshift, xmin, xmax, tlv_array) \
{       .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
	SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.tlv.p  = (tlv_array), \
	.info = snd_soc_info_volsw_sx, \
	.get = snd_soc_get_volsw_sx, \
	.put = snd_soc_put_volsw_sx, \
	.private_value = SOC_DOUBLE_R_VALUE(xreg, xrreg, xshift, xmin, xmax, 0) }
#define SOC_DOUBLE_R_S_TLV(xname, reg_left, reg_right, xshift, xmin, xmax, xsign_bit, xinvert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_get_volsw, .put = snd_soc_put_volsw, \
	.private_value = SOC_DOUBLE_R_S_VALUE(reg_left, reg_right, xshift, \
					    xmin, xmax, xsign_bit, xinvert) }
#define SOC_SINGLE_S_TLV(xname, xreg, xshift, xmin, xmax, xsign_bit, xinvert, tlv_array) \
	SOC_DOUBLE_R_S_TLV(xname, xreg, xreg, xshift, xmin, xmax, xsign_bit, xinvert, tlv_array)
#define SOC_SINGLE_S8_TLV(xname, xreg, xmin, xmax, tlv_array) \
{	.iface  = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
		  SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.tlv.p  = (tlv_array), \
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,\
	.put = snd_soc_put_volsw, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
	{.reg = xreg, .rreg = xreg,  \
	 .min = xmin, .max = xmax, \
	.sign_bit = 7,} }
#define SOC_DOUBLE_S8_TLV(xname, xreg, xmin, xmax, tlv_array) \
{	.iface  = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
		  SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.tlv.p  = (tlv_array), \
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,\
	.put = snd_soc_put_volsw, \
	.private_value = SOC_DOUBLE_S_VALUE(xreg, 0, 8, xmin, xmax, 7, 0, 0) }
#define SOC_ENUM_DOUBLE(xreg, xshift_l, xshift_r, xitems, xtexts) \
{	.reg = xreg, .shift_l = xshift_l, .shift_r = xshift_r, \
	.items = xitems, .texts = xtexts, \
	.mask = xitems ? roundup_pow_of_two(xitems) - 1 : 0}
#define SOC_ENUM_SINGLE(xreg, xshift, xitems, xtexts) \
	SOC_ENUM_DOUBLE(xreg, xshift, xshift, xitems, xtexts)
#define SOC_ENUM_SINGLE_EXT(xitems, xtexts) \
{	.items = xitems, .texts = xtexts }
#define SOC_VALUE_ENUM_DOUBLE(xreg, xshift_l, xshift_r, xmask, xitems, xtexts, xvalues) \
{	.reg = xreg, .shift_l = xshift_l, .shift_r = xshift_r, \
	.mask = xmask, .items = xitems, .texts = xtexts, .values = xvalues}
#define SOC_VALUE_ENUM_SINGLE(xreg, xshift, xmask, xitems, xtexts, xvalues) \
	SOC_VALUE_ENUM_DOUBLE(xreg, xshift, xshift, xmask, xitems, xtexts, xvalues)
#define SOC_VALUE_ENUM_SINGLE_AUTODISABLE(xreg, xshift, xmask, xitems, xtexts, xvalues) \
{	.reg = xreg, .shift_l = xshift, .shift_r = xshift, \
	.mask = xmask, .items = xitems, .texts = xtexts, \
	.values = xvalues, .autodisable = 1}
#define SOC_ENUM_SINGLE_VIRT(xitems, xtexts) \
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, xitems, xtexts)
#define SOC_ENUM(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,\
	.info = snd_soc_info_enum_double, \
	.get = snd_soc_get_enum_double, .put = snd_soc_put_enum_double, \
	.private_value = (unsigned long)&xenum }
#define SOC_SINGLE_EXT(xname, xreg, xshift, xmax, xinvert,\
	 xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_volsw, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = SOC_SINGLE_VALUE(xreg, xshift, 0, xmax, xinvert, 0) }
#define SOC_DOUBLE_EXT(xname, reg, shift_left, shift_right, max, invert,\
	 xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.info = snd_soc_info_volsw, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = \
		SOC_DOUBLE_VALUE(reg, shift_left, shift_right, 0, max, invert, 0) }
#define SOC_DOUBLE_R_EXT(xname, reg_left, reg_right, xshift, xmax, xinvert,\
	 xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_volsw, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = SOC_DOUBLE_R_VALUE(reg_left, reg_right, xshift, \
					    0, xmax, xinvert) }
#define SOC_SINGLE_EXT_TLV(xname, xreg, xshift, xmax, xinvert,\
	 xhandler_get, xhandler_put, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = SOC_SINGLE_VALUE(xreg, xshift, 0, xmax, xinvert, 0) }
#define SOC_SINGLE_RANGE_EXT_TLV(xname, xreg, xshift, xmin, xmax, xinvert, \
				 xhandler_get, xhandler_put, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = SOC_SINGLE_VALUE(xreg, xshift, xmin, xmax, xinvert, 0) }
#define SOC_DOUBLE_EXT_TLV(xname, xreg, shift_left, shift_right, xmax, xinvert,\
	 xhandler_get, xhandler_put, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
		 SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = SOC_DOUBLE_VALUE(xreg, shift_left, shift_right, \
					  0, xmax, xinvert, 0) }
#define SOC_DOUBLE_R_EXT_TLV(xname, reg_left, reg_right, xshift, xmax, xinvert,\
	 xhandler_get, xhandler_put, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
		 SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = SOC_DOUBLE_R_VALUE(reg_left, reg_right, xshift, \
					    0, xmax, xinvert) }
#define SOC_DOUBLE_R_S_EXT_TLV(xname, reg_left, reg_right, xshift, xmin, xmax, \
			       xsign_bit, xinvert, xhandler_get, xhandler_put, \
			       tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
		  SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = SOC_DOUBLE_R_S_VALUE(reg_left, reg_right, xshift, \
					      xmin, xmax, xsign_bit, xinvert) }
#define SOC_SINGLE_S_EXT_TLV(xname, xreg, xshift, xmin, xmax, \
			     xsign_bit, xinvert, xhandler_get, xhandler_put, \
			     tlv_array) \
	SOC_DOUBLE_R_S_EXT_TLV(xname, xreg, xreg, xshift, xmin, xmax, \
			       xsign_bit, xinvert, xhandler_get, xhandler_put, \
			       tlv_array)
#define SOC_SINGLE_BOOL_EXT(xname, xdata, xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_bool_ext, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = xdata }
#define SOC_ENUM_EXT(xname, xenum, xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_enum_double, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = (unsigned long)&xenum }
#define SOC_VALUE_ENUM_EXT(xname, xenum, xhandler_get, xhandler_put) \
	SOC_ENUM_EXT(xname, xenum, xhandler_get, xhandler_put)

#define SND_SOC_BYTES(xname, xbase, xregs)		      \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,   \
	.info = snd_soc_bytes_info, .get = snd_soc_bytes_get, \
	.put = snd_soc_bytes_put, .private_value =	      \
		((unsigned long)&(struct soc_bytes)           \
		{.base = xbase, .num_regs = xregs }) }
#define SND_SOC_BYTES_E(xname, xbase, xregs, xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_bytes_info, .get = xhandler_get, \
	.put = xhandler_put, .private_value = \
		((unsigned long)&(struct soc_bytes) \
		{.base = xbase, .num_regs = xregs }) }

#define SND_SOC_BYTES_MASK(xname, xbase, xregs, xmask)	      \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,   \
	.info = snd_soc_bytes_info, .get = snd_soc_bytes_get, \
	.put = snd_soc_bytes_put, .private_value =	      \
		((unsigned long)&(struct soc_bytes)           \
		{.base = xbase, .num_regs = xregs,	      \
		 .mask = xmask }) }

/*
 * SND_SOC_BYTES_EXT is deprecated, please USE SND_SOC_BYTES_TLV instead
 */
#define SND_SOC_BYTES_EXT(xname, xcount, xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_bytes_info_ext, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = (unsigned long)&(struct soc_bytes_ext) \
		{.max = xcount} }
#define SND_SOC_BYTES_TLV(xname, xcount, xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READWRITE | \
		  SNDRV_CTL_ELEM_ACCESS_TLV_CALLBACK, \
	.tlv.c = (snd_soc_bytes_tlv_callback), \
	.info = snd_soc_bytes_info_ext, \
	.private_value = (unsigned long)&(struct soc_bytes_ext) \
		{.max = xcount, .get = xhandler_get, .put = xhandler_put, } }
#define SOC_SINGLE_XR_SX(xname, xregbase, xregcount, xnbits, \
		xmin, xmax, xinvert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_xr_sx, .get = snd_soc_get_xr_sx, \
	.put = snd_soc_put_xr_sx, \
	.private_value = (unsigned long)&(struct soc_mreg_control) \
		{.regbase = xregbase, .regcount = xregcount, .nbits = xnbits, \
		.invert = xinvert, .min = xmin, .max = xmax} }

#define SOC_SINGLE_STROBE(xname, xreg, xshift, xinvert) \
	SOC_SINGLE_EXT(xname, xreg, xshift, 1, xinvert, \
		snd_soc_get_strobe, snd_soc_put_strobe)

/*
 * Simplified versions of above macros, declaring a struct and calculating
 * ARRAY_SIZE internally
 */
#define SOC_ENUM_DOUBLE_DECL(name, xreg, xshift_l, xshift_r, xtexts) \
	const struct soc_enum name = SOC_ENUM_DOUBLE(xreg, xshift_l, xshift_r, \
						ARRAY_SIZE(xtexts), xtexts)
#define SOC_ENUM_SINGLE_DECL(name, xreg, xshift, xtexts) \
	SOC_ENUM_DOUBLE_DECL(name, xreg, xshift, xshift, xtexts)
#define SOC_ENUM_SINGLE_EXT_DECL(name, xtexts) \
	const struct soc_enum name = SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(xtexts), xtexts)
#define SOC_VALUE_ENUM_DOUBLE_DECL(name, xreg, xshift_l, xshift_r, xmask, xtexts, xvalues) \
	const struct soc_enum name = SOC_VALUE_ENUM_DOUBLE(xreg, xshift_l, xshift_r, xmask, \
							ARRAY_SIZE(xtexts), xtexts, xvalues)
#define SOC_VALUE_ENUM_SINGLE_DECL(name, xreg, xshift, xmask, xtexts, xvalues) \
	SOC_VALUE_ENUM_DOUBLE_DECL(name, xreg, xshift, xshift, xmask, xtexts, xvalues)

#define SOC_VALUE_ENUM_SINGLE_AUTODISABLE_DECL(name, xreg, xshift, xmask, xtexts, xvalues) \
	const struct soc_enum name = SOC_VALUE_ENUM_SINGLE_AUTODISABLE(xreg, \
		xshift, xmask, ARRAY_SIZE(xtexts), xtexts, xvalues)

#define SOC_ENUM_SINGLE_VIRT_DECL(name, xtexts) \
	const struct soc_enum name = SOC_ENUM_SINGLE_VIRT(ARRAY_SIZE(xtexts), xtexts)

struct snd_soc_card;
struct snd_soc_pcm_runtime;
struct snd_soc_dai;
struct snd_soc_dai_driver;
struct snd_soc_dai_link;
struct snd_soc_component;
struct snd_soc_component_driver;
struct snd_soc_jack;
struct snd_soc_jack_pin;

#include <sound/soc-dapm.h>
#include <sound/soc-dpcm.h>
#include <sound/soc-topology.h>

enum snd_soc_pcm_subclass {
	SND_SOC_PCM_CLASS_PCM	= 0,
	SND_SOC_PCM_CLASS_BE	= 1,
};

int snd_soc_register_card(struct snd_soc_card *card);
void snd_soc_unregister_card(struct snd_soc_card *card);
int devm_snd_soc_register_card(struct device *dev, struct snd_soc_card *card);
int devm_snd_soc_register_deferrable_card(struct device *dev, struct snd_soc_card *card);
#ifdef CONFIG_PM_SLEEP
int snd_soc_suspend(struct device *dev);
int snd_soc_resume(struct device *dev);
#else
static inline int snd_soc_suspend(struct device *dev)
{
	return 0;
}

static inline int snd_soc_resume(struct device *dev)
{
	return 0;
}
#endif
int snd_soc_poweroff(struct device *dev);
int snd_soc_component_initialize(struct snd_soc_component *component,
				 const struct snd_soc_component_driver *driver,
				 struct device *dev);
int snd_soc_add_component(struct snd_soc_component *component,
			  struct snd_soc_dai_driver *dai_drv,
			  int num_dai);
int snd_soc_register_component(struct device *dev,
			 const struct snd_soc_component_driver *component_driver,
			 struct snd_soc_dai_driver *dai_drv, int num_dai);
int devm_snd_soc_register_component(struct device *dev,
			 const struct snd_soc_component_driver *component_driver,
			 struct snd_soc_dai_driver *dai_drv, int num_dai);
#define snd_soc_unregister_component(dev) snd_soc_unregister_component_by_driver(dev, NULL)
void snd_soc_unregister_component_by_driver(struct device *dev,
			 const struct snd_soc_component_driver *component_driver);
struct snd_soc_component *snd_soc_lookup_component_nolocked(struct device *dev,
							    const char *driver_name);
struct snd_soc_component *snd_soc_lookup_component(struct device *dev,
						   const char *driver_name);

int soc_new_pcm(struct snd_soc_pcm_runtime *rtd);
#ifdef CONFIG_SND_SOC_COMPRESS
int snd_soc_new_compress(struct snd_soc_pcm_runtime *rtd);
#else
static inline int snd_soc_new_compress(struct snd_soc_pcm_runtime *rtd)
{
	return 0;
}
#endif

struct snd_soc_pcm_runtime *snd_soc_get_pcm_runtime(struct snd_soc_card *card,
				struct snd_soc_dai_link *dai_link);

bool snd_soc_runtime_ignore_pmdown_time(struct snd_soc_pcm_runtime *rtd);

void snd_soc_runtime_action(struct snd_soc_pcm_runtime *rtd,
			    int stream, int action);
static inline void snd_soc_runtime_activate(struct snd_soc_pcm_runtime *rtd,
				     int stream)
{
	snd_soc_runtime_action(rtd, stream, 1);
}
static inline void snd_soc_runtime_deactivate(struct snd_soc_pcm_runtime *rtd,
				       int stream)
{
	snd_soc_runtime_action(rtd, stream, -1);
}

int snd_soc_runtime_calc_hw(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hardware *hw, int stream);

int snd_soc_runtime_set_dai_fmt(struct snd_soc_pcm_runtime *rtd,
	unsigned int dai_fmt);

/* Utility functions to get clock rates from various things */
int snd_soc_calc_frame_size(int sample_size, int channels, int tdm_slots);
int snd_soc_params_to_frame_size(const struct snd_pcm_hw_params *params);
int snd_soc_calc_bclk(int fs, int sample_size, int channels, int tdm_slots);
int snd_soc_params_to_bclk(const struct snd_pcm_hw_params *parms);
int snd_soc_tdm_params_to_bclk(const struct snd_pcm_hw_params *params,
			       int tdm_width, int tdm_slots, int slot_multiple);
int snd_soc_ret(const struct device *dev, int ret, const char *fmt, ...);

/* set runtime hw params */
static inline int snd_soc_set_runtime_hwparams(struct snd_pcm_substream *substream,
					       const struct snd_pcm_hardware *hw)
{
	substream->runtime->hw = *hw;

	return 0;
}

struct snd_ac97 *snd_soc_alloc_ac97_component(struct snd_soc_component *component);
struct snd_ac97 *snd_soc_new_ac97_component(struct snd_soc_component *component,
	unsigned int id, unsigned int id_mask);
void snd_soc_free_ac97_component(struct snd_ac97 *ac97);

#ifdef CONFIG_SND_SOC_AC97_BUS
int snd_soc_set_ac97_ops(struct snd_ac97_bus_ops *ops);
int snd_soc_set_ac97_ops_of_reset(struct snd_ac97_bus_ops *ops,
		struct platform_device *pdev);

extern struct snd_ac97_bus_ops *soc_ac97_ops;
#else
static inline int snd_soc_set_ac97_ops_of_reset(struct snd_ac97_bus_ops *ops,
	struct platform_device *pdev)
{
	return 0;
}

static inline int snd_soc_set_ac97_ops(struct snd_ac97_bus_ops *ops)
{
	return 0;
}
#endif

/*
 *Controls
 */
struct snd_kcontrol *snd_soc_cnew(const struct snd_kcontrol_new *_template,
				  void *data, const char *long_name,
				  const char *prefix);
int snd_soc_add_component_controls(struct snd_soc_component *component,
	const struct snd_kcontrol_new *controls, unsigned int num_controls);
int snd_soc_add_card_controls(struct snd_soc_card *soc_card,
	const struct snd_kcontrol_new *controls, int num_controls);
int snd_soc_add_dai_controls(struct snd_soc_dai *dai,
	const struct snd_kcontrol_new *controls, int num_controls);
int snd_soc_info_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo);
int snd_soc_get_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_put_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_info_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo);
int snd_soc_info_volsw_sx(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_info *uinfo);
#define snd_soc_info_bool_ext		snd_ctl_boolean_mono_info
int snd_soc_get_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_put_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
#define snd_soc_get_volsw_2r snd_soc_get_volsw
#define snd_soc_put_volsw_2r snd_soc_put_volsw
int snd_soc_get_volsw_sx(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_put_volsw_sx(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_limit_volume(struct snd_soc_card *card,
	const char *name, int max);
int snd_soc_bytes_info(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_info *uinfo);
int snd_soc_bytes_get(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol);
int snd_soc_bytes_put(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol);
int snd_soc_bytes_info_ext(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *ucontrol);
int snd_soc_bytes_tlv_callback(struct snd_kcontrol *kcontrol, int op_flag,
	unsigned int size, unsigned int __user *tlv);
int snd_soc_info_xr_sx(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo);
int snd_soc_get_xr_sx(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_put_xr_sx(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_get_strobe(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_put_strobe(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

enum snd_soc_trigger_order {
						/* start			stop		     */
	SND_SOC_TRIGGER_ORDER_DEFAULT	= 0,	/* Link->Component->DAI		DAI->Component->Link */
	SND_SOC_TRIGGER_ORDER_LDC,		/* Link->DAI->Component		Component->DAI->Link */

	SND_SOC_TRIGGER_ORDER_MAX,
};

/* SoC PCM stream information */
struct snd_soc_pcm_stream {
	const char *stream_name;
	u64 formats;			/* SNDRV_PCM_FMTBIT_* */
	u32 subformats;			/* for S32_LE format, SNDRV_PCM_SUBFMTBIT_* */
	unsigned int rates;		/* SNDRV_PCM_RATE_* */
	unsigned int rate_min;		/* min rate */
	unsigned int rate_max;		/* max rate */
	unsigned int channels_min;	/* min channels */
	unsigned int channels_max;	/* max channels */
	unsigned int sig_bits;		/* number of bits of content */
};

/* SoC audio ops */
struct snd_soc_ops {
	int (*startup)(struct snd_pcm_substream *);
	void (*shutdown)(struct snd_pcm_substream *);
	int (*hw_params)(struct snd_pcm_substream *, struct snd_pcm_hw_params *);
	int (*hw_free)(struct snd_pcm_substream *);
	int (*prepare)(struct snd_pcm_substream *);
	int (*trigger)(struct snd_pcm_substream *, int);
};

struct snd_soc_compr_ops {
	int (*startup)(struct snd_compr_stream *);
	void (*shutdown)(struct snd_compr_stream *);
	int (*set_params)(struct snd_compr_stream *);
};

struct snd_soc_component*
snd_soc_rtdcom_lookup(struct snd_soc_pcm_runtime *rtd,
		       const char *driver_name);

struct snd_soc_dai_link_component {
	const char *name;
	struct device_node *of_node;
	const char *dai_name;
	const struct of_phandle_args *dai_args;

	/*
	 * Extra format = SND_SOC_DAIFMT_Bx_Fx
	 *
	 * [Note] it is Bx_Fx base, not CBx_CFx
	 *
	 * It will be used with dai_link->dai_fmt
	 * see
	 *	snd_soc_runtime_set_dai_fmt()
	 */
	unsigned int ext_fmt;
};

/*
 * [dai_link->ch_maps Image sample]
 *
 *-------------------------
 * CPU0 <---> Codec0
 *
 * ch-map[0].cpu = 0	ch-map[0].codec = 0
 *
 *-------------------------
 * CPU0 <---> Codec0
 * CPU1 <---> Codec1
 * CPU2 <---> Codec2
 *
 * ch-map[0].cpu = 0	ch-map[0].codec = 0
 * ch-map[1].cpu = 1	ch-map[1].codec = 1
 * ch-map[2].cpu = 2	ch-map[2].codec = 2
 *
 *-------------------------
 * CPU0 <---> Codec0
 * CPU1 <-+-> Codec1
 * CPU2 <-/
 *
 * ch-map[0].cpu = 0	ch-map[0].codec = 0
 * ch-map[1].cpu = 1	ch-map[1].codec = 1
 * ch-map[2].cpu = 2	ch-map[2].codec = 1
 *
 *-------------------------
 * CPU0 <---> Codec0
 * CPU1 <-+-> Codec1
 *	  \-> Codec2
 *
 * ch-map[0].cpu = 0	ch-map[0].codec = 0
 * ch-map[1].cpu = 1	ch-map[1].codec = 1
 * ch-map[2].cpu = 1	ch-map[2].codec = 2
 *
 */
struct snd_soc_dai_link_ch_map {
	unsigned int cpu;
	unsigned int codec;
	unsigned int ch_mask;
};

struct snd_soc_dai_link {
	/* config - must be set by machine driver */
	const char *name;			/* Codec name */
	const char *stream_name;		/* Stream name */

	/*
	 * You MAY specify the link's CPU-side device, either by device name,
	 * or by DT/OF node, but not both. If this information is omitted,
	 * the CPU-side DAI is matched using .cpu_dai_name only, which hence
	 * must be globally unique. These fields are currently typically used
	 * only for codec to codec links, or systems using device tree.
	 */
	/*
	 * You MAY specify the DAI name of the CPU DAI. If this information is
	 * omitted, the CPU-side DAI is matched using .cpu_name/.cpu_of_node
	 * only, which only works well when that device exposes a single DAI.
	 */
	struct snd_soc_dai_link_component *cpus;
	unsigned int num_cpus;

	/*
	 * You MUST specify the link's codec, either by device name, or by
	 * DT/OF node, but not both.
	 */
	/* You MUST specify the DAI name within the codec */
	struct snd_soc_dai_link_component *codecs;
	unsigned int num_codecs;

	/* num_ch_maps = max(num_cpu, num_codecs) */
	struct snd_soc_dai_link_ch_map *ch_maps;

	/*
	 * You MAY specify the link's platform/PCM/DMA driver, either by
	 * device name, or by DT/OF node, but not both. Some forms of link
	 * do not need a platform. In such case, platforms are not mandatory.
	 */
	struct snd_soc_dai_link_component *platforms;
	unsigned int num_platforms;

	int id;	/* optional ID for machine driver link identification */

	/*
	 * for Codec2Codec
	 */
	const struct snd_soc_pcm_stream *c2c_params;
	unsigned int num_c2c_params;

	unsigned int dai_fmt;           /* format to set on init */

	enum snd_soc_dpcm_trigger trigger[2]; /* trigger type for DPCM */

	/* codec/machine specific init - e.g. add machine controls */
	int (*init)(struct snd_soc_pcm_runtime *rtd);

	/* codec/machine specific exit - dual of init() */
	void (*exit)(struct snd_soc_pcm_runtime *rtd);

	/* optional hw_params re-writing for BE and FE sync */
	int (*be_hw_params_fixup)(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params);

	/* machine stream operations */
	const struct snd_soc_ops *ops;
	const struct snd_soc_compr_ops *compr_ops;

	/*
	 * soc_pcm_trigger() start/stop sequence.
	 * see also
	 *	snd_soc_component_driver
	 *	soc_pcm_trigger()
	 */
	enum snd_soc_trigger_order trigger_start;
	enum snd_soc_trigger_order trigger_stop;

	/* Mark this pcm with non atomic ops */
	unsigned int nonatomic:1;

	/* For unidirectional dai links */
	unsigned int playback_only:1;
	unsigned int capture_only:1;

	/* Keep DAI active over suspend */
	unsigned int ignore_suspend:1;

	/* Symmetry requirements */
	unsigned int symmetric_rate:1;
	unsigned int symmetric_channels:1;
	unsigned int symmetric_sample_bits:1;

	/* Do not create a PCM for this DAI link (Backend link) */
	unsigned int no_pcm:1;

	/* This DAI link can route to other DAI links at runtime (Frontend)*/
	unsigned int dynamic:1;

	/* DPCM used FE & BE merged format */
	unsigned int dpcm_merged_format:1;
	/* DPCM used FE & BE merged channel */
	unsigned int dpcm_merged_chan:1;
	/* DPCM used FE & BE merged rate */
	unsigned int dpcm_merged_rate:1;

	/* pmdown_time is ignored at stop */
	unsigned int ignore_pmdown_time:1;

	/* Do not create a PCM for this DAI link (Backend link) */
	unsigned int ignore:1;

#ifdef CONFIG_SND_SOC_TOPOLOGY
	struct snd_soc_dobj dobj; /* For topology */
#endif
};

static inline int snd_soc_link_num_ch_map(const struct snd_soc_dai_link *link)
{
	return max(link->num_cpus, link->num_codecs);
}

static inline struct snd_soc_dai_link_component*
snd_soc_link_to_cpu(struct snd_soc_dai_link *link, int n) {
	return &(link)->cpus[n];
}

static inline struct snd_soc_dai_link_component*
snd_soc_link_to_codec(struct snd_soc_dai_link *link, int n) {
	return &(link)->codecs[n];
}

static inline struct snd_soc_dai_link_component*
snd_soc_link_to_platform(struct snd_soc_dai_link *link, int n) {
	return &(link)->platforms[n];
}

#define for_each_link_codecs(link, i, codec)				\
	for ((i) = 0;							\
	     ((i) < link->num_codecs) &&				\
		     ((codec) = snd_soc_link_to_codec(link, i));		\
	     (i)++)

#define for_each_link_platforms(link, i, platform)			\
	for ((i) = 0;							\
	     ((i) < link->num_platforms) &&				\
		     ((platform) = snd_soc_link_to_platform(link, i));	\
	     (i)++)

#define for_each_link_cpus(link, i, cpu)				\
	for ((i) = 0;							\
	     ((i) < link->num_cpus) &&					\
		     ((cpu) = snd_soc_link_to_cpu(link, i));		\
	     (i)++)

#define for_each_link_ch_maps(link, i, ch_map)			\
	for ((i) = 0;						\
	     ((i) < snd_soc_link_num_ch_map(link) &&		\
		      ((ch_map) = link->ch_maps + i));		\
	     (i)++)

/*
 * Sample 1 : Single CPU/Codec/Platform
 *
 * SND_SOC_DAILINK_DEFS(test,
 *	DAILINK_COMP_ARRAY(COMP_CPU("cpu_dai")),
 *	DAILINK_COMP_ARRAY(COMP_CODEC("codec", "codec_dai")),
 *	DAILINK_COMP_ARRAY(COMP_PLATFORM("platform")));
 *
 * struct snd_soc_dai_link link = {
 *	...
 *	SND_SOC_DAILINK_REG(test),
 * };
 *
 * Sample 2 : Multi CPU/Codec, no Platform
 *
 * SND_SOC_DAILINK_DEFS(test,
 *	DAILINK_COMP_ARRAY(COMP_CPU("cpu_dai1"),
 *			   COMP_CPU("cpu_dai2")),
 *	DAILINK_COMP_ARRAY(COMP_CODEC("codec1", "codec_dai1"),
 *			   COMP_CODEC("codec2", "codec_dai2")));
 *
 * struct snd_soc_dai_link link = {
 *	...
 *	SND_SOC_DAILINK_REG(test),
 * };
 *
 * Sample 3 : Define each CPU/Codec/Platform manually
 *
 * SND_SOC_DAILINK_DEF(test_cpu,
 *		DAILINK_COMP_ARRAY(COMP_CPU("cpu_dai1"),
 *				   COMP_CPU("cpu_dai2")));
 * SND_SOC_DAILINK_DEF(test_codec,
 *		DAILINK_COMP_ARRAY(COMP_CODEC("codec1", "codec_dai1"),
 *				   COMP_CODEC("codec2", "codec_dai2")));
 * SND_SOC_DAILINK_DEF(test_platform,
 *		DAILINK_COMP_ARRAY(COMP_PLATFORM("platform")));
 *
 * struct snd_soc_dai_link link = {
 *	...
 *	SND_SOC_DAILINK_REG(test_cpu,
 *			    test_codec,
 *			    test_platform),
 * };
 *
 * Sample 4 : Sample3 without platform
 *
 * struct snd_soc_dai_link link = {
 *	...
 *	SND_SOC_DAILINK_REG(test_cpu,
 *			    test_codec);
 * };
 */

#define SND_SOC_DAILINK_REG1(name)	 SND_SOC_DAILINK_REG3(name##_cpus, name##_codecs, name##_platforms)
#define SND_SOC_DAILINK_REG2(cpu, codec) SND_SOC_DAILINK_REG3(cpu, codec, null_dailink_component)
#define SND_SOC_DAILINK_REG3(cpu, codec, platform)	\
	.cpus		= cpu,				\
	.num_cpus	= ARRAY_SIZE(cpu),		\
	.codecs		= codec,			\
	.num_codecs	= ARRAY_SIZE(codec),		\
	.platforms	= platform,			\
	.num_platforms	= ARRAY_SIZE(platform)

#define SND_SOC_DAILINK_REG(...) \
	CONCATENATE(SND_SOC_DAILINK_REG, COUNT_ARGS(__VA_ARGS__))(__VA_ARGS__)

#define SND_SOC_DAILINK_DEF(name, def...)		\
	static struct snd_soc_dai_link_component name[]	= { def }

#define SND_SOC_DAILINK_DEFS(name, cpu, codec, platform...)	\
	SND_SOC_DAILINK_DEF(name##_cpus, cpu);			\
	SND_SOC_DAILINK_DEF(name##_codecs, codec);		\
	SND_SOC_DAILINK_DEF(name##_platforms, platform)

#define DAILINK_COMP_ARRAY(param...)	param
#define COMP_EMPTY()			{ }
#define COMP_CPU(_dai)			{ .dai_name = _dai, }
#define COMP_CODEC(_name, _dai)		{ .name = _name, .dai_name = _dai, }
#define COMP_PLATFORM(_name)		{ .name = _name }
#define COMP_AUX(_name)			{ .name = _name }
#define COMP_CODEC_CONF(_name)		{ .name = _name }
#define COMP_DUMMY()			/* see snd_soc_fill_dummy_dai() */

extern struct snd_soc_dai_link_component null_dailink_component[0];
extern struct snd_soc_dai_link_component snd_soc_dummy_dlc;
int snd_soc_dlc_is_dummy(struct snd_soc_dai_link_component *dlc);

struct snd_soc_codec_conf {
	/*
	 * specify device either by device name, or by
	 * DT/OF node, but not both.
	 */
	struct snd_soc_dai_link_component dlc;

	/*
	 * optional map of kcontrol, widget and path name prefixes that are
	 * associated per device
	 */
	const char *name_prefix;
};

struct snd_soc_aux_dev {
	/*
	 * specify multi-codec either by device name, or by
	 * DT/OF node, but not both.
	 */
	struct snd_soc_dai_link_component dlc;

	/* codec/machine specific init - e.g. add machine controls */
	int (*init)(struct snd_soc_component *component);
};

/* SoC card */
struct snd_soc_card {
	const char *name;
	const char *long_name;
	const char *driver_name;
	const char *components;
#ifdef CONFIG_DMI
	char dmi_longname[80];
#endif /* CONFIG_DMI */

#ifdef CONFIG_PCI
	/*
	 * PCI does not define 0 as invalid, so pci_subsystem_set indicates
	 * whether a value has been written to these fields.
	 */
	unsigned short pci_subsystem_vendor;
	unsigned short pci_subsystem_device;
	bool pci_subsystem_set;
#endif /* CONFIG_PCI */

	char topology_shortname[32];

	struct device *dev;
	struct snd_card *snd_card;
	struct module *owner;

	struct mutex mutex;
	struct mutex dapm_mutex;

	/* Mutex for PCM operations */
	struct mutex pcm_mutex;
	enum snd_soc_pcm_subclass pcm_subclass;

	int (*probe)(struct snd_soc_card *card);
	int (*late_probe)(struct snd_soc_card *card);
	void (*fixup_controls)(struct snd_soc_card *card);
	int (*remove)(struct snd_soc_card *card);

	/* the pre and post PM functions are used to do any PM work before and
	 * after the codec and DAI's do any PM work. */
	int (*suspend_pre)(struct snd_soc_card *card);
	int (*suspend_post)(struct snd_soc_card *card);
	int (*resume_pre)(struct snd_soc_card *card);
	int (*resume_post)(struct snd_soc_card *card);

	/* callbacks */
	int (*set_bias_level)(struct snd_soc_card *,
			      struct snd_soc_dapm_context *dapm,
			      enum snd_soc_bias_level level);
	int (*set_bias_level_post)(struct snd_soc_card *,
				   struct snd_soc_dapm_context *dapm,
				   enum snd_soc_bias_level level);

	int (*add_dai_link)(struct snd_soc_card *,
			    struct snd_soc_dai_link *link);
	void (*remove_dai_link)(struct snd_soc_card *,
			    struct snd_soc_dai_link *link);

	long pmdown_time;

	/* CPU <--> Codec DAI links  */
	struct snd_soc_dai_link *dai_link;  /* predefined links only */
	int num_links;  /* predefined links only */

	struct list_head rtd_list;
	int num_rtd;

	/* optional codec specific configuration */
	struct snd_soc_codec_conf *codec_conf;
	int num_configs;

	/*
	 * optional auxiliary devices such as amplifiers or codecs with DAI
	 * link unused
	 */
	struct snd_soc_aux_dev *aux_dev;
	int num_aux_devs;
	struct list_head aux_comp_list;

	const struct snd_kcontrol_new *controls;
	int num_controls;

	/*
	 * Card-specific routes and widgets.
	 * Note: of_dapm_xxx for Device Tree; Otherwise for driver build-in.
	 */
	const struct snd_soc_dapm_widget *dapm_widgets;
	int num_dapm_widgets;
	const struct snd_soc_dapm_route *dapm_routes;
	int num_dapm_routes;
	const struct snd_soc_dapm_widget *of_dapm_widgets;
	int num_of_dapm_widgets;
	const struct snd_soc_dapm_route *of_dapm_routes;
	int num_of_dapm_routes;

	/* lists of probed devices belonging to this card */
	struct list_head component_dev_list;
	struct list_head list;

	struct list_head widgets;
	struct list_head paths;
	struct list_head dapm_list;
	struct list_head dapm_dirty;

	/* attached dynamic objects */
	struct list_head dobj_list;

	/* Generic DAPM context for the card */
	struct snd_soc_dapm_context dapm;
	struct snd_soc_dapm_stats dapm_stats;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_card_root;
#endif
#ifdef CONFIG_PM_SLEEP
	struct work_struct deferred_resume_work;
#endif
	u32 pop_time;

	/* bit field */
	unsigned int instantiated:1;
	unsigned int topology_shortname_created:1;
	unsigned int fully_routed:1;
	unsigned int probed:1;
	unsigned int component_chaining:1;
	struct device *devres_dev;

	void *drvdata;
};
#define for_each_card_prelinks(card, i, link)				\
	for ((i) = 0;							\
	     ((i) < (card)->num_links) && ((link) = &(card)->dai_link[i]); \
	     (i)++)
#define for_each_card_pre_auxs(card, i, aux)				\
	for ((i) = 0;							\
	     ((i) < (card)->num_aux_devs) && ((aux) = &(card)->aux_dev[i]); \
	     (i)++)

#define for_each_card_rtds(card, rtd)			\
	list_for_each_entry(rtd, &(card)->rtd_list, list)
#define for_each_card_rtds_safe(card, rtd, _rtd)	\
	list_for_each_entry_safe(rtd, _rtd, &(card)->rtd_list, list)

#define for_each_card_auxs(card, component)			\
	list_for_each_entry(component, &card->aux_comp_list, card_aux_list)
#define for_each_card_auxs_safe(card, component, _comp)	\
	list_for_each_entry_safe(component, _comp,	\
				 &card->aux_comp_list, card_aux_list)

#define for_each_card_components(card, component)			\
	list_for_each_entry(component, &(card)->component_dev_list, card_list)

#define for_each_card_dapms(card, dapm)					\
	list_for_each_entry(dapm, &card->dapm_list, list)

#define for_each_card_widgets(card, w)\
	list_for_each_entry(w, &card->widgets, list)
#define for_each_card_widgets_safe(card, w, _w)	\
	list_for_each_entry_safe(w, _w, &card->widgets, list)


static inline int snd_soc_card_is_instantiated(struct snd_soc_card *card)
{
	return card && card->instantiated;
}

/* SoC machine DAI configuration, glues a codec and cpu DAI together */
struct snd_soc_pcm_runtime {
	struct device *dev;
	struct snd_soc_card *card;
	struct snd_soc_dai_link *dai_link;
	struct snd_pcm_ops ops;

	unsigned int c2c_params_select; /* currently selected c2c_param for dai link */

	/* Dynamic PCM BE runtime data */
	struct snd_soc_dpcm_runtime dpcm[SNDRV_PCM_STREAM_LAST + 1];
	struct snd_soc_dapm_widget *c2c_widget[SNDRV_PCM_STREAM_LAST + 1];

	long pmdown_time;

	/* runtime devices */
	struct snd_pcm *pcm;
	struct snd_compr *compr;

	/*
	 * dais = cpu_dai + codec_dai
	 * see
	 *	soc_new_pcm_runtime()
	 *	snd_soc_rtd_to_cpu()
	 *	snd_soc_rtd_to_codec()
	 */
	struct snd_soc_dai **dais;

	struct delayed_work delayed_work;
	void (*close_delayed_work_func)(struct snd_soc_pcm_runtime *rtd);
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_dpcm_root;
#endif

	unsigned int id; /* 0-based and monotonic increasing */
	struct list_head list; /* rtd list of the soc card */

	/* function mark */
	struct snd_pcm_substream *mark_startup;
	struct snd_pcm_substream *mark_hw_params;
	struct snd_pcm_substream *mark_trigger;
	struct snd_compr_stream  *mark_compr_startup;

	/* bit field */
	unsigned int pop_wait:1;
	unsigned int fe_compr:1; /* for Dynamic PCM */
	unsigned int initialized:1;

	/* CPU/Codec/Platform */
	int num_components;
	struct snd_soc_component *components[] __counted_by(num_components);
};

/* see soc_new_pcm_runtime()  */
#define snd_soc_rtd_to_cpu(rtd, n)   (rtd)->dais[n]
#define snd_soc_rtd_to_codec(rtd, n) (rtd)->dais[n + (rtd)->dai_link->num_cpus]

static inline struct snd_soc_pcm_runtime *
snd_soc_substream_to_rtd(const struct snd_pcm_substream *substream)
{
	return snd_pcm_substream_chip(substream);
}

#define for_each_rtd_components(rtd, i, component)			\
	for ((i) = 0, component = NULL;					\
	     ((i) < rtd->num_components) && ((component) = rtd->components[i]);\
	     (i)++)
#define for_each_rtd_cpu_dais(rtd, i, dai)				\
	for ((i) = 0;							\
	     ((i) < rtd->dai_link->num_cpus) && ((dai) = snd_soc_rtd_to_cpu(rtd, i)); \
	     (i)++)
#define for_each_rtd_codec_dais(rtd, i, dai)				\
	for ((i) = 0;							\
	     ((i) < rtd->dai_link->num_codecs) && ((dai) = snd_soc_rtd_to_codec(rtd, i)); \
	     (i)++)
#define for_each_rtd_dais(rtd, i, dai)					\
	for ((i) = 0;							\
	     ((i) < (rtd)->dai_link->num_cpus + (rtd)->dai_link->num_codecs) &&	\
		     ((dai) = (rtd)->dais[i]);				\
	     (i)++)
#define for_each_rtd_dais_reverse(rtd, i, dai)					\
	for ((i) = (rtd)->dai_link->num_cpus + (rtd)->dai_link->num_codecs - 1;	\
	     (i) >= 0 && ((dai) = (rtd)->dais[i]);				\
	     (i)--)
#define for_each_rtd_ch_maps(rtd, i, ch_maps) for_each_link_ch_maps(rtd->dai_link, i, ch_maps)

void snd_soc_close_delayed_work(struct snd_soc_pcm_runtime *rtd);

/* mixer control */
struct soc_mixer_control {
	/* Minimum and maximum specified as written to the hardware */
	int min, max;
	/* Limited maximum value specified as presented through the control */
	int platform_max;
	int reg, rreg;
	unsigned int shift, rshift;
	u32 num_channels;
	unsigned int sign_bit;
	unsigned int invert:1;
	unsigned int autodisable:1;
#ifdef CONFIG_SND_SOC_TOPOLOGY
	struct snd_soc_dobj dobj;
#endif
};

struct soc_bytes {
	int base;
	int num_regs;
	u32 mask;
};

struct soc_bytes_ext {
	int max;
#ifdef CONFIG_SND_SOC_TOPOLOGY
	struct snd_soc_dobj dobj;
#endif
	/* used for TLV byte control */
	int (*get)(struct snd_kcontrol *kcontrol, unsigned int __user *bytes,
			unsigned int size);
	int (*put)(struct snd_kcontrol *kcontrol, const unsigned int __user *bytes,
			unsigned int size);
};

/* multi register control */
struct soc_mreg_control {
	long min, max;
	unsigned int regbase, regcount, nbits, invert;
};

/* enumerated kcontrol */
struct soc_enum {
	int reg;
	unsigned char shift_l;
	unsigned char shift_r;
	unsigned int items;
	unsigned int mask;
	const char * const *texts;
	const unsigned int *values;
	unsigned int autodisable:1;
#ifdef CONFIG_SND_SOC_TOPOLOGY
	struct snd_soc_dobj dobj;
#endif
};

static inline bool snd_soc_volsw_is_stereo(const struct soc_mixer_control *mc)
{
	if (mc->reg == mc->rreg && mc->shift == mc->rshift)
		return false;
	/*
	 * mc->reg == mc->rreg && mc->shift != mc->rshift, or
	 * mc->reg != mc->rreg means that the control is
	 * stereo (bits in one register or in two registers)
	 */
	return true;
}

static inline unsigned int snd_soc_enum_val_to_item(const struct soc_enum *e,
	unsigned int val)
{
	unsigned int i;

	if (!e->values)
		return val;

	for (i = 0; i < e->items; i++)
		if (val == e->values[i])
			return i;

	return 0;
}

static inline unsigned int snd_soc_enum_item_to_val(const struct soc_enum *e,
	unsigned int item)
{
	if (!e->values)
		return item;

	return e->values[item];
}

/**
 * snd_soc_kcontrol_component() - Returns the component that registered the
 *  control
 * @kcontrol: The control for which to get the component
 *
 * Note: This function will work correctly if the control has been registered
 * for a component. With snd_soc_add_codec_controls() or via table based
 * setup for either a CODEC or component driver. Otherwise the behavior is
 * undefined.
 */
static inline struct snd_soc_component *snd_soc_kcontrol_component(
	struct snd_kcontrol *kcontrol)
{
	return snd_kcontrol_chip(kcontrol);
}

int snd_soc_util_init(void);
void snd_soc_util_exit(void);

int snd_soc_of_parse_card_name(struct snd_soc_card *card,
			       const char *propname);
int snd_soc_of_parse_audio_simple_widgets(struct snd_soc_card *card,
					  const char *propname);
int snd_soc_of_parse_pin_switches(struct snd_soc_card *card, const char *prop);
int snd_soc_of_get_slot_mask(struct device_node *np,
			     const char *prop_name,
			     unsigned int *mask);
int snd_soc_of_parse_tdm_slot(struct device_node *np,
			      unsigned int *tx_mask,
			      unsigned int *rx_mask,
			      unsigned int *slots,
			      unsigned int *slot_width);
void snd_soc_of_parse_node_prefix(struct device_node *np,
				   struct snd_soc_codec_conf *codec_conf,
				   struct device_node *of_node,
				   const char *propname);
static inline
void snd_soc_of_parse_audio_prefix(struct snd_soc_card *card,
				   struct snd_soc_codec_conf *codec_conf,
				   struct device_node *of_node,
				   const char *propname)
{
	snd_soc_of_parse_node_prefix(card->dev->of_node,
				     codec_conf, of_node, propname);
}

int snd_soc_of_parse_audio_routing(struct snd_soc_card *card,
				   const char *propname);
int snd_soc_of_parse_aux_devs(struct snd_soc_card *card, const char *propname);

unsigned int snd_soc_daifmt_clock_provider_flipped(unsigned int dai_fmt);
unsigned int snd_soc_daifmt_clock_provider_from_bitmap(unsigned int bit_frame);

unsigned int snd_soc_daifmt_parse_format(struct device_node *np, const char *prefix);
unsigned int snd_soc_daifmt_parse_clock_provider_raw(struct device_node *np,
						     const char *prefix,
						     struct device_node **bitclkmaster,
						     struct device_node **framemaster);
#define snd_soc_daifmt_parse_clock_provider_as_bitmap(np, prefix)	\
	snd_soc_daifmt_parse_clock_provider_raw(np, prefix, NULL, NULL)
#define snd_soc_daifmt_parse_clock_provider_as_phandle			\
	snd_soc_daifmt_parse_clock_provider_raw
#define snd_soc_daifmt_parse_clock_provider_as_flag(np, prefix)		\
	snd_soc_daifmt_clock_provider_from_bitmap(			\
		snd_soc_daifmt_parse_clock_provider_as_bitmap(np, prefix))

int snd_soc_get_stream_cpu(const struct snd_soc_dai_link *dai_link, int stream);
int snd_soc_get_dlc(const struct of_phandle_args *args,
		    struct snd_soc_dai_link_component *dlc);
int snd_soc_of_get_dlc(struct device_node *of_node,
		       struct of_phandle_args *args,
		       struct snd_soc_dai_link_component *dlc,
		       int index);
int snd_soc_get_dai_id(struct device_node *ep);
int snd_soc_get_dai_name(const struct of_phandle_args *args,
			 const char **dai_name);
int snd_soc_of_get_dai_name(struct device_node *of_node,
			    const char **dai_name, int index);
int snd_soc_of_get_dai_link_codecs(struct device *dev,
				   struct device_node *of_node,
				   struct snd_soc_dai_link *dai_link);
void snd_soc_of_put_dai_link_codecs(struct snd_soc_dai_link *dai_link);
int snd_soc_of_get_dai_link_cpus(struct device *dev,
				 struct device_node *of_node,
				 struct snd_soc_dai_link *dai_link);
void snd_soc_of_put_dai_link_cpus(struct snd_soc_dai_link *dai_link);

int snd_soc_add_pcm_runtimes(struct snd_soc_card *card,
			     struct snd_soc_dai_link *dai_link,
			     int num_dai_link);
void snd_soc_remove_pcm_runtime(struct snd_soc_card *card,
				struct snd_soc_pcm_runtime *rtd);

void snd_soc_dlc_use_cpu_as_platform(struct snd_soc_dai_link_component *platforms,
				     struct snd_soc_dai_link_component *cpus);
struct of_phandle_args *snd_soc_copy_dai_args(struct device *dev,
					      const struct of_phandle_args *args);
struct snd_soc_dai *snd_soc_get_dai_via_args(const struct of_phandle_args *dai_args);
struct snd_soc_dai *snd_soc_register_dai(struct snd_soc_component *component,
					 struct snd_soc_dai_driver *dai_drv,
					 bool legacy_dai_naming);
void snd_soc_unregister_dai(struct snd_soc_dai *dai);

struct snd_soc_dai *snd_soc_find_dai(
	const struct snd_soc_dai_link_component *dlc);
struct snd_soc_dai *snd_soc_find_dai_with_mutex(
	const struct snd_soc_dai_link_component *dlc);

#include <sound/soc-dai.h>

static inline
int snd_soc_fixup_dai_links_platform_name(struct snd_soc_card *card,
					  const char *platform_name)
{
	struct snd_soc_dai_link *dai_link;
	const char *name;
	int i;

	if (!platform_name) /* nothing to do */
		return 0;

	/* set platform name for each dailink */
	for_each_card_prelinks(card, i, dai_link) {
		/* only single platform is supported for now */
		if (dai_link->num_platforms != 1)
			return -EINVAL;

		if (!dai_link->platforms)
			return -EINVAL;

		name = devm_kstrdup(card->dev, platform_name, GFP_KERNEL);
		if (!name)
			return -ENOMEM;

		/* only single platform is supported for now */
		dai_link->platforms->name = name;
	}

	return 0;
}

#ifdef CONFIG_DEBUG_FS
extern struct dentry *snd_soc_debugfs_root;
#endif

extern const struct dev_pm_ops snd_soc_pm_ops;

/*
 *	DAPM helper functions
 */
enum snd_soc_dapm_subclass {
	SND_SOC_DAPM_CLASS_ROOT		= 0,
	SND_SOC_DAPM_CLASS_RUNTIME	= 1,
};

static inline void _snd_soc_dapm_mutex_lock_root_c(struct snd_soc_card *card)
{
	mutex_lock_nested(&card->dapm_mutex, SND_SOC_DAPM_CLASS_ROOT);
}

static inline void _snd_soc_dapm_mutex_lock_c(struct snd_soc_card *card)
{
	mutex_lock_nested(&card->dapm_mutex, SND_SOC_DAPM_CLASS_RUNTIME);
}

static inline void _snd_soc_dapm_mutex_unlock_c(struct snd_soc_card *card)
{
	mutex_unlock(&card->dapm_mutex);
}

static inline void _snd_soc_dapm_mutex_assert_held_c(struct snd_soc_card *card)
{
	lockdep_assert_held(&card->dapm_mutex);
}

static inline void _snd_soc_dapm_mutex_lock_root_d(struct snd_soc_dapm_context *dapm)
{
	_snd_soc_dapm_mutex_lock_root_c(dapm->card);
}

static inline void _snd_soc_dapm_mutex_lock_d(struct snd_soc_dapm_context *dapm)
{
	_snd_soc_dapm_mutex_lock_c(dapm->card);
}

static inline void _snd_soc_dapm_mutex_unlock_d(struct snd_soc_dapm_context *dapm)
{
	_snd_soc_dapm_mutex_unlock_c(dapm->card);
}

static inline void _snd_soc_dapm_mutex_assert_held_d(struct snd_soc_dapm_context *dapm)
{
	_snd_soc_dapm_mutex_assert_held_c(dapm->card);
}

#define snd_soc_dapm_mutex_lock_root(x) _Generic((x),			\
	struct snd_soc_card * :		_snd_soc_dapm_mutex_lock_root_c, \
	struct snd_soc_dapm_context * :	_snd_soc_dapm_mutex_lock_root_d)(x)
#define snd_soc_dapm_mutex_lock(x) _Generic((x),			\
	struct snd_soc_card * :		_snd_soc_dapm_mutex_lock_c,	\
	struct snd_soc_dapm_context * :	_snd_soc_dapm_mutex_lock_d)(x)
#define snd_soc_dapm_mutex_unlock(x) _Generic((x),			\
	struct snd_soc_card * :		_snd_soc_dapm_mutex_unlock_c,	\
	struct snd_soc_dapm_context * :	_snd_soc_dapm_mutex_unlock_d)(x)
#define snd_soc_dapm_mutex_assert_held(x) _Generic((x),			\
	struct snd_soc_card * :		_snd_soc_dapm_mutex_assert_held_c, \
	struct snd_soc_dapm_context * :	_snd_soc_dapm_mutex_assert_held_d)(x)

/*
 *	PCM helper functions
 */
static inline void _snd_soc_dpcm_mutex_lock_c(struct snd_soc_card *card)
{
	mutex_lock_nested(&card->pcm_mutex, card->pcm_subclass);
}

static inline void _snd_soc_dpcm_mutex_unlock_c(struct snd_soc_card *card)
{
	mutex_unlock(&card->pcm_mutex);
}

static inline void _snd_soc_dpcm_mutex_assert_held_c(struct snd_soc_card *card)
{
	lockdep_assert_held(&card->pcm_mutex);
}

static inline void _snd_soc_dpcm_mutex_lock_r(struct snd_soc_pcm_runtime *rtd)
{
	_snd_soc_dpcm_mutex_lock_c(rtd->card);
}

static inline void _snd_soc_dpcm_mutex_unlock_r(struct snd_soc_pcm_runtime *rtd)
{
	_snd_soc_dpcm_mutex_unlock_c(rtd->card);
}

static inline void _snd_soc_dpcm_mutex_assert_held_r(struct snd_soc_pcm_runtime *rtd)
{
	_snd_soc_dpcm_mutex_assert_held_c(rtd->card);
}

#define snd_soc_dpcm_mutex_lock(x) _Generic((x),			\
	 struct snd_soc_card * :	_snd_soc_dpcm_mutex_lock_c,	\
	 struct snd_soc_pcm_runtime * :	_snd_soc_dpcm_mutex_lock_r)(x)

#define snd_soc_dpcm_mutex_unlock(x) _Generic((x),			\
	 struct snd_soc_card * :	_snd_soc_dpcm_mutex_unlock_c,	\
	 struct snd_soc_pcm_runtime * :	_snd_soc_dpcm_mutex_unlock_r)(x)

#define snd_soc_dpcm_mutex_assert_held(x) _Generic((x),		\
	struct snd_soc_card * :		_snd_soc_dpcm_mutex_assert_held_c, \
	struct snd_soc_pcm_runtime * :	_snd_soc_dpcm_mutex_assert_held_r)(x)

#include <sound/soc-component.h>
#include <sound/soc-card.h>
#include <sound/soc-jack.h>

#endif
