// SPDX-License-Identifier: GPL-2.0
// ASoC Sound Card driver for Microchip's ASRC
//
// Copyright (C) 2018-2022 Microchip Technology Inc. and its subsidiaries
//
// Author: Codrin Ciubotariu <codrin.ciubotariu@microchip.com>

#include <dt-bindings/sound/microchip,asrc-card.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/simple_card_utils.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/soc-dapm.h>

#include "../codecs/ad193x.h"
#include "../codecs/wm8731.h"

#define MCHP_ASRC_DSPS	4
#define XTAL_RATE 12288000	/* used for Mikroe Proto board*/

#define MCHP_PERIOD_SIZE_MIN	16384u	/* Workaround, calculated by trial and error */

static const char * const mchp_asrc_ctrl_playback_texts[] = {"Off",
	"ASRC Playback PCM 1 BE 1", "ASRC Playback PCM 1 BE 2",
		"ASRC Playback PCM 1 BE 3", "ASRC Playback PCM 1 BE 4",
	"ASRC Playback PCM 2 BE 1", "ASRC Playback PCM 2 BE 2",
		"ASRC Playback PCM 2 BE 3", "ASRC Playback PCM 2 BE 4",
	"ASRC Playback PCM 3 BE 1", "ASRC Playback PCM 3 BE 2",
		"ASRC Playback PCM 3 BE 3", "ASRC Playback PCM 3 BE 4",
	"ASRC Playback PCM 4 BE 1", "ASRC Playback PCM 4 BE 2",
		"ASRC Playback PCM 4 BE 3", "ASRC Playback PCM 4 BE 4",
};

static const char * const mchp_asrc_ctrl_capture_texts[] = {"Off",
	"ASRC Capture PCM 1 BE 1", "ASRC Capture PCM 1 BE 2",
		"ASRC Capture PCM 1 BE 3", "ASRC Capture PCM 1 BE 4",
	"ASRC Capture PCM 2 BE 1", "ASRC Capture PCM 2 BE 2",
		"ASRC Capture PCM 2 BE 3", "ASRC Capture PCM 2 BE 4",
	"ASRC Capture PCM 3 BE 1", "ASRC Capture PCM 3 BE 2",
		"ASRC Capture PCM 3 BE 3", "ASRC Capture PCM 3 BE 4",
	"ASRC Capture PCM 4 BE 1", "ASRC Capture PCM 4 BE 2",
		"ASRC Capture PCM 4 BE 3", "ASRC Capture PCM 4 BE 4",
};

static const struct soc_enum mchp_asrc_playback_enum =
	SOC_ENUM_SINGLE_VIRT(ARRAY_SIZE(mchp_asrc_ctrl_playback_texts),
			     mchp_asrc_ctrl_playback_texts);

static const struct soc_enum mchp_asrc_capture_enum =
	SOC_ENUM_SINGLE_VIRT(ARRAY_SIZE(mchp_asrc_ctrl_capture_texts),
			     mchp_asrc_ctrl_capture_texts);

static const struct snd_soc_dapm_route audio_playback_map[MCHP_ASRC_DSPS][MCHP_ASRC_DSPS] = {
	{
		{NULL, "ASRC Playback PCM 1 BE 1", "ASRC-Playback 1"},
		{NULL, "ASRC Playback PCM 2 BE 1", "ASRC-Playback 2"},
		{NULL, "ASRC Playback PCM 3 BE 1", "ASRC-Playback 3"},
		{NULL, "ASRC Playback PCM 4 BE 1", "ASRC-Playback 4"},
	}, {
		{NULL, "ASRC Playback PCM 1 BE 2", "ASRC-Playback 1"},
		{NULL, "ASRC Playback PCM 2 BE 2", "ASRC-Playback 2"},
		{NULL, "ASRC Playback PCM 3 BE 2", "ASRC-Playback 3"},
		{NULL, "ASRC Playback PCM 4 BE 2", "ASRC-Playback 4"},
	}, {
		{NULL, "ASRC Playback PCM 1 BE 3", "ASRC-Playback 1"},
		{NULL, "ASRC Playback PCM 2 BE 3", "ASRC-Playback 2"},
		{NULL, "ASRC Playback PCM 3 BE 3", "ASRC-Playback 3"},
		{NULL, "ASRC Playback PCM 4 BE 3", "ASRC-Playback 4"},
	}, {
		{NULL, "ASRC Playback PCM 1 BE 4", "ASRC-Playback 1"},
		{NULL, "ASRC Playback PCM 2 BE 4", "ASRC-Playback 2"},
		{NULL, "ASRC Playback PCM 3 BE 4", "ASRC-Playback 3"},
		{NULL, "ASRC Playback PCM 4 BE 4", "ASRC-Playback 4"},
	}
};

static const struct snd_soc_dapm_route audio_capture_map[MCHP_ASRC_DSPS][MCHP_ASRC_DSPS] = {
	{
		{"ASRC-Capture 1", "ASRC Capture PCM 1 BE 1", NULL},
		{"ASRC-Capture 2", "ASRC Capture PCM 2 BE 1", NULL},
		{"ASRC-Capture 3", "ASRC Capture PCM 3 BE 1", NULL},
		{"ASRC-Capture 4", "ASRC Capture PCM 4 BE 1", NULL},
	}, {
		{"ASRC-Capture 1", "ASRC Capture PCM 1 BE 2", NULL},
		{"ASRC-Capture 2", "ASRC Capture PCM 2 BE 2", NULL},
		{"ASRC-Capture 3", "ASRC Capture PCM 3 BE 2", NULL},
		{"ASRC-Capture 4", "ASRC Capture PCM 4 BE 2", NULL},
	}, {
		{"ASRC-Capture 1", "ASRC Capture PCM 1 BE 3", NULL},
		{"ASRC-Capture 2", "ASRC Capture PCM 2 BE 3", NULL},
		{"ASRC-Capture 3", "ASRC Capture PCM 3 BE 3", NULL},
		{"ASRC-Capture 4", "ASRC Capture PCM 4 BE 3", NULL},
	}, {
		{"ASRC-Capture 1", "ASRC Capture PCM 1 BE 4", NULL},
		{"ASRC-Capture 2", "ASRC Capture PCM 2 BE 4", NULL},
		{"ASRC-Capture 3", "ASRC Capture PCM 3 BE 4", NULL},
		{"ASRC-Capture 4", "ASRC Capture PCM 4 BE 4", NULL},
	}
};

#define PREFIX "microchip,"

struct mchp_dai_priv {
	int slots;
	int slot_width;
};

struct mchp_dai_link_priv {
	int id;
	struct mchp_dai_priv cpu;
	struct mchp_dai_priv codec;
	int convert_rate;
	int convert_channels;
	int convert_format;
};

struct mchp_card_priv {
	struct mchp_dai_link_priv *dai_link;
	int ctrl_playback_no;
	int ctrl_capture_no;
};

static int mchp_asrc_card_add_rtm_route(struct snd_soc_card *card,
					struct snd_soc_pcm_runtime *rtd,
					struct snd_soc_dapm_route *route,
					int *aifs_added)
{
	struct snd_soc_dapm_context *dapm = &card->dapm;
	struct mchp_card_priv *card_priv = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *cpu_dai;
	struct snd_kcontrol_new *control;
	struct snd_soc_dapm_widget *widget;
	int aifs = card_priv->ctrl_playback_no + card_priv->ctrl_capture_no;
	char *aux;
	int i, j, k;
	int err;

	for_each_rtd_cpu_dais(rtd, i, cpu_dai) {
		int route_pos;
		bool codec_playback_avail = false;
		bool codec_capture_avail = false;

		for (j = 0; j < rtd->dai_link->num_codecs; j++) {
			struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, j);

			if (codec_dai->driver->playback.stream_name)
				codec_playback_avail = true;
			if (codec_dai->driver->capture.stream_name)
				codec_capture_avail = true;
		}
		if (cpu_dai->stream[SNDRV_PCM_STREAM_PLAYBACK].widget && codec_playback_avail) {
			dev_dbg(card->dev, "CPU DAI %s: playback widget %s\n",
				cpu_dai->name, cpu_dai->stream[SNDRV_PCM_STREAM_PLAYBACK].widget->name);

			aux = devm_kmalloc(card->dev, 255, GFP_KERNEL);
			if (!aux)
				return -ENOMEM;
			if (cpu_dai->component->name_prefix)
				snprintf(aux, 255, "%s TX", cpu_dai->component->name_prefix);
			else
				snprintf(aux, 255, "%s TX", cpu_dai->name);

			control = devm_kzalloc(card->dev, sizeof(*control), GFP_KERNEL);
			if (!control)
				return -ENOMEM;
			control->iface = SNDRV_CTL_ELEM_IFACE_MIXER;
			control->name = "Playback Route";
			control->info = snd_soc_info_enum_double;
			control->get = snd_soc_dapm_get_enum_double;
			control->put = snd_soc_dapm_put_enum_double;
			control->private_value = (unsigned long)&mchp_asrc_playback_enum;

			widget = devm_kzalloc(card->dev, sizeof(*widget), GFP_KERNEL);
			if (!widget)
				return -ENOMEM;
			widget->id = snd_soc_dapm_mux;
			widget->name = aux;
			widget->kcontrol_news = control;
			widget->num_kcontrols = 1;
			err = snd_soc_dapm_new_controls(dapm, widget, 1);
			if (err) {
				dev_err(card->dev, "failed to add widget for %s: %d\n", aux, err);
				return err;
			}

			for (j = 0; j < MCHP_ASRC_DSPS; j++) { /* position in ASRC instance */
				for (k = 0; k < MCHP_ASRC_DSPS; k++) { /* ASRC instance number */
					route_pos = *aifs_added * MCHP_ASRC_DSPS +
						j * aifs * MCHP_ASRC_DSPS + k;

					route[route_pos] = audio_playback_map[j][k];
					route[route_pos].sink = aux;
					dev_dbg(card->dev, "adding route[%d]: %s -> %s -> %s\n",
						route_pos, route[route_pos].source,
						route[route_pos].control,
						route[route_pos].sink);
				}
			}
			route_pos = aifs * MCHP_ASRC_DSPS * MCHP_ASRC_DSPS + *aifs_added;
			route[route_pos].source = aux;
			route[route_pos].sink = cpu_dai->stream[SNDRV_PCM_STREAM_PLAYBACK].widget->name;
			dev_dbg(card->dev, "adding route[%d]: %s -> %s -> %s\n",
				route_pos, route[route_pos].source, route[route_pos].control,
				route[route_pos].sink);
			(*aifs_added)++;
		}
		if (cpu_dai->stream[SNDRV_PCM_STREAM_CAPTURE].widget && codec_capture_avail) {
			dev_dbg(card->dev, "CPU DAI %s: capture widget %s\n",
				cpu_dai->name, cpu_dai->stream[SNDRV_PCM_STREAM_CAPTURE].widget->name);

			aux = devm_kmalloc(card->dev, 255, GFP_KERNEL);
			if (!aux)
				return -ENOMEM;
			if (cpu_dai->component->name_prefix)
				snprintf(aux, 255, "%s RX", cpu_dai->component->name_prefix);
			else
				snprintf(aux, 255, "%s RX", cpu_dai->name);

			control = devm_kzalloc(card->dev, sizeof(*control), GFP_KERNEL);
			if (!control)
				return -ENOMEM;
			control->iface = SNDRV_CTL_ELEM_IFACE_MIXER;
			control->name = "Capture Route";
			control->info = snd_soc_info_enum_double;
			control->get = snd_soc_dapm_get_enum_double;
			control->put = snd_soc_dapm_put_enum_double;
			control->private_value = (unsigned long)&mchp_asrc_capture_enum;
			widget = devm_kzalloc(card->dev, sizeof(*widget), GFP_KERNEL);
			if (!widget)
				return -ENOMEM;
			widget->id = snd_soc_dapm_demux;
			widget->name = aux;
			widget->kcontrol_news = control;
			widget->num_kcontrols = 1;
			err = snd_soc_dapm_new_controls(dapm, widget, 1);
			if (err) {
				dev_err(card->dev, "failed to add widget for %s: %d\n", aux, err);
				return err;
			}
			for (j = 0; j < MCHP_ASRC_DSPS; j++) {
				for (k = 0; k < MCHP_ASRC_DSPS; k++) {
					route_pos = *aifs_added * MCHP_ASRC_DSPS +
						j * aifs * MCHP_ASRC_DSPS + k;

					route[route_pos] = audio_capture_map[j][k];
					route[route_pos].source = aux;
					dev_dbg(card->dev, "adding route[%d]: %s -> %s -> %s\n",
						route_pos, route[route_pos].source,
						route[route_pos].control, route[route_pos].sink);
				}
			}
			route_pos = aifs * MCHP_ASRC_DSPS * MCHP_ASRC_DSPS + *aifs_added;
			route[route_pos].source = cpu_dai->stream[SNDRV_PCM_STREAM_CAPTURE].widget->name;
			route[route_pos].sink = aux;
			dev_dbg(card->dev, "adding route[%d]: %s -> %s -> %s\n", route_pos,
				route[route_pos].source, route[route_pos].control,
				route[route_pos].sink);
			(*aifs_added)++;
		}
	}
	return 0;
}

static int mchp_asrc_card_late_probe(struct snd_soc_card *card)
{
	struct mchp_card_priv *card_priv = snd_soc_card_get_drvdata(card);
	struct snd_soc_dapm_context *dapm = &card->dapm;
	struct snd_soc_dapm_route *route;
#ifdef DEBUG
	struct snd_soc_dapm_widget *widget;
#endif
	int aifs = card_priv->ctrl_playback_no + card_priv->ctrl_capture_no;
	int aifs_added = 0;
	int ret;
	struct snd_soc_pcm_runtime *rtd;

	route = devm_kzalloc(card->dev, sizeof(*route) *
					(MCHP_ASRC_DSPS * MCHP_ASRC_DSPS + 1) *
					aifs, GFP_KERNEL);
	if (!route)
		return -ENOMEM;

	for_each_card_rtds(card, rtd) {
		if (!rtd->dai_link->no_pcm)
			continue;
		ret = mchp_asrc_card_add_rtm_route(card, rtd, route, &aifs_added);
		if (ret < 0)
			return ret;
	}

	dev_dbg(card->dev, "Adding %d routes from total of %d\n", aifs, aifs_added);
	ret = snd_soc_dapm_add_routes(dapm, route, aifs_added *
				      (MCHP_ASRC_DSPS * MCHP_ASRC_DSPS + 1));
	if (ret < 0) {
		dev_err(card->dev, "failed to add routes: %d\n", ret);
		return ret;
	}
	dev_dbg(card->dev, "Added routes\n");

	dev_dbg(card->dev, "widgets available:\n");
#ifdef DEBUG
	for_each_card_widgets(dapm->card, widget)
		dev_dbg(card->dev, "%s\n", widget->name);
#endif

	return 0;
}

static int mchp_asrc_card_be_dai_link_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_card *card = rtd->card;
	struct mchp_card_priv *card_priv = snd_soc_card_get_drvdata(card);
	int i;
	int codec_playback_avail = false;
	int codec_capture_avail = false;

	if (cpu_dai->component)
		dev_dbg(card->dev, "%s: prefix %s\n", __func__, cpu_dai->component->name_prefix);
	else
		dev_dbg(card->dev, "%s: no prefix for DAI %s\n", __func__, cpu_dai->name);

	for (i = 0; i < rtd->dai_link->num_codecs; i++) {
		struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, i);

		if (codec_dai->driver->playback.stream_name)
			codec_playback_avail = true;
		if (codec_dai->driver->capture.stream_name)
			codec_capture_avail = true;
	}
	if (cpu_dai->driver->playback.stream_name && codec_playback_avail)
		card_priv->ctrl_playback_no++;

	if (cpu_dai->driver->capture.stream_name && codec_capture_avail)
		card_priv->ctrl_capture_no++;

	return 0;
}

static int mchp_asoc_card_fixup(struct snd_soc_pcm_runtime *rtd, struct snd_pcm_hw_params *params)
{
	struct snd_soc_card *card = rtd->card;
	struct mchp_card_priv *card_priv = snd_soc_card_get_drvdata(card);
	struct mchp_dai_link_priv *priv = card_priv->dai_link;
	struct snd_interval *rate, *period_size;
	struct snd_mask *mask;
	struct snd_interval *ct = hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	int i;

	for (i = 0; i < card->num_links; i++, priv++) {
		if (rtd->dai_link->id == priv->id)
			break;
	}
	if (i == card->num_links) {
		dev_err(card->dev, "priv not found for %s\n", rtd->dai_link->name);
		return -EINVAL;
	}

	ct->min = priv->convert_channels;
	ct->max = priv->convert_channels;

	rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	rate->max = priv->convert_rate;
	rate->min = priv->convert_rate;

	mask = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);
	snd_mask_none(mask);
	snd_mask_set(mask, priv->convert_format);

	/* set workaround to avoid ASRC noise issue */
	period_size = hw_param_interval(params, SNDRV_PCM_HW_PARAM_PERIOD_BYTES);
	period_size->min = max(period_size->min, MCHP_PERIOD_SIZE_MIN);

	return 0;
}

static int mchp_asrc_card_dai_link_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai_link *dai_link = rtd->dai_link;
	struct snd_soc_card *card = rtd->card;
	unsigned int i;
	int err = 0;
	struct mchp_card_priv *card_priv = snd_soc_card_get_drvdata(card);
	struct mchp_dai_link_priv *priv = card_priv->dai_link;

	for (i = 0; i < card->num_links; i++, priv++) {
		if (dai_link->id == priv->id)
			break;
	}
	if (i == card->num_links) {
		dev_err(card->dev, "private not found for %s\n", dai_link->name);
		return -EINVAL;
	}

	dev_dbg(card->dev, "Found priv id %d, cpu slot %d, codec slot %d\n",
		priv->id, priv->cpu.slots, priv->codec.slots);

	/* get TDM values */
	if (priv->cpu.slots) {
		dev_dbg(card->dev, "Setting TDM slot %d, width %d in %s node\n",
			priv->cpu.slots, priv->cpu.slot_width, cpu_dai->name);
		err = snd_soc_dai_set_tdm_slot(cpu_dai,
					       GENMASK(priv->cpu.slots - 1, 0),
					       GENMASK(priv->cpu.slots - 1, 0),
					       priv->cpu.slots,
					       priv->cpu.slot_width);
		if (err) {
			dev_err(card->dev, "Failed to set TDM params for %s node: %d\n",
				cpu_dai->name, err);
		}
	}

	for (i = 0; i < rtd->dai_link->num_codecs; i++) {
		struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, i);

		/* Mikroe Proto Board uses a 12.288MhZ XTAL */
		if (strstr(codec_dai->driver->name, "wm8731")) {
			dev_dbg(card->dev, "setting sysclk for %s\n",
				codec_dai->driver->name);
			err = snd_soc_dai_set_sysclk(codec_dai, WM8731_SYSCLK_XTAL,
						     XTAL_RATE, SND_SOC_CLOCK_IN);
			if (err < 0) {
				dev_err(card->dev, "Failed to set WM8731 SYSCLK: %d\n",
					err);
			}
		}
		if (priv->codec.slots) {
			dev_info(card->dev, "Setting TDM slot %d, width %d in %s node\n",
				 priv->codec.slots, priv->codec.slot_width,
				 codec_dai->name);
			err = snd_soc_dai_set_tdm_slot(codec_dai,
						       GENMASK(priv->codec.slots - 1, 0),
						       GENMASK(priv->codec.slots - 1, 0),
						       priv->codec.slots,
						       priv->codec.slot_width);
			if (err)
				dev_err(card->dev, "Failed to set TDM params for %s node: %d\n",
					codec_dai->name, err);
		}
	}

	return 0;
}

static int mchp_asoc_card_parse_convert(struct device *dev, struct device_node *np,
					char *prefix,
					struct mchp_dai_link_priv *priv)
{
	char prop[128];
	int ret;

	if (!prefix)
		prefix = "";

	/* sampling rate convert */
	snprintf(prop, sizeof(prop), "%s%s", prefix, "convert-rate");
	ret = of_property_read_u32(np, prop, &priv->convert_rate);
	if (ret < 0) {
		dev_err(dev, "unable to get convert-rate for %s", np->full_name);
		return ret;
	}
	if (priv->convert_rate < 8000 || priv->convert_rate > 192000) {
		dev_err(dev, "invalid %sconvert-rate value for %s: %d",
			prefix, np->full_name, priv->convert_rate);
		return -EINVAL;
	}
	dev_dbg(dev, "convert-rate for %s: %d\n", np->full_name, priv->convert_rate);

	/* channels transfer */
	snprintf(prop, sizeof(prop), "%s%s", prefix, "convert-channels");
	ret = of_property_read_u32(np, prop, &priv->convert_channels);
	if (ret < 0) {
		dev_err(dev, "unable to get %sconvert-channels for %s", prefix,
			np->full_name);
		return ret;
	}
	if (priv->convert_channels < 1 || priv->convert_channels > 8) {
		dev_err(dev, "invalid %sconvert-channels value for %s: %d",
			prefix, np->full_name, priv->convert_channels);
		return -EINVAL;
	}
	dev_dbg(dev, "convert-channels for %s: %d\n", np->full_name,
		priv->convert_channels);

	/* sample format */
	snprintf(prop, sizeof(prop), "%s%s", prefix, "convert-sample-format");
	ret = of_property_read_u32(np, prop, &priv->convert_format);
	if (ret < 0) {
		dev_err(dev, "unable to get convert-sample-format for %s", np->full_name);
		return ret;
	}
	if (priv->convert_format != MCHP_ASRC_PCM_FORMAT_S8 &&
	    priv->convert_format != MCHP_ASRC_PCM_FORMAT_S16_LE &&
	    priv->convert_format != MCHP_ASRC_PCM_FORMAT_S24_LE &&
	    priv->convert_format != MCHP_ASRC_PCM_FORMAT_S32_LE) {
		dev_err(dev, "invalid %sconvert-sample-format value for %s: %d",
			prefix, np->full_name, priv->convert_format);
		return -EINVAL;
	}
	dev_dbg(dev, "convert-sample-format for %s: %d\n", np->full_name,
		priv->convert_format);

	return 0;
}

static void mchp_parse_tdm_params(struct device *dev, struct device_node *np,
				  struct mchp_dai_priv *priv)
{
	int err;

	if (!np || !priv)
		return;

	err = snd_soc_of_parse_tdm_slot(np, NULL, NULL, &priv->slots,
					&priv->slot_width);
	if (err) {
		dev_dbg(dev,
			"TDM slot num/width not available in %s node: %d\n",
			np->name, err);
	} else {
		dev_dbg(dev, "TDM slot %d, width %d in %s node\n",
			priv->slots, priv->slot_width, np->name);
	}
}

static void mchp_asrc_card_of_put(struct snd_soc_card *card)
{
	int i;

	for (i = 0; i < card->num_links; i++) {
		struct snd_soc_dai_link *dai_link = &card->dai_link[i];

		if (dai_link->no_pcm)
			continue;

		of_node_put(dai_link->cpus->of_node);
		snd_soc_of_put_dai_link_codecs(dai_link);
	}
}

static int mchp_asrc_card_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *node;
	struct snd_soc_card *card;
	struct mchp_card_priv *priv;
	struct snd_soc_dai_link *dai_link;
	int err, num_links, num_dai_links;
	int be_index = 0;
	int i;
	int asrc_fe_no = 0;
	struct of_phandle_args *asrc_args;

	if (!np) {
		dev_err(&pdev->dev, "No device node supplied\n");
		return -EINVAL;
	}

	card = devm_kzalloc(&pdev->dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	node = of_get_child_by_name(np, PREFIX "dai-link");
	if (node) {
		of_node_put(node);
		node = NULL;
		num_links = of_get_available_child_count(np);
	} else {
		dev_err(&pdev->dev, "no DT dai-links found\n");
		return -EINVAL;
	}

	/* allocate private structure for BEs */
	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dai_link = devm_kzalloc(&pdev->dev,
				      sizeof(*priv->dai_link) * num_links,
				      GFP_KERNEL);
	if (!priv->dai_link)
		return -ENOMEM;

	snd_soc_card_set_drvdata(card, priv);

	asrc_fe_no = of_count_phandle_with_args(np, PREFIX "audio-asrc", "#sound-dai-cells");
	if (asrc_fe_no < 0) {
		dev_info(&pdev->dev, "no ASRC instances found\n");
		asrc_fe_no = 0;
	} else {
		dev_info(&pdev->dev, "ASRC %d FEs\n", asrc_fe_no);
	}

	if (asrc_fe_no) {
		asrc_args = devm_kmalloc(&pdev->dev, sizeof(*asrc_args) * asrc_fe_no,
					 GFP_KERNEL);
		if (!asrc_args)
			return -ENOMEM;

		/* all DAI links will also get a BE */
		num_dai_links = num_links * 2;
	} else {
		num_dai_links = num_links;
	}

	num_dai_links += asrc_fe_no;

	card->dai_link = devm_kzalloc(&pdev->dev,
				      sizeof(*card->dai_link) * num_dai_links,
				      GFP_KERNEL);
	if (!card->dai_link)
		return -ENOMEM;

	dai_link = card->dai_link;

	for (i = 0; i < asrc_fe_no; i++) {
		struct snd_soc_dai_link_component *comp;
		struct device_node *asrc;

		err = of_parse_phandle_with_args(np, PREFIX "audio-asrc",
						 "#sound-dai-cells", i, &asrc_args[i]);
		if (err < 0) {
			if (err == -EPROBE_DEFER)
				return -EPROBE_DEFER;

			dev_warn(&pdev->dev, "ASRC DT phandle %d not found: %d\n",
				 i, err);
			continue;
		}

		dev_dbg(&pdev->dev, "ASRC DT phandle %d found: %s\n",
			i, asrc_args[i].np->name);

		asrc = asrc_args[i].np;
		comp = devm_kzalloc(&pdev->dev, 3 * sizeof(*comp), GFP_KERNEL);
		if (!comp) {
			of_node_put(asrc);
			return -ENOMEM;
		}

		dai_link->cpus = &comp[0];
		dai_link->codecs = &comp[1];
		dai_link->platforms = &comp[2];

		dai_link->cpus->of_node = asrc;
		dai_link->platforms->of_node = asrc;
		dai_link->codecs->name = "snd-soc-dummy";
		dai_link->codecs->dai_name = "snd-soc-dummy-dai";
		dai_link->num_cpus = 1;
		dai_link->num_codecs = 1;
		dai_link->num_platforms = 1;

		dai_link->dpcm_playback = 1;
		dai_link->dpcm_capture = 1;
		dai_link->dynamic = 1;
		dai_link->dpcm_loopback = 1;
		dai_link->id = card->num_links;
		dai_link->trigger[SNDRV_PCM_STREAM_PLAYBACK] =
			dai_link->trigger[SNDRV_PCM_STREAM_CAPTURE] = SND_SOC_DPCM_TRIGGER_PRE;
		err = snd_soc_get_dai_name(&asrc_args[i], &dai_link->cpus->dai_name);
		if (err) {
			if (err == -EPROBE_DEFER) {
				of_node_put(asrc);
				return -EPROBE_DEFER;
			}
			dev_err(&pdev->dev, "error getting asrc dai name\n");
		} else {
			dev_dbg(&pdev->dev, "asrc cpu_dai_name %s\n",
				dai_link->cpus->dai_name);
			dai_link->name = dai_link->cpus->dai_name;
			dai_link->stream_name = dai_link->cpus->dai_name;
		}
		dev_info(&pdev->dev, "Found DPCM FE: %s\n", dai_link->cpus->dai_name);

		of_node_put(asrc);
		dai_link++;
		card->num_links++;
	}

	for_each_available_child_of_node(np, node) {
		struct device_node *cpu;
		struct device_node *codec;
		int index;
		char *name;
		struct snd_soc_dai_link_component *comp;

		/* add normal PCM first */
		comp = devm_kzalloc(&pdev->dev, 3 * sizeof(*comp), GFP_KERNEL);
		if (!comp)
			return -ENOMEM;

		dai_link->cpus = &comp[0];
		dai_link->codecs = &comp[1];
		dai_link->platforms = &comp[2];
		dai_link->num_cpus = 1;
		dai_link->num_codecs = 1;
		dai_link->num_platforms = 1;

		cpu = of_get_child_by_name(node, "cpu");
		if (!cpu) {
			dev_err(&pdev->dev, "no cpu DT node found\n");
			continue;
		}
		mchp_parse_tdm_params(&pdev->dev, cpu, &priv->dai_link[be_index].cpu);
		codec = of_get_child_by_name(node, "codec");
		if (!codec) {
			dev_err(&pdev->dev, "no codec DT node found\n");
			of_node_put(cpu);
			continue;
		}
		mchp_parse_tdm_params(&pdev->dev, codec, &priv->dai_link[be_index].codec);
		err = asoc_simple_parse_daifmt(&pdev->dev, node, codec, PREFIX,
					       &dai_link->dai_fmt);
		if (err < 0) {
			of_node_put(codec);
			of_node_put(cpu);
			if (err == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			dev_err(&pdev->dev, "error getting daifmt from DT\n");
			continue;
		}

		dai_link->cpus->of_node = of_parse_phandle(cpu, "sound-dai", 0);
		if (!dai_link->cpus->of_node) {
			dev_err(&pdev->dev, "error getting cpu phandle for %s\n",
				cpu->name);
			of_node_put(codec);
			of_node_put(cpu);
			continue;
		}
		err = snd_soc_of_get_dai_link_codecs(&pdev->dev, codec, dai_link);
		if (err) {
			of_node_put(dai_link->cpus->of_node);
			of_node_put(codec);
			of_node_put(cpu);
			if (err == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			dev_err(&pdev->dev, "error getting DAI codecs: %d\n", err);
			continue;
		}

		err = snd_soc_of_get_dai_name(cpu, &dai_link->cpus->dai_name, 0);
		if (err) {
			snd_soc_of_put_dai_link_codecs(dai_link);
			of_node_put(dai_link->cpus->of_node);
			of_node_put(codec);
			of_node_put(cpu);
			if (err == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			dev_err(&pdev->dev, "error getting cpu dai name: %d\n", err);
			continue;
		}
		dev_dbg(&pdev->dev, "current cpu_dai_name %s\n",
			dai_link->cpus->dai_name);

		name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s -",
				      dai_link->cpus->dai_name);
		if (!name) {
			snd_soc_of_put_dai_link_codecs(dai_link);
			of_node_put(dai_link->cpus->of_node);
			of_node_put(codec);
			of_node_put(cpu);
			return -ENOMEM;
		}

		for (index = 0;
		     index < dai_link->num_codecs;
		     index++) {
			name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s %s",
					      name,
					      dai_link->codecs[index].dai_name);
			if (!name) {
				snd_soc_of_put_dai_link_codecs(dai_link);
				of_node_put(dai_link->cpus->of_node);
				of_node_put(codec);
				of_node_put(cpu);
				return -ENOMEM;
			}
		}

		dai_link->name = name;
		dai_link->stream_name = name;
		dai_link->init = &mchp_asrc_card_dai_link_init;
		dai_link->id = card->num_links;
		dai_link->platforms->of_node = dai_link->cpus->of_node;
		priv->dai_link[be_index].id = card->num_links;

		dai_link++;
		card->num_links++;

		/*
		 * if the convert properties are not available, do not create a
		 * BE DAI Link for this DAI
		 */
		err = mchp_asoc_card_parse_convert(&pdev->dev, node, PREFIX,
						   &priv->dai_link[be_index]);
		if (err) {
			dev_info(&pdev->dev, "not registering DAI %s as BE\n",
				 (dai_link - 1)->name);
		}
		if (!asrc_fe_no || err) {
			be_index++;
			of_node_put(codec);
			of_node_put(cpu);
			continue;
		}

		/* add the DPCM BE */

		memcpy(dai_link, &card->dai_link[card->num_links - 1], sizeof(*dai_link));

		name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "BE %s",
				      card->dai_link[card->num_links - 1].name);
		dai_link->name = name;
		dai_link->stream_name = name;
		dai_link->no_pcm = 1;
		dai_link->ignore_suspend = 1;
		dai_link->ignore_pmdown_time = 1;
		dai_link->init = &mchp_asrc_card_be_dai_link_init;
		dai_link->trigger[SNDRV_PCM_STREAM_PLAYBACK] =
			dai_link->trigger[SNDRV_PCM_STREAM_CAPTURE] = SND_SOC_DPCM_TRIGGER_POST;

		if (strstr(name, "spdiftx")) {
			dai_link->dpcm_playback = 1;
		} else if (strstr(name, "spdifrx")) {
			dai_link->dpcm_capture = 1;
		} else if (strstr(name, "pdmc")) {
			dai_link->dpcm_capture = 1;
		} else {
			dai_link->dpcm_playback = 1;
			dai_link->dpcm_capture = 1;
		}
		dai_link->be_hw_params_fixup = mchp_asoc_card_fixup;
		dev_info(&pdev->dev, "Found DPCM BE: %s, id %d\n",
			 dai_link->name, dai_link->id);

		of_node_put(codec);
		of_node_put(cpu);

		dai_link++;
		card->num_links++;
		be_index++;
	}

	if (!card->num_links) {
		dev_err(&pdev->dev, "no legit dai-links found\n");
		return -EINVAL;
	}
	dev_dbg(&pdev->dev, "Found %d links\n", card->num_links);

	card->dev = &pdev->dev;
	card->owner = THIS_MODULE;

	err = snd_soc_of_parse_card_name(card, PREFIX "model");
	if (err) {
		dev_err(&pdev->dev, PREFIX "model not found in DT\n");
		goto _dai_link_put;
	}

	if (of_property_read_bool(np, PREFIX "audio-routing"))
		snd_soc_of_parse_audio_routing(card, PREFIX "audio-routing");

	priv->ctrl_playback_no = 0;
	priv->ctrl_capture_no = 0;

	card->late_probe = mchp_asrc_card_late_probe;

	err = snd_soc_register_card(card);
	if (err) {
		dev_err(&pdev->dev,
			"ASoc: Platform device allocation failed: %d\n", err);
		goto _dai_link_put;
	}

_dai_link_put:
	mchp_asrc_card_of_put(card);

	return err;
}

static int mchp_asrc_card_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id mchp_asrc_card_of_match[] = {
	{ .compatible = "microchip,asrc-card",},
	{},
};
MODULE_DEVICE_TABLE(of, mchp_asrc_card_of_match);

static struct platform_driver mchp_asrc_card_driver = {
	.driver = {
		.name = "mchp-asrc-card",
		.of_match_table = of_match_ptr(mchp_asrc_card_of_match),
	},
	.probe = mchp_asrc_card_probe,
	.remove = mchp_asrc_card_remove,
};
module_platform_driver(mchp_asrc_card_driver);

/* Module information */
MODULE_AUTHOR("Codrin Ciubotariu <codrin.ciubotariu@microchip.com>");
MODULE_DESCRIPTION("ALSA SoC machine driver for Microchip's ASRC");
MODULE_LICENSE("GPL v2");
