/*
 * uda1342.c - Philips UDA1342 ALSA SoC audio driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) 2007-2009 Philipp Zabel <philipp.zabel@gmail.com>
 *
 * Modified by Richard Purdie <richard@openedhand.com> to fit into SoC
 * codec model.
 *
 * Copyright (c) 2005 Giorgio Padrin <giorgio@mandarinlogiq.org>
 * Copyright 2005 Openedhand Ltd.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "uda1342.h"

/* codec private data */
struct uda1342_priv {
	struct snd_soc_codec *codec;
	unsigned int dac_clk;
	struct work_struct work;
	void *control_data;
};

/*
 * uda1342 register cache
 */
static const u16 uda1342_reg[UDA1342_CACHEREGNUM] = {
	0x1402, 0x0014, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000,
	0xff03, 0x3030, 0xc400, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0030, 0x0030, 0x0000, 0x0000,
};

static unsigned long uda1342_cache_dirty;

/*
 * read uda1342 register cache
 */
static inline unsigned int uda1342_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg == UDA1342_RESET)
		return 0;
	if (reg >= UDA1342_CACHEREGNUM)
		return -1;
	return cache[reg];
}

/*
 * write uda1342 register cache
 */
static inline void uda1342_write_reg_cache(struct snd_soc_codec *codec,
	u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;

	if (reg >= UDA1342_CACHEREGNUM)
		return;
	if ((reg >= 0x10) && (cache[reg] != value))
		set_bit(reg - 0x10, &uda1342_cache_dirty);
	cache[reg] = value;
}

/*
 * write to the UDA1342 register space
 */
static int uda1342_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	u8 data[3];

	/* data is
	 *   data[0] is register offset
	 *   data[1] is MS byte
	 *   data[2] is LS byte
	 */
	data[0] = reg;
	data[1] = (value & 0xff00) >> 8;
	data[2] = value & 0x00ff;

	uda1342_write_reg_cache(codec, reg, value);

	/* the interpolator & decimator regs must only be written when the
	 * codec DAI is active.
	 */
	pr_debug("uda1342: hw write %x val %x\n", reg, value);
	if (codec->hw_write(codec->control_data, data, 3) == 3) {
		unsigned int val;
		i2c_master_send(codec->control_data, data, 1);
		i2c_master_recv(codec->control_data, data, 2);
		val = (data[0]<<8) | data[1];
		if (val != value) {
			pr_debug("uda1342: READ BACK VAL %x\n",
					(data[0]<<8) | data[1]);
			return -EIO;
		}
		if (reg >= 0x10)
			clear_bit(reg - 0x10, &uda1342_cache_dirty);
		return 0;
	} else
		return -EIO;
}

static void uda1342_sync_cache(struct snd_soc_codec *codec)
{
	int reg;
	u8 data[3];
	u16 *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware */
	for (reg = 0; reg < UDA1342_MVOL; reg++) {
		data[0] = reg;
		data[1] = (cache[reg] & 0xff00) >> 8;
		data[2] = cache[reg] & 0x00ff;
		if (codec->hw_write(codec->control_data, data, 3) != 3)
			dev_err(codec->dev, "%s: write to reg 0x%x failed\n",
				__func__, reg);
	}
}

static int uda1342_reset(struct snd_soc_codec *codec)
{
	int iface;

	iface = uda1342_read_reg_cache(codec, 0);
	uda1342_write(codec, 0, iface|0x8000);
	return 0;
}

static void uda1342_flush_work(struct work_struct *work)
{
	struct uda1342_priv *uda1342 = container_of(work, struct uda1342_priv, work);
	struct snd_soc_codec *uda1342_codec = uda1342->codec;
	int bit, reg;

	for_each_set_bit(bit, &uda1342_cache_dirty, UDA1342_CACHEREGNUM - 0x10) {
		reg = 0x10 + bit;
		pr_debug("uda1342: flush reg %x val %x:\n", reg,
				uda1342_read_reg_cache(uda1342_codec, reg));
		uda1342_write(uda1342_codec, reg,
				uda1342_read_reg_cache(uda1342_codec, reg));
		clear_bit(bit, &uda1342_cache_dirty);
	}

}

/* from 0 to 30 dB in 2 dB steps */
static DECLARE_TLV_DB_SCALE(mic_tlv, 0, 50, 0);

static const unsigned int uda1342_tlv[] = {
	TLV_DB_RANGE_HEAD(3),
	255-199, 255-0, TLV_DB_SCALE_ITEM(-5000+25, 25, 0),
	255-207, 255-200, TLV_DB_SCALE_ITEM(-5400+50, 50, 0),
	255-220, 255-208 , TLV_DB_SCALE_ITEM(-6600+60, 60, 0),
};

static const struct snd_kcontrol_new uda1342_snd_controls[] = {
	SOC_DOUBLE_TLV("Master Playback Volume", 0x11, 0, 8, 255, 1, uda1342_tlv),
	SOC_DOUBLE_TLV("Analog Mixer Volume", 0x12, 0, 8, 255, 1, uda1342_tlv),
	SOC_SINGLE_TLV("Mic Capture Volume", 0x20, 0, 48, 0, mic_tlv),
	SOC_SINGLE_TLV("Line Capture Volume", 0x21, 0, 48, 0, mic_tlv),
};


static int uda1342_set_dai_fmt_both(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	int iface;

	/* set up DAI based upon fmt */
	iface = uda1342_read_reg_cache(codec, 0);
	iface &= ~0x1c;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_LSB:
		iface |= 4;
		break;
	case SND_SOC_DAIFMT_MSB:
		iface |= 0x10;
		break;
	}


	uda1342_write(codec, 0, iface);

	return 0;
}



static int uda1342_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct uda1342_priv *uda1342 = snd_soc_codec_get_drvdata(codec);
	int mixer = uda1342_read_reg_cache(codec, UDA1342_MIXER);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		schedule_work(&uda1342->work);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		schedule_work(&uda1342->work);
		break;
	}
	return 0;
}

static int uda1342_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	return 0;
}

static void uda1342_pcm_shutdown(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
}

static int uda1342_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	int mute_reg = uda1342_read_reg_cache(codec, 0x10);

	pr_debug("%s mute: %d\n", __func__, mute);

	if (mute)
		mute_reg |= (4<<4);
	else
		mute_reg &= ~(7<<4);

	uda1342_write(codec, 0x10, mute_reg);

	return 0;
}


#define UDA1342_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		       SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
		       SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)

static const struct snd_soc_dai_ops uda1342_dai_ops = {
	.hw_params	= uda1342_pcm_hw_params,
	.shutdown	= uda1342_pcm_shutdown,
	.trigger	= uda1342_trigger,
	.digital_mute	= uda1342_mute,
	.set_fmt	= uda1342_set_dai_fmt_both,
};


static struct snd_soc_dai_driver uda1342_dai = {
	.name = "uda1342-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = UDA1342_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = UDA1342_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = &uda1342_dai_ops,
};

static int uda1342_suspend(struct snd_soc_codec *codec)
{
	return 0;
}

static int uda1342_resume(struct snd_soc_codec *codec)
{
	return 0;
}

static int uda1342_probe(struct snd_soc_codec *codec)
{
	struct uda1342_priv *uda1342 = snd_soc_codec_get_drvdata(codec);
	int ret;

	uda1342->codec = codec;

	codec->hw_write = (hw_write_t)i2c_master_send;
	codec->control_data = uda1342->control_data;

	uda1342_reset(codec);

	INIT_WORK(&uda1342->work, uda1342_flush_work);

	return 0;

}

/* power down chip */
static int uda1342_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_uda1342 = {
	.probe =	uda1342_probe,
	.remove =	uda1342_remove,
	.suspend =	uda1342_suspend,
	.resume =	uda1342_resume,
	.read =		uda1342_read_reg_cache,
	.write =	uda1342_write,
	.reg_cache_size = ARRAY_SIZE(uda1342_reg),
	.reg_word_size = sizeof(u16),
	.reg_cache_default = uda1342_reg,
	.reg_cache_step = 1,

	.controls = uda1342_snd_controls,
	.num_controls = ARRAY_SIZE(uda1342_snd_controls),
};

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static int uda1342_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct uda1342_priv *uda1342;
	int ret;

	uda1342 = devm_kzalloc(&i2c->dev, sizeof(struct uda1342_priv),
			       GFP_KERNEL);
	if (uda1342 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, uda1342);
	uda1342->control_data = i2c;

	ret =  snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_uda1342, &uda1342_dai, 1);
	printk("ret=%d\n", ret);
	return ret;
}

static int uda1342_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_codec(&i2c->dev);
	return 0;
}

static const struct i2c_device_id uda1342_i2c_id[] = {
	{ "codec_uda1342", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, uda1342_i2c_id);

static struct i2c_driver uda1342_i2c_driver = {
	.driver = {
		.name =  "uda1342-codec",
		.owner = THIS_MODULE,
	},
	.probe =    uda1342_i2c_probe,
	.remove =   uda1342_i2c_remove,
	.id_table = uda1342_i2c_id,
};
#endif

static int __init uda1342_modinit(void)
{
	int ret = 0;
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	ret = i2c_add_driver(&uda1342_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register UDA1342 I2C driver: %d\n", ret);
#endif
	return ret;
}
module_init(uda1342_modinit);

static void __exit uda1342_exit(void)
{
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&uda1342_i2c_driver);
#endif
}
module_exit(uda1342_exit);

MODULE_AUTHOR("Giorgio Padrin");
MODULE_DESCRIPTION("Audio support for codec Philips UDA1342");
MODULE_LICENSE("GPL");
