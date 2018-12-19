/*
 * ls2k.c  --  SoC audio for ls2k
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/i2c.h>

#include "ls2k-pcm.h"

static struct snd_soc_card snd_soc_z2;


static int edb93xx_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
#if 1
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int err;
	unsigned int mclk_rate;
	unsigned int rate = params_rate(params);

	/*
	 * According to CS4271 datasheet we use MCLK/LRCK=256 for
	 * rates below 50kHz and 128 for higher sample rates
	 */
	if (rate < 50000)
		mclk_rate = rate * 64 * 4;
	else
		mclk_rate = rate * 64 * 2;

	err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk_rate,
				     SND_SOC_CLOCK_IN);
	if (err)
		return err;

	return snd_soc_dai_set_sysclk(cpu_dai, 0, mclk_rate,
				      SND_SOC_CLOCK_OUT);
#else
	return 0;
#endif
}

static struct snd_soc_ops edb93xx_ops = {
	.hw_params	= edb93xx_hw_params,
};


static struct snd_soc_dai_link z2_dai = {
#ifdef CONFIG_SND_SOC_RT5651
	.name		= "rt5651",
	.stream_name	= "rt5651 Duplex",
	.codec_dai_name	= "rt5651-aif1",
	.codec_name	= "rt5651.3-001a",
#elif defined(CONFIG_SND_SOC_UDA1342)
	.name		= "uda1342",
	.stream_name	= "UDA1342 Duplex",
	.codec_dai_name	= "uda1342-hifi",
	.codec_name	= "uda1342-codec.1-001a",
#else
	#define USE_DUMMY_CODEC
	.name		= "dummy",
	.stream_name	= "dummy Duplex",
	.codec_dai_name	= "snd-soc-dummy-dai",
	.codec_name	= "snd-soc-dummy",

#endif
	.cpu_dai_name	= "ls2k-i2s",
	.ops = &edb93xx_ops,
	.platform_name = "ls2k-pcm-audio",
	.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			  SND_SOC_DAIFMT_CBS_CFS,
};


/*zqx----snd_soc_device和snd_soc_card代表Machine驱动*/
static struct snd_soc_card snd_soc_z2 = {
	.name		= "ls2k",
	.owner		= THIS_MODULE,
	.dai_link	= &z2_dai,
	.num_links	= 1,
};

/*zqx----platform_device代表整个音频子系统*/
static struct platform_device *ls2k_snd_ac97_device;

#ifdef USE_DUMMY_CODEC
static int i2c_write_codec(struct i2c_adapter *adapter, unsigned char reg, unsigned char *val)
{
	unsigned char msg[3] = {reg, *val, *(val + 1)};
	struct i2c_msg msgs[] = {
	    {
		.addr   = 0x1a,
		.flags  = 0,
		.len    = 3,
		.buf    = &msg[0],
	    }
	};

	if (i2c_transfer(adapter, msgs, 1) == 1) {
		return 0;
	}
	return -1;
}
static int codec_i2c_init(void)
{
	struct i2c_adapter *adapter;
	unsigned char set_reg[]={
	/******  reg   bit_hi bit_lo  ****/
	   0x00,   0x14,   0x02,
	   0x01,   0x00,   0x14,
	   0x10,   0xff,   0x03,
	   0x11,   0x30,   0x30,
	   0x12,   0xc4,   0x00,
	   0x20,   0x00,   0x30,
	   0x21,   0x00,   0x30,
	};
	int j;
	unsigned char  val[2] = {0x94,0x02};

	adapter = i2c_get_adapter(1);

	i2c_write_codec(adapter, 0x0, val);
		udelay(15000);

	for(j = 0; j < ARRAY_SIZE(set_reg) / 3; j++)
		i2c_write_codec(adapter, set_reg[j * 3], &set_reg[3 * j + 1]);

	return 0;
}
#endif

static int __init ls2k_init(void)
{
	int ret;

	pr_debug("Entered %s\n", __func__);

	ls2k_snd_ac97_device = platform_device_alloc("soc-audio", -1);
	if (!ls2k_snd_ac97_device)
		return -ENOMEM;

	/*将snd_soc_z2赋给 'plateform_device->dev->p->drive_data'----dev的核心驱动*/
	platform_set_drvdata(ls2k_snd_ac97_device,
				&snd_soc_z2);
	snd_soc_z2.dev = &ls2k_snd_ac97_device->dev;
#ifdef USE_DUMMY_CODEC
	codec_i2c_init();
#endif
	ret = platform_device_add(ls2k_snd_ac97_device);

	if (ret)
		platform_device_put(ls2k_snd_ac97_device);

	pr_debug("Exited %s\n", __func__);
	return ret;
}

static void __exit ls2k_exit(void)
{
	pr_debug("Entered %s\n", __func__);
	platform_device_unregister(ls2k_snd_ac97_device);
	pr_debug("Exited %s\n", __func__);
}

module_init(ls2k_init);
module_exit(ls2k_exit);

/* Module information */
MODULE_AUTHOR("Zhuo Qixiang");
MODULE_DESCRIPTION("ALSA SoC LS1A(LS2K)");
MODULE_LICENSE("GPL");





