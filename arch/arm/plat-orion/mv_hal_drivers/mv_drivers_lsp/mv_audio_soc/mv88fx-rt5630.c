/*
 *
 *	Marvell Orion Alsa SOC Sound driver
 *
 *	Author: Yuval Elmaliah
 *	Author: Ethan Ku
       Author: Brian Hsu
 *	Copyright (C) 2008 Marvell Ltd.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/asoundef.h>
#include <asm/mach-types.h>
//#include <asm/arch/hardware.h>
#include "../sound/soc/codecs/rt5630.h"
#include <linux/mbus.h>
#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <../arch/arm/mach-dove/common.h>
//#include <asm/plat-orion/i2s-orion.h>
#include "mv88fx-pcm.h"
#include "mv88fx-i2s.h"
#include "ctrlEnv/mvCtrlEnvSpec.h"
#include "audio/mvAudioRegs.h"

//#define mv88fx_snd_debug(fmt, arg...) printk(KERN_ERR fmt, ##arg)


extern int mvmpp_gpio_get_value(unsigned int);
extern int mvmpp_gpio_set_value(unsigned int,int);

static struct mv88fx_snd_machine_data mv88fx_machine_data[2];


static int mv88fx_machine_hifi_startup(struct snd_pcm_substream *substream)
{
	return 0;
}


static void mv88fx_machine_hifi_shutdown(struct snd_pcm_substream *substream)
{
//	printk("%s\n",__func__);
}


static int mv88fx_machine_hifi_hw_params(struct snd_pcm_substream *substream,	
				   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int clk = 0;
	int ret = 0;
	unsigned int format;

//	printk("%s\n",__func__);
	/* set codec DAI configuration */

	mv88fx_snd_debug("mv88fx_machine_hifi_hw_params");

	switch (params_rate(params)) {
	case 44100:
		clk = 11289600;
		break;
	case 48000:
		clk = 12288000;
		break;
	case 96000:
		clk = 24576000;
		break;
	}

	format = SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF;
	if ( codec_dai->driver->ops && codec_dai->driver->ops->set_fmt )
		ret = codec_dai->driver->ops->set_fmt(codec_dai, format);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	if ( cpu_dai->driver->ops && cpu_dai->driver->ops->set_fmt )
		ret = cpu_dai->driver->ops->set_fmt(cpu_dai, format);
	if (ret < 0)
		return ret;

	/* cpu clock is the mv88fx master clock sent to codec */
	if ( cpu_dai->driver->ops && cpu_dai->driver->ops->set_sysclk )
		ret = cpu_dai->driver->ops->set_sysclk(cpu_dai, 0, 0, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* codec system clock is supplied by codec pll1*/
	ret = codec_dai->driver->ops->set_pll(codec_dai, RT5630_PLL1_FROM_MCLK, 0, clk,clk*2);
	 if (ret < 0)
	  	return ret;
	 
	/* codec system clock is supplied by mv88fx*/
	if ( codec_dai->driver->ops && codec_dai->driver->ops->set_sysclk )
		ret = codec_dai->driver->ops->set_sysclk(codec_dai, 0, clk*2, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	return 0;
}

static int mv88fx_machine_hifi_trigger(struct snd_pcm_substream *substream,int cmd)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if(substream->stream==SNDRV_PCM_STREAM_PLAYBACK)
		{
//			mvmpp_gpio_set_value(MPP_Amp_PwrDn,1);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		if(substream->stream==SNDRV_PCM_STREAM_PLAYBACK)
		{
//			mvmpp_gpio_set_value(MPP_Amp_PwrDn,0);
		}
		break;

	}
	return 0;
}

static int mv88fx_rt5630_hifi_init(struct snd_soc_codec *codec)
{
	mv88fx_snd_debug("");
//	mvmpp_gpio_set_value(MPP_Amp_PwrDn,0);
	return 0;
}

static int mv88fx_machine_voice_startup(struct snd_pcm_substream *substream)
{
	return 0;
}




static int mv88fx_rt5630_init(struct snd_soc_codec *codec)
{
	mv88fx_snd_debug("");
//	mvmpp_gpio_set_value(MPP_Amp_PwrDn,0);
	return 0;
}

/* machine stream operations */
static struct snd_soc_ops mv88fx_rt5630_machine_ops[] ={
	{
		.startup = mv88fx_machine_hifi_startup,
		.shutdown = mv88fx_machine_hifi_shutdown,
		.hw_params = mv88fx_machine_hifi_hw_params,
		.trigger = mv88fx_machine_hifi_trigger,
	},
};

static struct snd_soc_dai_link d2plug_headphone_dai = {
	.name = "RT5630",
	.stream_name = "RT5630 HIFI",
	.platform_name = "mv88fx-pcm-audio.0",
	.cpu_dai_name = "mv88fx-i2s.0",
	//.cpu_dai = &mv88fx_i2s_dai0,
	.codec_dai_name = "rt5630-hifi",
	.codec_name = "rt5630-codec.0-001f", 
	//.codec_dai = &rt5630_dai[0],
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS, //@@@@@
	.ops = &mv88fx_rt5630_machine_ops,
};


static struct snd_soc_dai_link d2plug_hdmi_dai = {
	.name = "TDA19988",
	.stream_name = "TDA19988 HIFI",
	.platform_name = "mv88fx-pcm-audio.1",
	.cpu_dai_name = "mv88fx-i2s.1",
	//.cpu_dai = &mv88fx_i2s_dai1,
	//.codec_dai = &tda19988_dai,
	.codec_dai_name = "tda19988-hifi",
	//.codec_name = "tda19988-codec.0-0070",
	.codec_name = "tda19988codec",
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS, //@@@@@
};


static int mv88fx_probe(struct snd_soc_card *pdev)
{
	return 0;
}

static int mv88fx_remove(struct snd_soc_card *pdev)
{
	return 0;
}

static struct snd_soc_card d2plug_card[] = {
	{
		.name = "D2PLUG-I2S0-AUDIOJACK",
		//.platform = &mv88fx_soc_platform,
		.probe = mv88fx_probe,
		.remove = mv88fx_remove,

		/* CPU <--> Codec DAI links */
		.dai_link = &d2plug_headphone_dai,
		.num_links = 1,
	},
	{
		.name = "D2PLUG-I2S1-HDMI",
		//.platform = &mv88fx_soc_platform,
		.probe = mv88fx_probe,
		.remove = mv88fx_remove,

		/* CPU <--> Codec DAI links */
		.dai_link = &d2plug_hdmi_dai,
		.num_links = 1,
	}
};

/*
struct platform_device tda19988_codec_dev = {
	.name           = "tda19988-codec",
	.id             = -1,
};

struct platform_device rt5630_codec_dev = {
	.name           = "rt5630-codec",
	.id             = -1,
};
*/

static int mv88fx_initalize_machine_data(struct platform_device *pdev)
{
	struct resource *r = NULL;
	int err = 0;

	mv88fx_snd_debug("mv88fx_initalize_machine_data");

	mv88fx_machine_data[pdev->id].port = pdev->id;
	mv88fx_machine_data[pdev->id].pdata =
		(struct orion_i2s_platform_data *)pdev->dev.platform_data;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		err = -ENXIO;
		goto error;
	}
	r = request_mem_region(r->start, SZ_16K, DRIVER_NAME);
	if (!r) {
		err = -EBUSY;
		goto error;
	}
	mv88fx_machine_data[pdev->id].res = r;
	mv88fx_machine_data[pdev->id].base = ioremap(r->start, SZ_16K);

	if (!mv88fx_machine_data[pdev->id].base) {
		mv88fx_snd_error("ioremap failed");
		err = -ENOMEM;
		goto error;
	}
	mv88fx_machine_data[pdev->id].base -= MV_AUDIO_REGS_OFFSET(mv88fx_machine_data[pdev->id].port);

	mv88fx_machine_data[pdev->id].irq = platform_get_irq(pdev, 0);
	if (mv88fx_machine_data[pdev->id].irq == NO_IRQ) {
		err = -ENXIO;
		goto error;
	}
#if defined(CONFIG_HAVE_CLK)
	mv88fx_machine_data[pdev->id].clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(mv88fx_machine_data[pdev->id].clk))
		dev_notice(&pdev->dev, "cannot get clkdev\n");
	else
		clk_enable(mv88fx_machine_data[pdev->id].clk);
#endif
	return 0;
error:
	if (mv88fx_machine_data[pdev->id].base) {
		iounmap(mv88fx_machine_data[pdev->id].base);
		mv88fx_machine_data[pdev->id].base = NULL;
	}
	release_mem_region(mv88fx_machine_data[pdev->id].res->start, SZ_16K);
	return err;
}

static int mv88fx_snd_probe(struct platform_device *pdev)
{
	int ret = 0;
	mv88fx_snd_debug("1. mv88fx_snd_probe() is called, machine_arch_type is %d\n", machine_arch_type);

	if (mv88fx_initalize_machine_data(pdev) != 0)
		goto error;

	mv88fx_machine_data[pdev->id].snd_dev = platform_device_alloc("soc-audio", pdev->id);
	if (!mv88fx_machine_data[pdev->id].snd_dev) {
		ret = -ENOMEM;
		goto error;
	}

	if ( machine_is_d2plug() )
	{
		mv88fx_snd_debug("2. mv88fx_snd_probe(): snd_dev is 0x%x \n",mv88fx_machine_data[pdev->id].snd_dev);
		if (pdev->id) {
			platform_set_drvdata(mv88fx_machine_data[pdev->id].snd_dev, &d2plug_card[1]);
			//d2plug_hdmi.dev = &mv88fx_machine_data[pdev->id].snd_dev->dev;
		} else {
			platform_set_drvdata(mv88fx_machine_data[pdev->id].snd_dev, &d2plug_card[0]);
			//d2plug_headphone.dev = &mv88fx_machine_data[pdev->id].snd_dev->dev;
		}
	}

	mv88fx_machine_data[pdev->id].snd_dev->dev.platform_data = &mv88fx_machine_data[pdev->id];  //@@@@
	 //for example: in function mv88fx_i2s_dai_probe() will use the platform_date

	ret = platform_device_add(mv88fx_machine_data[pdev->id].snd_dev);

	if(ret)
	{
          platform_device_put(mv88fx_machine_data[pdev->id].snd_dev);
	}

	/* ID 1 for D2plug is for HDMI */
	/*
	if ( machine_is_d2plug() && pdev->id)
	{
		return platform_device_register(&tda19988_codec_dev);
	}

	ret = platform_device_register(&rt5630_codec_dev);
	if(ret)
                platform_device_unregister(&rt5630_codec_dev);
        */
	return ret;
error:
	mv88fx_snd_error("");
#if defined(CONFIG_HAVE_CLK)
	if (!IS_ERR(mv88fx_machine_data[pdev->id].clk)) {
		clk_disable(mv88fx_machine_data[pdev->id].clk);
		clk_put(mv88fx_machine_data[pdev->id].clk);
	}
#endif
	if (mv88fx_machine_data[pdev->id].snd_dev)
		platform_device_unregister(mv88fx_machine_data[pdev->id].snd_dev);
	return ret;

}

static int mv88fx_snd_remove(struct platform_device *dev)
{
	mv88fx_snd_debug("");
#if defined(CONFIG_HAVE_CLK)
	if (!IS_ERR(mv88fx_machine_data[dev->id].clk)) {
		clk_disable(mv88fx_machine_data[dev->id].clk);
		clk_put(mv88fx_machine_data[dev->id].clk);
	}
#endif
	mv88fx_machine_data[dev->id].snd_dev->dev.platform_data = NULL;
	platform_device_unregister(mv88fx_machine_data[dev->id].snd_dev);
	release_mem_region(mv88fx_machine_data[dev->id].res->start, SZ_16K);
	/* ID 1 for D2plug is for HDMI */
	/*
	if (machine_is_d2plug() && dev->id)
		platform_device_unregister(&tda19988_codec_dev);
	else
		platform_device_unregister(&rt5630_codec_dev);
	*/
	return 0;
}

static struct platform_driver mv88fx_snd_driver = {
	.probe = mv88fx_snd_probe,
	.remove = mv88fx_snd_remove,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		   .name = DRIVER_NAME,
		   },

};

static int __init mv88fx_snd_init(void)
{
//	if (!machine_is_dove_rd_avng() && !machine_is_dove_avng_v3() && !machine_is_dove_d2plug() )
//		return -ENODEV;
	mv88fx_snd_debug("");
	return platform_driver_register(&mv88fx_snd_driver);
}

static void __exit mv88fx_snd_exit(void)
{
	mv88fx_snd_debug("");
	platform_driver_unregister(&mv88fx_snd_driver);

}

module_init(mv88fx_snd_init);
module_exit(mv88fx_snd_exit);

/* Module information */
MODULE_AUTHOR("Yuval Elmaliah <eyuval@marvell.com>");
MODULE_AUTHOR("Ethan Ku <eku@marvell.com>");
MODULE_AUTHOR("Brian Hsu <eku@marvell.com>");
MODULE_DESCRIPTION("ALSA SoC Dove");
MODULE_LICENSE("GPL");
