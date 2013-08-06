/*
 * TDA19988 ALSA SoC (ASoC) codec driver
 *
 * Author: Tanmay Upadhyay <tanmay.upadhyay@einfochips.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * This is a dummy ASoC device driver for the NXP TDA19988 codec. This just
 * provides dai to bind to SOC dai. No actual functionality is implemented.
 * This is tested with Dove D2plug only & as of now depends highly on
 * plat-orion/mv_hal_drivers/mv_drivers_lsp/mv_audio_soc/mv88fx-rt5630.c which
 * creates platform device for the driver
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
/*
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/jiffies.h>
#include <asm/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <asm/div64.h>
*/

struct snd_soc_dai_driver tda19988_dai = {
	.name = "tda19988-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = (SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_96000),
		.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S32_LE),
	},
};
EXPORT_SYMBOL_GPL(tda19988_dai);

static int tda19988_init(struct snd_soc_codec *codec)
{
	int ret;
	codec->name = "TDA19988";
	//codec->name = "tda19988-codec.0-0070";
	//codec->owner = THIS_MODULE;
	//codec->dai = &tda19988_dai;
	//codec->num_dai = 1;

	//mutex_init(&codec->mutex);
	//INIT_LIST_HEAD(&codec->dapm_widgets);

	return 0;

}

static int tda19988_probe(struct snd_soc_codec *codec)
{
	struct snd_soc_card *card = codec->card;
	if (tda19988_init(codec) < 0) {
		printk(KERN_ERR "Failed to initialise TDA19988\n");
		kfree(codec);
		card->rtd->codec = NULL;
		return -ENODEV;
	}

	return 0;
}

static int tda19988_remove(struct snd_soc_codec *codec)
{
	return 0;
}
/*
struct snd_soc_codec_device soc_codec_dev_tda19988 = {
	.probe = 	tda19988_probe,
	.remove = 	tda19988_remove,
};
*/
struct snd_soc_codec_driver soc_codec_dev_tda19988 = {
		.probe =	tda19988_probe,
		.remove =	tda19988_remove,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_tda19988);

#if 1
static int __devinit tda19988_codecs_probe(struct platform_device *pdev)
{
	//tda19988_dai.dev = &pdev->dev;

	return snd_soc_register_codec(&pdev->dev, &soc_codec_dev_tda19988, &tda19988_dai, 1);
}


static struct platform_device tda19988_codecsdev = {
	.name = "tda19988codec",
	.id = -1,
};
static struct platform_driver tda19988_codecsdev_drv = {
	.probe	= tda19988_codecs_probe,
	.driver			= {
				   .name = "tda19988codec",
				   .owner = THIS_MODULE,
		},
};

static int __init tda19988_modinit(void)
{
	platform_device_register(&tda19988_codecsdev);
	return platform_driver_register(&tda19988_codecsdev_drv);
}

static void __exit tda19988_exit(void)
{
	platform_device_unregister(&tda19988_codecsdev);
	platform_driver_unregister(&tda19988_codecsdev_drv);
}
#endif

module_init(tda19988_modinit);
module_exit(tda19988_exit);
MODULE_DESCRIPTION("NXP TDA19988 _dummy_ ALSA SoC Codec Driver");
MODULE_LICENSE("GPL");
