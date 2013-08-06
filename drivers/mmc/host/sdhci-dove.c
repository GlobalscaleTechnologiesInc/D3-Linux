/*
 * sdhci-dove.c Support for SDHCI on Marvell's Dove SoC
 *
 * Author: Saeed Bishara <saeed@marvell.com>
 *	   Mike Rapoport <mike@compulab.co.il>
 * Based on sdhci-cns3xxx.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/mmc/host.h>

#include "sdhci-pltfm.h"
#if 1

static void sdhci_sdio_gpio_irq_enable(struct sdhci_host *host);
static void sdhci_sdio_gpio_irq_disable(struct sdhci_host *host);
/*
static struct sdhci_ops sdhci_mv_ops = {
	.gpio_irq_enable = sdhci_sdio_gpio_irq_enable,
	.gpio_irq_disable = sdhci_sdio_gpio_irq_disable,
}; */

irqreturn_t sdhci_dove_gpio_irq(int irq, void *dev_id)
{
	struct sdhci_host* host = dev_id;
#ifdef VERBOSE
	DBG("*** %s got gpio interrupt\n",
		mmc_hostname(host->mmc));
#endif

#ifdef VERBOSE
	sdhci_dumpregs(host);
#endif
	mmc_signal_sdio_irq(host->mmc);

	return IRQ_HANDLED;
}


static void sdhci_enable_sdio_gpio_irq(struct mmc_host *mmc, int enable)
{
	struct sdhci_host *host = mmc_priv(mmc);
	struct sdhci_pltfm_host *mv_host = sdhci_priv(host);
	unsigned long flags;
	struct sdhci_dove_int_wa *wa_info;

	if (!mv_host->dove_card_int_wa)
		return;

	wa_info = &mv_host->dove_int_wa_info;
	spin_lock_irqsave(&host->lock, flags);

	if (enable) {
		if (wa_info->status == 0) {
			enable_irq(wa_info->irq);
			wa_info->status = 1;
		}
	} else {
		if (wa_info->status == 1) {
			disable_irq_nosync(wa_info->irq);
			wa_info->status = 0;
		}
	}
	
	mmiowb();

	spin_unlock_irqrestore(&host->lock, flags);
}

static void sdhci_sdio_gpio_irq_enable(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *mv_host = sdhci_priv(host);
	u32 mpp_ctrl4;
	if (!mv_host->dove_card_int_wa)
		return;

	mpp_ctrl4 = readl(DOVE_MPP_CTRL4_VIRT_BASE);
	mpp_ctrl4 |= 1 << mv_host->dove_int_wa_info.func_select_bit;
	writel(mpp_ctrl4, DOVE_MPP_CTRL4_VIRT_BASE);

	mmiowb();
}

static void sdhci_sdio_gpio_irq_disable(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *mv_host = sdhci_priv(host);
	u32 mpp_ctrl4;

	if (!mv_host->dove_card_int_wa)
		return;

	mpp_ctrl4 = readl(DOVE_MPP_CTRL4_VIRT_BASE);
	mpp_ctrl4 &= ~(1 << mv_host->dove_int_wa_info.func_select_bit);
	writel(mpp_ctrl4, DOVE_MPP_CTRL4_VIRT_BASE);

	mmiowb();
}

#endif
static u16 sdhci_dove_readw(struct sdhci_host *host, int reg)
{
	u16 ret;

	switch (reg) {
	case SDHCI_HOST_VERSION:
	case SDHCI_SLOT_INT_STATUS:
		/* those registers don't exist */
		return 0;
	default:
		ret = readw(host->ioaddr + reg);
	}
	return ret;
}

static u32 sdhci_dove_readl(struct sdhci_host *host, int reg)
{
	u32 ret;

	switch (reg) {
	case SDHCI_CAPABILITIES:
		ret = readl(host->ioaddr + reg);
		/* Mask the support for 3.0V */
		ret &= ~SDHCI_CAN_VDD_300; 
		break;
	default:
		ret = readl(host->ioaddr + reg);
	}
	return ret;
}

static struct sdhci_ops sdhci_dove_ops = {
	.gpio_irq_enable = sdhci_sdio_gpio_irq_enable,
	.gpio_irq_disable = sdhci_sdio_gpio_irq_disable,
	.read_w	= sdhci_dove_readw,
	.read_l	= sdhci_dove_readl,
};
/*
static struct sdhci_pltfm_data sdhci_dove_pdata = {
	.ops	= &sdhci_dove_ops,
	.quirks	= SDHCI_QUIRK_NO_SIMULT_VDD_AND_POWER |
		  SDHCI_QUIRK_NO_BUSY_IRQ |
		  SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_FORCE_DMA,
};
*/
static struct sdhci_pltfm_data sdhci_dove_pdata = {
	.ops	= &sdhci_dove_ops,
	.quirks = SDHCI_QUIRK_NO_SIMULT_VDD_AND_POWER |
			 SDHCI_QUIRK_NO_BUSY_IRQ |
			 SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
			 //SDHCI_QUIRK_PIO_NEEDS_DELAY |
			 //SDHCI_QUIRK_FORCE_DMA,
			 SDHCI_QUIRK_PIO_USE_WORD_ACCESS |
			 SDHCI_QUIRK_HIGH_SPEED_WA,
};

static int __devinit sdhci_dove_probe(struct platform_device *pdev)
{
	int ret;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *mv_host; 
	
	ret = sdhci_pltfm_register(pdev, &sdhci_dove_pdata);
	if (ret)  printk("sdhci_dove_probe() failed\n");

	host = (struct sdhci_host *)platform_get_drvdata(pdev);
	mv_host = sdhci_priv(host);
	if (mv_host->dove_card_int_wa)
		host->mmc->ops->enable_sdio_irq = sdhci_enable_sdio_gpio_irq;
	return ret;
}

static int __devexit sdhci_dove_remove(struct platform_device *pdev)
{
	return sdhci_pltfm_unregister(pdev);
}

static struct platform_driver sdhci_dove_driver = {
	.driver		= {
		.name	= "sdhci-dove",
		.owner	= THIS_MODULE,
		.pm	= SDHCI_PLTFM_PMOPS,
	},
	.probe		= sdhci_dove_probe,
	.remove		= __devexit_p(sdhci_dove_remove),
};

module_platform_driver(sdhci_dove_driver);

MODULE_DESCRIPTION("SDHCI driver for Dove");
MODULE_AUTHOR("Saeed Bishara <saeed@marvell.com>, "
	      "Mike Rapoport <mike@compulab.co.il>");
MODULE_LICENSE("GPL v2");
