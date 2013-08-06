/*
 * arch/arm/mach-dove/dove-db-setup.c
 *
 * Marvell DB-MV88AP510-BP Development Board Setup
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/nand.h>
#include <linux/timer.h>
#include <linux/ata_platform.h>
#include <linux/mv643xx_eth.h>
#include <linux/i2c.h>
#include <linux/pci.h>
#include <linux/spi/spi.h>
#include <linux/spi/orion_spi.h>
#include <linux/spi/flash.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/dove.h>
#include "common.h"
#include "pmu/mvPmu.h"
#include "pmu/mvPmuRegs.h"

#include "mpp.h"
#include <video/dovefb.h>
#include <video/dovefbreg.h>
#include <mach/dove_bl.h>
#include <linux/delay.h>
#include <linux/module.h>

#include <plat/i2s-orion.h>
#include <linux/adi9889_i2c.h>
#include "idt5v49ee503.h"
#include "clock.h"
#include <asm/hardware/pxa-dma.h>
#include "pdma/mvPdma.h"

#include <mach/pm.h>

#include <linux/leds.h>

extern unsigned int useHalDrivers;
extern char *useNandHal;

static unsigned int standby_fix = 1;
module_param(standby_fix, uint, 0);
MODULE_PARM_DESC(standby_fix, "if 1 then CKE and MRESET are connected to MPP4 and MPP6");


static unsigned int use_hal_giga = 0;
#ifdef CONFIG_MV643XX_ETH
module_param(use_hal_giga, uint, 0);
MODULE_PARM_DESC(use_hal_giga, "Use the HAL giga driver");
#endif

static int dvs_enable = 0;
module_param(dvs_enable, int, 1);
MODULE_PARM_DESC(dvs_enable, "if 1 then enable DVS");

/*****************************************************************************
 * MPP
 ****************************************************************************/
static unsigned int dove_d2plug_a0_mpp_modes[] __initdata = {
		MPP0_GPIO0,	/*LED_D0 */
		MPP1_GPIO1,	/*LED_D1 */
		MPP2_GPIO2,	/*LED_D2 */
		MPP3_GPIO3,	/* PM_Switch_Pressed				   */
		MPP4_GPIO4,	/* M_CKE_MASK	  */
		MPP5_GPIO5,		/* PM_CPU_EN	*/
		MPP6_GPIO6,		/*M_RST_MASK		   */
		MPP7_GPIO7,	/* LED_STBY_ONn 		 */
		MPP8_GPIO8,	/* OFF_CTRL */
		MPP9_GPIO9,	/* CORE_DVS_CTR   */
		MPP10_GPIO10,	/* CPU_DVS_CTRL 	*/
		MPP11_GPIO11,	/* LED_SATA_ONn 	*/
		MPP12_GPIO12,	/* GPIO */
		MPP13_GPIO13,	/* WLAN_WAKEUP_HOST */
		MPP14_GPIO14,	/* GPIO 		  */
		MPP15_GPIO15,	/* GPIO 	 */
		MPP16_GPIO16,	/* GPIO 		*/
			
		MPP17_TW_SDA2,	/* I2C_1_SDA	  */
		MPP18_GPIO18,	/* LCD0_PWM   */
		MPP19_TW_SCK2,	/* I2C_1_SCK  */
			
		//should be GPIO
		MPP20_GPIO20,
		MPP21_GPIO21,
		MPP22_GPIO22,
		MPP23_GPIO23,
		/*MPP20_SPI_1_MISO,  //SPI
		MPP21_SPI_1_CS,    //SPI
		MPP22_SPI_1_MOSI,  //SPI
		MPP23_SPI_1_SCK,   //SPI */
		0

};

static unsigned int dove_d2plug_a0_mppgrp_modes[] __initdata = {
	MPP_GRP_24_39_GPIO,
	MPP_GRP_40_45_SD0,
	MPP_GRP_46_51_SD1,
	MPP_GRP_58_61_SPI,
	MPP_GRP_62_63_UA1,
	0
};


/*****************************************************************************
 * LCD
 ****************************************************************************/
/*
 * LCD HW output Red[0] to LDD[0] when set bit [19:16] of reg 0x190
 * to 0x0. Which means HW outputs BGR format default. All platforms
 * uses this controller should enable .panel_rbswap. Unless layout
 * design connects Blue[0] to LDD[0] instead.
 */
static struct dovefb_mach_info dove_d2plug_lcd0_dmi = {
	.id_gfx			= "GFX Layer 0",
	.id_ovly		= "Video Layer 0",
	.clk_src		= MRVL_EXT_CLK1,
	//.clk_name		= "IDT_CLK1",
	.clk_name		= "AKOSC_CLK",
//	.num_modes		= ARRAY_SIZE(video_modes),
//	.modes			= video_modes,
	.pix_fmt		= PIX_FMT_RGB888PACK,
	.io_pin_allocation	= IOPAD_DUMB24,
	.panel_rgb_type		= DUMB24_RGB888_0,
	.panel_rgb_reverse_lanes= 0,
	.gpio_output_data	= 0,
	.gpio_output_mask	= 0,
#ifndef CONFIG_FB_DOVE_CLCD0_I2C_DEFAULT_SETTING
	//.ddc_i2c_adapter	= CONFIG_FB_DOVE_CLCD0_I2C_CHANNEL,
	//.ddc_i2c_address	= CONFIG_FB_DOVE_CLCD0_I2C_ADDRESS,
	.ddc_i2c_adapter	= 0,
	.ddc_i2c_address	= 0x3f,
#else
	.ddc_i2c_adapter	= 0,
	.ddc_i2c_address	= 0x3f,
#endif
	.invert_composite_blank	= 0,
	.invert_pix_val_ena	= 0,
	.invert_pixclock	= 0,
	.invert_vsync		= 0,
	.invert_hsync		= 0,
	.panel_rbswap		= 1,
	.active			= 1,
};

static struct dovefb_mach_info dove_d2plug_lcd0_vid_dmi = {
	.id_ovly		= "Video Layer 0",
//	.num_modes		= ARRAY_SIZE(video_modes),
//	.modes			= video_modes,
	.pix_fmt		= PIX_FMT_RGB888PACK,
	.io_pin_allocation	= IOPAD_DUMB24,
	.panel_rgb_type		= DUMB24_RGB888_0,
	.panel_rgb_reverse_lanes= 0,
	.gpio_output_data	= 0,
	.gpio_output_mask	= 0,
	.ddc_i2c_adapter	= -1,
	.invert_composite_blank	= 0,
	.invert_pix_val_ena	= 0,
	.invert_pixclock	= 0,
	.invert_vsync		= 0,
	.invert_hsync		= 0,
	.panel_rbswap		= 1,
	.active			= 0,
	.enable_lcd0		= 0,
};

static struct dovefb_mach_info dove_d2plug_lcd1_dmi = {
	.id_gfx			= "GFX Layer 1",
	.id_ovly		= "Video Layer 1",
	.clk_src		= MRVL_PLL_CLK,
	.clk_name		= "accurate_LCDCLK",
//	.num_modes		= ARRAY_SIZE(video_modes),
//	.modes			= video_modes,
	.pix_fmt		= PIX_FMT_RGB888PACK,
	.io_pin_allocation	= IOPAD_DUMB24,
	.panel_rgb_type		= DUMB24_RGB888_0,
	.panel_rgb_reverse_lanes= 0,
	.gpio_output_data	= 0,
	.gpio_output_mask	= 0,
#ifndef CONFIG_FB_DOVE_CLCD1_I2C_DEFAULT_SETTING
	.ddc_i2c_adapter	= 1,
	.ddc_i2c_address	= 0x50,
#else
	.ddc_i2c_adapter	= 1	,
	.ddc_i2c_address	= 0x50,
#endif
	.invert_composite_blank	= 0,
	.invert_pix_val_ena	= 0,
	.invert_pixclock	= 0,
	.invert_vsync		= 0,
	.invert_hsync		= 0,
	.panel_rbswap		= 1,
	.active			= 1,
#ifndef CONFIG_FB_DOVE_CLCD
	.enable_lcd0		= 1,
#else
	.enable_lcd0		= 0,
#endif
};

static struct dovefb_mach_info dove_d2plug_lcd1_vid_dmi = {
	.id_ovly		= "Video Layer 1",
//	.num_modes		= ARRAY_SIZE(video_modes),
//	.modes			= video_modes,
	.pix_fmt		= PIX_FMT_RGB888PACK,
	.io_pin_allocation	= IOPAD_DUMB24,
	.panel_rgb_type		= DUMB24_RGB888_0,
	.panel_rgb_reverse_lanes= 0,
	.gpio_output_data	= 0,
	.gpio_output_mask	= 0,
	.ddc_i2c_adapter	= -1,
	.invert_composite_blank	= 0,
	.invert_pix_val_ena	= 0,
	.invert_pixclock	= 0,
	.invert_vsync		= 0,
	.invert_hsync		= 0,
	.panel_rbswap		= 1,
	.active			= 0,
};


/*****************************************************************************
 * BACKLIGHT
 ****************************************************************************/
#define MV_GPP_REGS_OFFSET(grp)			((grp) ? 0xD0420:0xD0400)
#define MV_GPP_REGS_BASE(unit)		(MV_GPP_REGS_OFFSET(unit))
#define GPP_DATA_OUT_REG(grp)			(MV_GPP_REGS_BASE(grp) + 0x00)
static struct dovebl_platform_data dove_d2plug_backlight_data = {
	.default_intensity = 0xa,
	.gpio_pm_control = 1,

	.lcd_start = DOVE_SB_REGS_VIRT_BASE,	/* lcd power control reg base. */
	.lcd_end = DOVE_SB_REGS_VIRT_BASE+GPP_DATA_OUT_REG(0),	/* end of reg map. */
	.lcd_offset = GPP_DATA_OUT_REG(0),	/* register offset */
	.lcd_mapped = 1,		/* va = 0, pa = 1 */
	.lcd_mask = (1<<26),		/* mask, bit[11] */
	.lcd_on = (1<<26),		/* value to enable lcd power */
	.lcd_off = 0x0,			/* value to disable lcd power */

	.blpwr_start = DOVE_SB_REGS_VIRT_BASE, /* bl pwr ctrl reg base. */
	.blpwr_end = DOVE_SB_REGS_VIRT_BASE+GPP_DATA_OUT_REG(0),	/* end of reg map. */
	.blpwr_offset = GPP_DATA_OUT_REG(0),	/* register offset */
	.blpwr_mapped = 1,		/* pa = 0, va = 1 */
	.blpwr_mask = (1<<28),		/* mask, bit[13] */
	.blpwr_on = (1<<28),		/* value to enable bl power */
	.blpwr_off = 0x0,		/* value to disable bl power */

	.btn_start = DOVE_LCD1_PHYS_BASE, /* brightness control reg base. */
	.btn_end = DOVE_LCD1_PHYS_BASE+0x1C8,	/* end of reg map. */
	.btn_offset = LCD_CFG_GRA_PITCH,	/* register offset */
	.btn_mapped = 0,		/* pa = 0, va = 1 */
	.btn_mask = 0xF0000000,		/* mask */
	.btn_level = 15,		/* how many level can be configured. */
	.btn_min = 0x1,			/* min value */
	.btn_max = 0xF,			/* max value */
	.btn_inc = 0x1,			/* increment */
};


void __init dove_d2plug_clcd_init(void) {
#ifdef CONFIG_FB_DOVE
	clcd_platform_init(&dove_d2plug_lcd0_dmi, &dove_d2plug_lcd0_vid_dmi,
			   &dove_d2plug_lcd1_dmi, &dove_d2plug_lcd1_vid_dmi,
			   &dove_d2plug_backlight_data);

#endif /* CONFIG_FB_DOVE */
}

struct adi9889_i2c_platform_data avng_v3_adi9889_i2c_conf ={
	.audio_format = I2S_FORMAT,

};

/*****************************************************************************
 * I2C devices:
 * 	ALC5630 codec, address 0x
 * 	Battery charger, address 0x??
 * 	G-Sensor, address 0x??
 * 	MCU PIC-16F887, address 0x??
 ****************************************************************************/
static struct i2c_board_info __initdata dove_d2plug_i2c_bus0_devs[] = {
#if 0
	{
		I2C_BOARD_INFO("rt5630", 0x1f),
	},
	{
		I2C_BOARD_INFO("rt5623", 0x1a),
	},
#endif
	{
		//I2C_BOARD_INFO("tda19988", 0x70),
		I2C_BOARD_INFO("tda998X", 0x70),
	},
	{
		I2C_BOARD_INFO("tda99Xcec", 0x34),
	},
};

static struct i2c_board_info __initdata dove_d2plug_i2c_bus1_devs[] = {
#ifdef CONFIG_ADI9889
	{
		I2C_BOARD_INFO("adi9889_i2c", 0x39),
		.platform_data = &avng_v3_adi9889_i2c_conf,			
	},
	{
		I2C_BOARD_INFO("adi9889_edid_i2c", 0x3F),
		.platform_data = &avng_v3_adi9889_i2c_conf,			
	},
#endif

};

/*****************************************************************************
 * IDT clock
 ****************************************************************************/
static struct idt_data dove_uc2_idt_data = {
	/* clock 0 connected to pin LCD_EXT_REF_CLK[0]*/
	.clock0_enable = 1,
	.clock0_out_id = IDT_OUT_ID_2,
	.clock0_pll_id = IDT_PLL_1,
	/* clock 1 connected to pin LCD_EXT_REF_CLK[1]*/
	.clock1_enable = 1,
	.clock1_out_id = IDT_OUT_ID_3,
	.clock1_pll_id = IDT_PLL_2,
};

static struct i2c_board_info __initdata idt = {
	I2C_BOARD_INFO("idt5v49ee503", 0x6A),
	.platform_data = &dove_uc2_idt_data,
};


/*****************************************************************************
 * Ethernet
 ****************************************************************************/

static struct mv643xx_eth_platform_data dove_db_ge00_data = {
	//.phy_addr	= MV643XX_ETH_PHY_ADDR_DEFAULT,
	.phy_addr	= 1,
};


/*****************************************************************************
 * SATA
 ****************************************************************************/
static struct mv_sata_platform_data dove_db_sata_data = {
	.n_ports        = 1,
};


/*****************************************************************************
 * Audio I2S
 ****************************************************************************/
static struct orion_i2s_platform_data i2s0_data = {
	.i2s_play	= 1,
	.i2s_rec	= 1,
};

/* I2S1 is connected to HDMI which doesn't have record */
static struct orion_i2s_platform_data i2s1_data = {
	.i2s_play	= 1,
};


/*****************************************************************************
 * SPI Devices:
 * 	SPI0: 4M Flash ST-M25P32-VMF6P
 ****************************************************************************/
static const struct flash_platform_data dove_db_spi_flash_data = {
	.type		= "mx25l3205d",
};

static struct spi_board_info __initdata dove_db_spi_flash_info[] = {
	{
		.modalias       = "m25p80",
		.platform_data  = &dove_db_spi_flash_data,
		.irq            = -1,
		.max_speed_hz   = 20000000,
		.bus_num        = 0,
		.chip_select    = 0,
	},
};

/*****************************************************************************
 * GPIO
 *
 ***************************************************************************/
#define DOVE_AVNG_POWER_OFF_GPIO	(8)
static void dove_d2plug_power_off(void)
{
	if (gpio_direction_output(DOVE_AVNG_POWER_OFF_GPIO, 0) != 0) {
 		printk(KERN_ERR "%s failed to set power off output pin %d\n",
		       __func__, DOVE_AVNG_POWER_OFF_GPIO);
	}
}

static void dove_d2plug_gpio_init(u32 rev)
{
	int pin;
	unsigned long uiReg;

	printk(KERN_ERR"dove_d2plug_gpio_init ENTER!\n");
#if 0
	orion_gpio_set_valid(0, 1);
	if (gpio_request(0, "led0") != 0)
		printk(KERN_ERR "Dove: failed to setup GPIO for led0\n");
	gpio_direction_output(0,0);	/* led0 */
	
	orion_gpio_set_valid(1, 1);
	if (gpio_request(1, "led1") != 0)
		printk(KERN_ERR "Dove: failed to setup GPIO for led1\n");
	gpio_direction_output(1,0);	/* led1 */
	
	orion_gpio_set_valid(2, 1);
	if (gpio_request(2, "led2") != 0)
		printk(KERN_ERR "Dove: failed to setup GPIO for led2\n");
	gpio_direction_output(2,0);	/* led2 */
#endif  //because registered in leds-gpio
	if (rev >= DOVE_REV_X0) {
		pin = 16;
	} else {
		pin = 6;
	}
#if 0   //for ddr reset
	orion_gpio_set_valid(pin, 1);
	if (gpio_request(pin, "MPP_DDR_TERM") != 0)
	printk(KERN_ERR "Dove: failed to setup GPIO for MPP_DDR_TERM\n");
	gpio_direction_output(pin, 1);	/* Enable DDR 1.8v */
#endif

#if 1
	 orion_gpio_set_valid(5, 1);  //for pex1 rst
	 if (gpio_request(5, "pex1-rst") != 0)
		 printk(KERN_ERR "Dove: failed to setup GPIO for pex1-rst\n");
	 gpio_direction_input(5);	 /* pex1-rst */
#endif

	 orion_gpio_set_valid(7, 1);  //for pex0 rst; usb 3.0
	if (gpio_request(7, "pex0-rst" ) != 0)
		printk(KERN_ERR "Dove: failed to setup GPIO for pex0-rst\n");
	gpio_direction_output(7,1);	/*PCI-E reset */

	 orion_gpio_set_valid(8, 1);  //for usb pwr;
	if (gpio_request(8, "usb-pwr" ) != 0)
		printk(KERN_ERR "Dove: failed to setup GPIO for usb-pwr\n");
	gpio_direction_output(8,1);	/*PCI-E reset */

	orion_gpio_set_valid(14, 1);
	if (gpio_request(14, "HDMI_CKSW") != 0)
		printk(KERN_ERR "Dove: failed to setup GPIO for HDMI_CKSW\n");
	else
		gpio_direction_output(14, 0);   /* Allow HDMI clk */

	orion_gpio_set_valid(15, 1);
	if (gpio_request(15, "HDMI_CKSWET1") != 0)
		printk(KERN_ERR "Dove: failed to setup GPIO for HDMI_CKSWET1\n");
	else
		gpio_direction_output(15, 1); /* 148.5 MHz for HDMI*/
	
	orion_gpio_set_valid(16, 1);
	if (gpio_request(16, "HDMI_CKSWET2") != 0)
		printk(KERN_ERR "Dove: failed to setup GPIO for HDMI_CKSWET2\n");
	else
		gpio_direction_output(16, 0); /* 148.5 MHz for HDMI*/

	orion_gpio_set_valid(23, 1);
	if (gpio_request(23, "WLAN_PD") != 0)
		printk(KERN_ERR "Dove: failed to setup GPIO for WLAN_PD\n");
	else
	{
		gpio_direction_output(23, 0);
		mdelay(20);
		gpio_direction_output(23, 1); /* WLAN_PD*/
	}
	
   /////////////////////////// input part
	orion_gpio_set_valid(9, 1);
	if (gpio_request(9, "SRD1_CLKREQn") != 0)
		printk(KERN_ERR "Dove: failed to setup GPIO for SRD1_CLKREQn\n");
	else
		gpio_direction_input(9);

#if 0
	orion_gpio_set_valid(10, 1);
	if (gpio_request(10, "HDMI_INT") != 0)
		printk(KERN_ERR "Dove: failed to setup GPIO for HDMI_INT\n");
	else
		gpio_direction_input(10);
#endif

	orion_gpio_set_valid(11, 1);
	if (gpio_request(11, "GPIO_RST_SWIN") != 0)
		printk(KERN_ERR "Dove: failed to setup GPIO for GPIO_RST_SWIN\n");
	else
		gpio_direction_input(11);
	
	orion_gpio_set_valid(12, 1);
	if (gpio_request(12, "SRD1_WAKEn") != 0)
		printk(KERN_ERR "Dove: failed to setup GPIO for SRD1_WAKEn\n");
	else
		gpio_direction_input(12);

	orion_gpio_set_valid(13, 1);
	if (gpio_request(13, "USB_OVC") != 0)
		printk(KERN_ERR "Dove: failed to setup GPIO for USB_OVC\n");
	else
		gpio_direction_input(13);


	msleep(200);	
 	printk(KERN_ERR"dove_d2plug_gpio_init exit!\n");

}

#ifdef CONFIG_PM
extern int global_dvs_enable;
/*****************************************************************************
 * POWER MANAGEMENT
 ****************************************************************************/
static int __init dove_d2plug_pm_init(void)
{
	MV_PMU_INFO pmuInitInfo;
	u32 dev, rev;

	global_dvs_enable = dvs_enable;

	dove_pcie_id(&dev, &rev);

	pmuInitInfo.batFltMngDis = MV_FALSE;			/* Keep battery fault enabled */
	pmuInitInfo.exitOnBatFltDis = MV_FALSE;			/* Keep exit from STANDBY on battery fail enabled */
	pmuInitInfo.sigSelctor[0] = PMU_SIGNAL_NC;
	pmuInitInfo.sigSelctor[1] = PMU_SIGNAL_NC;
	pmuInitInfo.sigSelctor[2] = PMU_SIGNAL_SLP_PWRDWN;	/* STANDBY => 0: I/O off, 1: I/O on */
	pmuInitInfo.sigSelctor[3] = PMU_SIGNAL_EXT0_WKUP;	/* power on push button */
	if (rev >= DOVE_REV_X0) { /* For X0 and higher Power Good indication is not needed */
		if (standby_fix)
			pmuInitInfo.sigSelctor[4] = PMU_SIGNAL_CKE_OVRID;	/* CKE controlled by Dove */
		else
			pmuInitInfo.sigSelctor[4] = PMU_SIGNAL_NC;
	}
	else
		pmuInitInfo.sigSelctor[4] = PMU_SIGNAL_CPU_PWRGOOD;	/* CORE power good used as Standby PG */

	pmuInitInfo.sigSelctor[5] = PMU_SIGNAL_CPU_PWRDWN;	/* DEEP-IdLE => 0: CPU off, 1: CPU on */

	if ((rev >= DOVE_REV_X0) && (standby_fix)) /* For boards with X0 we use MPP6 as MRESET */
		pmuInitInfo.sigSelctor[6] = PMU_SIGNAL_MRESET_OVRID;		/* M_RESET is pulled up - always HI */
	else
		pmuInitInfo.sigSelctor[6] = PMU_SIGNAL_NC;
	pmuInitInfo.sigSelctor[7] = PMU_SIGNAL_1;		/* Standby Led - inverted */
	pmuInitInfo.sigSelctor[8] = PMU_SIGNAL_NC;
	pmuInitInfo.sigSelctor[9] = PMU_SIGNAL_NC;		/* CPU power good  - not used */
	pmuInitInfo.sigSelctor[10] = PMU_SIGNAL_SDI;		/* Voltage regulator control */
	pmuInitInfo.sigSelctor[11] = PMU_SIGNAL_NC;
	pmuInitInfo.sigSelctor[12] = PMU_SIGNAL_NC;
	pmuInitInfo.sigSelctor[13] = PMU_SIGNAL_NC;
	pmuInitInfo.sigSelctor[14] = PMU_SIGNAL_NC;
	pmuInitInfo.sigSelctor[15] = PMU_SIGNAL_NC;
	pmuInitInfo.dvsDelay = 0x4200;				/* ~100us in 166MHz cc - delay for DVS change */
	if (rev >= DOVE_REV_X0) { /* For X0 and higher wait at least 150ms + spare */
		pmuInitInfo.standbyPwrDelay = 0x2000;		/* 250ms delay to wait for complete powerup */
		pmuInitInfo.ddrTermGpioNum = 16;		/* GPIO 16 used to disable terminations */
	} else {
		pmuInitInfo.standbyPwrDelay = 0x140;		/* 10ms delay after getting the power good indication */
		pmuInitInfo.ddrTermGpioNum = 6;			/* GPIO 6 used to disable terminations */
	}

	/* Initialize the PMU HAL */
	if (mvPmuInit(&pmuInitInfo) != MV_OK)
	{
		printk(KERN_ERR "ERROR: Failed to initialise the PMU!\n");
		return 0;
	}

	/* Configure wakeup events */
	mvPmuWakeupEventSet(PMU_STBY_WKUP_CTRL_EXT0_FALL | PMU_STBY_WKUP_CTRL_RTC_MASK);

	/* Register the PM operation in the Linux stack */
	dove_pm_register();

	return 0;
}

__initcall(dove_d2plug_pm_init);
#endif


#ifdef CONFIG_BATTERY_MCU
void __init dove_battery_init_v3(void)
{
	platform_device_register_simple("battery", 0, NULL, 0);
}
#endif

/*****************************************************************************
 * PCI
 ****************************************************************************/
static int __init dove_d2plug_pci_init(void)
{
	//if (machine_is_dove_db())
	//dove_pcie_init(1, 1);
	dove_pcie_init(1, 1);
	return 0;
}

subsys_initcall(dove_d2plug_pci_init);

/*****************************************************************************
 * gpio leds
 ****************************************************************************/
static struct gpio_led plug_led_pins[] = {
		{
			.name			= "d1",
			.gpio			= 0,
			.active_low 	= 0,
		},
		{
			.name			= "d2",
			.gpio			= 1,
			.active_low 	= 0,
		},
		{
			.name			= "d3",
			.gpio			= 2,
			.active_low 	= 0,
		},
};
	
static struct gpio_led_platform_data plug_led_data = {
		.leds		= plug_led_pins,
		.num_leds	= ARRAY_SIZE(plug_led_pins),
};
	
static struct platform_device plug_leds = {
		.name	= "leds-gpio",
		.id = -1,
		.dev	= {
			.platform_data	= &plug_led_data,
		}
};

/*****************************************************************************
 * Board Init
 ****************************************************************************/
extern struct mbus_dram_target_info orion_mbus_dram_info;
extern int __init pxa_init_dma_wins(struct mbus_dram_target_info *dram);

static void __init dove_db_init(void)
{
	/*
	 * Basic Dove setup. Needs to be called early.
	 */
	u32 dev, rev;
	dove_init();
	
	dove_pcie_id(&dev, &rev);
	
	dove_mpp_conf(dove_d2plug_a0_mpp_modes, dove_d2plug_a0_mppgrp_modes, 
		MPP_GRP_AU1_52_57_AU1, MPP_GRP_NFC_64_71_NFC);
	
	dove_d2plug_gpio_init(rev);
	
	pm_power_off = dove_d2plug_power_off;
	
	 /* sdio card interrupt workaround using GPIOs */
	dove_sd_card_int_wa_setup(0);
	dove_sd_card_int_wa_setup(1);

	pxa_init_dma_wins(&orion_mbus_dram_info);
	pxa_init_dma(16);
#ifdef CONFIG_MV_HAL_DRIVERS_SUPPORT
		if (useHalDrivers || useNandHal) {
			if (mvPdmaHalInit(MV_PDMA_MAX_CHANNELS_NUM) != MV_OK) {
				printk(KERN_ERR "mvPdmaHalInit() failed.\n");
				BUG();
			}
			/* reserve channels for NAND Data and command PDMA */
			pxa_reserve_dma_channel(MV_PDMA_NAND_DATA);
			pxa_reserve_dma_channel(MV_PDMA_NAND_COMMAND);
		}
#endif

#ifdef CONFIG_MV_ETHERNET
			if (use_hal_giga || useHalDrivers)
				dove_mv_eth_init();
			else
#endif
	dove_ge00_init(&dove_db_ge00_data);
	 
	dove_ehci0_init();
	dove_ehci1_init();

	//ds_clks_disable_all(0, 0);
	dove_sata_init(&dove_db_sata_data);
	
	dove_sdio0_init();
	dove_sdio1_init();
	//dove_spi0_init();  //it is called in following place.
	//dove_spi1_init();
	//dove_d2plug_nfc_init();
	
	dove_uart0_init();
	dove_uart1_init();

	dove_d2plug_clcd_init();
	dove_vmeta_init();
	dove_gpu_init();
	
	dove_cesa_init();
	dove_hwmon_init();

	//dove_i2s_init(0, &i2s0_data);       //for audio jack; (d3plug)
	dove_i2s_init(1, &i2s1_data); //for hdmi
	
	dove_i2c_init();
	dove_i2c_exp_init(0);
	//if (rev >= DOVE_REV_X0) {  dove_i2c_exp_init(1);  } 

	i2c_register_board_info(0, dove_d2plug_i2c_bus0_devs,
					ARRAY_SIZE(dove_d2plug_i2c_bus0_devs));
	//i2c_register_board_info(1, dove_d2plug_i2c_bus1_devs, ARRAY_SIZE(dove_d2plug_i2c_bus1_devs));
	//if (rev >= DOVE_REV_A0) i2c_register_board_info(0, &idt, 1);
	
	dove_spi0_init();
	spi_register_board_info(dove_db_spi_flash_info,
				ARRAY_SIZE(dove_db_spi_flash_info));
	
	platform_device_register(&plug_leds);
#ifdef CONFIG_BATTERY_MCU
		dove_battery_init_v3();
#endif
	//ds_clks_enable_all();
	printk(KERN_INFO"ENd of INIT ***************************\r\n");
	
}

MACHINE_START(D2PLUG, "Marvell DB-MV88AP510-BP Development Board")
	.atag_offset	= 0x100,
	.init_machine	= dove_db_init,
	.map_io		= dove_map_io,
	.init_early	= dove_init_early,
	.init_irq	= dove_init_irq,
	.timer		= &dove_timer,
	.restart	= dove_restart,
	/* reserve memory for VMETA and GPU */
	.fixup		= dove_tag_fixup_mem32,
	.reserve	= dove_reserve_mem,
MACHINE_END
