/*
 * arch/arm/mach-dove/common.c
 *
 * Core functions for Marvell Dove 88AP510 System On Chip
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/clk.h>
#include <linux/ata_platform.h>
#include <linux/gpio.h>
#include <asm/page.h>
#include <asm/setup.h>
#include <asm/memblock.h>
#include <asm/timex.h>
#include <asm/hardware/cache-tauros2.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/mach/pci.h>
#include <mach/dove.h>
#include <mach/bridge-regs.h>
#include <asm/mach/arch.h>
#include <linux/irq.h>

#include <plat/time.h>
#include <plat/ehci-orion.h>
#include <plat/i2s-orion.h>
#include <plat/common.h>
#include <plat/addr-map.h>
#include <plat/orion_nand.h>
#include <plat/mv_eth.h>
#include <plat/mv_cesa.h>

#include "mvTypes.h"
#include <linux/mv643xx_i2c.h>
#include <linux/platform_data/mv_usb.h>

#include <ctrlEnv/mvCtrlEnvRegs.h>
#include "ctrlEnv/sys/mvCpuIfRegs.h"

#include "common.h"
#include "mach/pm.h"

#include <linux/dove_sdhci.h>

#ifdef CONFIG_MV_HAL_DRIVERS_SUPPORT
#include "mvTypes.h"
#endif
#ifdef CONFIG_MV_ETHERNET
#include "../plat-orion/mv_hal_drivers/mv_drivers_lsp/mv_network/mv_ethernet/mv_netdev.h"
#endif


static unsigned int dove_vmeta_memory_start;
static unsigned int dove_gpu_memory_start;


#ifdef CONFIG_UIO_VMETA
#define UIO_DOVE_VMETA_MEM_SIZE (CONFIG_UIO_DOVE_VMETA_MEM_SIZE << 20)
#else
#define UIO_DOVE_VMETA_MEM_SIZE 0
#endif

static unsigned int vmeta_size = UIO_DOVE_VMETA_MEM_SIZE;

static int __init vmeta_size_setup(char *str)
{
	get_option(&str, &vmeta_size);

	if (!vmeta_size)
		return 1;

	vmeta_size <<= 20;

	return 1;
}
__setup("vmeta_size=", vmeta_size_setup);


/* used for memory allocation for the GPU graphics engine */
#ifdef CONFIG_DOVE_GPU
#define DOVE_GPU_MEM_SIZE (CONFIG_DOVE_GPU_MEM_SIZE << 20)
#else
#define DOVE_GPU_MEM_SIZE 0
#endif

static unsigned int gpu_size = DOVE_GPU_MEM_SIZE;

static int __init gpu_size_setup(char *str)
{
	get_option(&str, &gpu_size);

	if (!gpu_size)
		return 1;

	gpu_size <<= 20;

	return 1;
}
__setup("gpu_size=", gpu_size_setup);

unsigned int __initdata pvt_size = 0;

static int __init pvt_size_setup(char *str)
{
	get_option(&str, &pvt_size);

	if (!pvt_size)
		return 1;

	pvt_size <<= 20;

	return 1;
}
__setup("pvt_size=", pvt_size_setup);

char *useNandHal = NULL;
static int __init useNandHal_setup(char *s)
{
	useNandHal = s;
	return 1;
}
__setup("useNandHal=", useNandHal_setup);

int useHalDrivers = 0;
#ifdef CONFIG_MV_HAL_DRIVERS_SUPPORT
static int __init useHalDrivers_setup(char *__unused)
{
     useHalDrivers = 1;
     return 1;
}
__setup("useHalDrivers", useHalDrivers_setup);
#endif

#ifdef CONFIG_MV_INCLUDE_USB
//#include "mvSysUsbApi.h"
MV_STATUS mvSysUsbInit(MV_U32 dev, MV_BOOL isHost);
/* Required to get the configuration string from the Kernel Command Line */
static char *usb0Mode = "host";
static char *usb1Mode = "host";
static char *usb_dev_name  = "mv-udc";
int mv_usb0_cmdline_config(char *s);
int mv_usb1_cmdline_config(char *s);
__setup("usb0Mode=", mv_usb0_cmdline_config);
__setup("usb1Mode=", mv_usb1_cmdline_config);

int mv_usb0_cmdline_config(char *s)
{
    usb0Mode = s;
    return 1;
}

int mv_usb1_cmdline_config(char *s)
{
    usb1Mode = s;
    return 1;
}

#endif 

static int noL2 = 0;
static int __init noL2_setup(char *__unused)
{
     noL2 = 1;
     return 1;
}

__setup("noL2", noL2_setup);

int pm_disable = 0;
static int __init pm_disable_setup(char *__unused)
{
     pm_disable = 1;
     return 1;
}

__setup("pm_disable", pm_disable_setup);

static int __init pm_enable_setup(char *__unused)
{
     pm_disable = 0;
     return 1;
}

__setup("pm_enable", pm_enable_setup);


int cpufreq_disable = 0;
static int __init cpufreq_disable_setup(char *__unused)
{
     cpufreq_disable = 1;
     return 1;
}

__setup("cpufreq_disable", cpufreq_disable_setup);

#ifdef CONFIG_PM
enum orion_dwnstrm_conf_save_state {
	/* CPU Configuration Registers */
	DOVE_DWNSTRM_BRDG_CPU_CONFIG = 0,
	DOVE_DWNSTRM_BRDG_CPU_CONTROL,
	DOVE_DWNSTRM_BRDG_RSTOUTn_MASK,
	DOVE_DWNSTRM_BRDG_BRIDGE_MASK,
	DOVE_DWNSTRM_BRDG_POWER_MANAGEMENT,
	/* CPU Timers Registers */
	DOVE_DWNSTRM_BRDG_TIMER_CTRL,
	DOVE_DWNSTRM_BRDG_TIMER0_RELOAD,
	DOVE_DWNSTRM_BRDG_TIMER1_RELOAD,
	DOVE_DWNSTRM_BRDG_TIMER_WD_RELOAD,
	/* Main Interrupt Controller Registers */
	DOVE_DWNSTRM_BRDG_IRQ_MASK_LOW,
	DOVE_DWNSTRM_BRDG_FIQ_MASK_LOW,
	DOVE_DWNSTRM_BRDG_ENDPOINT_MASK_LOW,
	DOVE_DWNSTRM_BRDG_IRQ_MASK_HIGH,
	DOVE_DWNSTRM_BRDG_FIQ_MASK_HIGH,
	DOVE_DWNSTRM_BRDG_ENDPOINT_MASK_HIGH,
	DOVE_DWNSTRM_BRDG_PCIE_INTERRUPT_MASK,	

	DOVE_DWNSTRM_BRDG_SIZE
};

#define DOVE_DWNSTRM_BRDG_SAVE(x) \
	dove_downstream_regs[DOVE_DWNSTRM_BRDG_##x] = readl(x)
#define DOVE_DWNSTRM_BRDG_RESTORE(x) \
	writel(dove_downstream_regs[DOVE_DWNSTRM_BRDG_##x], x)

static u32 dove_downstream_regs[DOVE_DWNSTRM_BRDG_SIZE];

enum orion_uptrm_conf_save_state {
	/* Upstream Bridge Configuration Registers */
	DOVE_UPSTRM_AXI_P_D_CTRL_REG = 0,
	DOVE_UPSTRM_D2X_ARB_LO_REG,
	DOVE_UPSTRM_D2X_ARB_HI_REG,

	DOVE_UPSTRM_BRDG_SIZE
};

#define DOVE_UPSTRM_BRDG_SAVE(x) \
	dove_upstream_regs[DOVE_##x] = readl(DOVE_SB_REGS_VIRT_BASE | x)
#define DOVE_UPSTRM_BRDG_RESTORE(x) \
	writel(dove_upstream_regs[DOVE_##x], (DOVE_SB_REGS_VIRT_BASE | x))

static u32 dove_upstream_regs[DOVE_UPSTRM_BRDG_SIZE];
#endif


static int get_tclk(void);

/*****************************************************************************
 * I/O Address Mapping
 ****************************************************************************/
static struct map_desc dove_io_desc[] __initdata = {
	{
		.virtual	= DOVE_SCRATCHPAD_VIRT_BASE,
		.pfn		= __phys_to_pfn(DOVE_SCRATCHPAD_PHYS_BASE),
		.length		= DOVE_SCRATCHPAD_SIZE,
		.type		= MT_EXEC_REGS,
	}, 	{
		.virtual	= DOVE_SB_REGS_VIRT_BASE,
		.pfn		= __phys_to_pfn(DOVE_SB_REGS_PHYS_BASE),
		.length		= DOVE_SB_REGS_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= DOVE_NB_REGS_VIRT_BASE,
		.pfn		= __phys_to_pfn(DOVE_NB_REGS_PHYS_BASE),
		.length		= DOVE_NB_REGS_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= DOVE_PCIE0_IO_VIRT_BASE,
		.pfn		= __phys_to_pfn(DOVE_PCIE0_IO_PHYS_BASE),
		.length		= DOVE_PCIE0_IO_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= DOVE_PCIE1_IO_VIRT_BASE,
		.pfn		= __phys_to_pfn(DOVE_PCIE1_IO_PHYS_BASE),
		.length		= DOVE_PCIE1_IO_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= DOVE_CESA_VIRT_BASE,
		.pfn		= __phys_to_pfn(DOVE_CESA_PHYS_BASE),
		.length		= DOVE_CESA_SIZE,
		.type		= MT_DEVICE,
	},
};

void __init dove_map_io(void)
{
	iotable_init(dove_io_desc, ARRAY_SIZE(dove_io_desc));
}

/*****************************************************************************
 * EHCI0
 ****************************************************************************/
char * usb0_clkname = "usb0";
char * usb1_clkname = "usb1";
struct mv_usb_platform_data mv_udc0_data = {
 	.clknum = 1,
	.clkname = &usb0_clkname,
  //leave it emtpy
};
struct mv_usb_platform_data mv_udc1_data = {
 	.clknum = 1,
	.clkname = &usb1_clkname,
  //leave it emtpy
};

void __init dove_ehci0_init(void)
{
		char *name;   name = NULL;
		void *pdata;  pdata = NULL;
#ifdef CONFIG_MV_INCLUDE_USB
		if (	(strcmp(usb0Mode, "device") == 0) && 
				(strcmp(usb1Mode, "device") == 0)) {
				printk("Warning: trying to set both USB0 and USB1 to device mode!\n");
			}
		
		if (strcmp(usb0Mode, "host") == 0) {
				printk("Initializing USB0 Host\n");
				if (useHalDrivers) {
					//dove_ehci_data.dram = NULL;
					//dove_ehci_data.phy_version = EHCI_PHY_NA;
					mvSysUsbInit(0, 1);
				}
			}
		else {
				printk("Initializing USB0 Device\n");
				name = usb_dev_name;
				pdata = &mv_udc0_data;
				mvSysUsbInit(0, 0);
			}
#endif

	orion_ehci_init(DOVE_USB0_PHYS_BASE, IRQ_DOVE_USB0, EHCI_PHY_NA, pdata, name );
}

/*****************************************************************************
 * EHCI1
 ****************************************************************************/
void __init dove_ehci1_init(void)
{
		char *name;   name = NULL;
		void *pdata;  pdata = NULL;

#ifdef CONFIG_MV_INCLUDE_USB
		if (strcmp(usb1Mode, "host") == 0) {
			printk("Initializing USB1 Host\n");
			if (useHalDrivers) {
				//dove_ehci_data.dram = NULL;
				//dove_ehci_data.phy_version = EHCI_PHY_NA;
				mvSysUsbInit(1, 1);
			}
		}
		else {
			printk("Initializing USB1 Device\n");
			name = usb_dev_name;
			pdata = &mv_udc1_data;
			mvSysUsbInit(1, 0);
		}
#endif
	orion_ehci_1_init(DOVE_USB1_PHYS_BASE, IRQ_DOVE_USB1, pdata, name );
}

#ifdef CONFIG_MV_ETHERNET
/*****************************************************************************
 * Ethernet
 ****************************************************************************/
struct mv_eth_addr_dec_platform_data dove_eth_addr_dec_data = {
	.dram		= &orion_mbus_dram_info,
};

static struct platform_device dove_eth_addr_dec = {
	.name		= MV_ETH_ADDR_DEC_NAME,
	.id		= 0,
	.dev		= {
		.platform_data	= &dove_eth_addr_dec_data,
	},
	.num_resources	= 0,
};

static struct mv_netdev_platform_data dove_eth_data = {
	.port_number = 0
};

static struct resource dove_eth_resources[] = {
	{
		.name	= "eth irq",
		.start	= IRQ_DOVE_GE00_SUM,
		.end	= IRQ_DOVE_GE00_SUM,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device dove_eth = {
	.name		= MV_NETDEV_ETH_NAME,
	.id		= 0,
	.num_resources	= 1,
	.resource	= dove_eth_resources,
};

void __init dove_mv_eth_init(void)
{
	dove_eth.dev.platform_data = &dove_eth_data;
	platform_device_register(&dove_eth);
	if(!useHalDrivers)
		platform_device_register(&dove_eth_addr_dec);
}
#endif

#if 1
/*****************************************************************************
 * GE00
 ****************************************************************************/
void __init dove_ge00_init(struct mv643xx_eth_platform_data *eth_data)
{
	orion_ge00_init(eth_data,
			DOVE_GE00_PHYS_BASE, IRQ_DOVE_GE00_SUM,
			0, get_tclk());
}
#endif

/*****************************************************************************
 * SoC RTC
 ****************************************************************************/
void __init dove_rtc_init(void)
{
	orion_rtc_init(DOVE_RTC_PHYS_BASE, IRQ_DOVE_RTC);
}

/*****************************************************************************
 * SATA
 ****************************************************************************/
void __init dove_sata_init(struct mv_sata_platform_data *sata_data)
{
	orion_sata_init(sata_data, DOVE_SATA_PHYS_BASE, IRQ_DOVE_SATA);

}

/*****************************************************************************
 * UART0
 ****************************************************************************/
void __init dove_uart0_init(void)
{
	orion_uart0_init(DOVE_UART0_VIRT_BASE, DOVE_UART0_PHYS_BASE,
			 IRQ_DOVE_UART_0, get_tclk());
}

/*****************************************************************************
 * UART1
 ****************************************************************************/
void __init dove_uart1_init(void)
{
	orion_uart1_init(DOVE_UART1_VIRT_BASE, DOVE_UART1_PHYS_BASE,
			 IRQ_DOVE_UART_1, get_tclk());
}

/*****************************************************************************
 * UART2
 ****************************************************************************/
void __init dove_uart2_init(void)
{
	orion_uart2_init(DOVE_UART2_VIRT_BASE, DOVE_UART2_PHYS_BASE,
			 IRQ_DOVE_UART_2, get_tclk());
}

/*****************************************************************************
 * UART3
 ****************************************************************************/
void __init dove_uart3_init(void)
{
	orion_uart3_init(DOVE_UART3_VIRT_BASE, DOVE_UART3_PHYS_BASE,
			 IRQ_DOVE_UART_3, get_tclk());
}

/*****************************************************************************
 * SPI
 ****************************************************************************/
void __init dove_spi0_init(void)
{
	orion_spi_init(DOVE_SPI0_PHYS_BASE, get_tclk());
}

void __init dove_spi1_init(void)
{
	orion_spi_1_init(DOVE_SPI1_PHYS_BASE, get_tclk());
}

/*****************************************************************************
 * I2C
 ****************************************************************************/
static struct mv64xxx_i2c_pdata dove_i2c_data = {
	.freq_m 	= 10, /* assumes 166 MHz TCLK gets 94.3kHz */
	.freq_n 	= 3,
	.delay_after_stop = 3, /* 3 ms delay needed when freq is 94.3kHz */
	.timeout	= 1000, /* Default timeout of 1 second */
#ifdef CONFIG_I2C_MV64XXX_PORT_EXPANDER
	.select_exp_port = dove_select_exp_port,
#endif
};
	
static struct resource dove_i2c_resources[] = {
	{
		.name	= "i2c base",
		.start	= DOVE_I2C_PHYS_BASE,
		.end	= DOVE_I2C_PHYS_BASE + 0x20 -1,
		.flags	= IORESOURCE_MEM,
	}, {
		.name	= "i2c irq",
		.start	= IRQ_DOVE_I2C,
		.end	= IRQ_DOVE_I2C,
		.flags	= IORESOURCE_IRQ,
	},
};
	
static struct platform_device dove_i2c = {
	.name		= MV64XXX_I2C_CTLR_NAME,
	.id 	= 0,
	.num_resources	= ARRAY_SIZE(dove_i2c_resources),
	.resource	= dove_i2c_resources,
	.dev		= {
		.platform_data = &dove_i2c_data,
	},
};
#ifdef CONFIG_I2C_MV64XXX_PORT_EXPANDER
static struct mv64xxx_i2c_exp_pdata dove_i2c_exp_port0_data = {
	.hw_adapter = &dove_i2c,
	.timeout	= 1000, /* Default timeout of 1 second */
};
	
static struct platform_device dove_i2c_exp_port0 = {
	.name		= MV64XXX_I2C_EXPANDER_NAME,
	.id 	= 0,
	.dev		= {
		.platform_data = &dove_i2c_exp_port0_data,
	},
};
	
static struct mv64xxx_i2c_exp_pdata dove_i2c_exp_port1_data = {
	.hw_adapter = &dove_i2c,
	.timeout	= 1000, /* Default timeout of 1 second */
};
	
static struct platform_device dove_i2c_exp_port1 = {
	.name		= MV64XXX_I2C_EXPANDER_NAME,
	.id 	= 1,
	.dev		= {
		.platform_data = &dove_i2c_exp_port1_data,
	},
};
	
static struct mv64xxx_i2c_exp_pdata dove_i2c_exp_port2_data = {
	.hw_adapter = &dove_i2c,
	.timeout	= 1000, /* Default timeout of 1 second */
};
	
static struct platform_device dove_i2c_exp_port2 = {
	.name		= MV64XXX_I2C_EXPANDER_NAME,
	.id 	= 2,
	.dev		= {
		.platform_data = &dove_i2c_exp_port2_data,
	},
};
#endif
/*void __init dove_i2c_init(void)
{
	orion_i2c_init(DOVE_I2C_PHYS_BASE, IRQ_DOVE_I2C, 10);
} */

void __init dove_i2c_init(void)
{
	platform_device_register(&dove_i2c);
}
	
void __init dove_i2c_exp_init(int nr)
{
#ifdef CONFIG_I2C_MV64XXX_PORT_EXPANDER
	if (nr == 0) {
		dove_i2c_exp_port0_data.hw_adapter = &dove_i2c;
		platform_device_register(&dove_i2c_exp_port0);
	}
	
	if (nr == 1) {
		dove_i2c_exp_port1_data.hw_adapter = &dove_i2c;
		platform_device_register(&dove_i2c_exp_port1);
	}
	if (nr == 2) {
		dove_i2c_exp_port2_data.hw_adapter = &dove_i2c;
		platform_device_register(&dove_i2c_exp_port2);
	}
#endif
}
	
#ifdef CONFIG_DOVE_DB_USE_GPIO_I2C
static struct i2c_gpio_platform_data dove_gpio_i2c_pdata = {
	.sda_pin	= 17,
	.scl_pin	= 19,
	.udelay 	= 2, /* ~100 kHz */
};
	
static struct platform_device dove_gpio_i2c_device = {
	.name					= "i2c-gpio",
	.id 					= 1,
	.dev.platform_data		= &dove_gpio_i2c_pdata,
};
	
void __init dove_add_gpio_i2c(void)
{
	platform_device_register(&dove_gpio_i2c_device);
}
#endif


/*****************************************************************************
 * Time handling
 ****************************************************************************/
void __init dove_init_early(void)
{
	orion_time_set_base(TIMER_VIRT_BASE);
}

static int get_tclk(void)
{
	/* use DOVE_RESET_SAMPLE_HI/LO to detect tclk */
	return 166666667;
}

static void dove_timer_init(void)
{
	orion_time_init(BRIDGE_VIRT_BASE, BRIDGE_INT_TIMER1_CLR,
			IRQ_DOVE_BRIDGE, get_tclk());
}

struct sys_timer dove_timer = {
	.init = dove_timer_init,
};

/*****************************************************************************
 * XOR 0
 ****************************************************************************/
void __init dove_xor0_init(void)
{
	orion_xor0_init(DOVE_XOR0_PHYS_BASE, DOVE_XOR0_HIGH_PHYS_BASE,
			IRQ_DOVE_XOR_00, IRQ_DOVE_XOR_01);
}

/*****************************************************************************
 * XOR 1
 ****************************************************************************/
void __init dove_xor1_init(void)
{
	orion_xor1_init(DOVE_XOR1_PHYS_BASE, DOVE_XOR1_HIGH_PHYS_BASE,
			IRQ_DOVE_XOR_10, IRQ_DOVE_XOR_11);
}

/*****************************************************************************
 * SDIO
 ****************************************************************************/
#define DOVE_SD0_START_GPIO	40
#define DOVE_SD1_START_GPIO	46
struct sdhci_dove_int_wa sdio0_data = {
		.gpio = DOVE_SD0_START_GPIO + 3,
		.func_select_bit = 0
	};
	
static u64 sdio_dmamask = DMA_BIT_MASK(32);

/*****************************************************************************
 * SDIO0
 ****************************************************************************/
 
static struct resource dove_sdio0_resources[] = {
	{
		.start	= DOVE_SDIO0_PHYS_BASE,
		.end	= DOVE_SDIO0_PHYS_BASE + 0xff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_DOVE_SDIO0,
		.end	= IRQ_DOVE_SDIO0,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device dove_sdio0 = {
	.name		= "sdhci-dove",
	.id		= 0,
	.dev		= {
		.dma_mask		= &sdio_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.resource	= dove_sdio0_resources,
	.num_resources	= ARRAY_SIZE(dove_sdio0_resources),
};

void __init dove_sdio0_init(void)
{
	platform_device_register(&dove_sdio0);
}

/*****************************************************************************
 * SDIO1
 ****************************************************************************/
struct sdhci_dove_int_wa sdio1_data = {
	.gpio = DOVE_SD1_START_GPIO + 3,
	.func_select_bit = 1
};

static struct resource dove_sdio1_resources[] = {
	{
		.start	= DOVE_SDIO1_PHYS_BASE,
		.end	= DOVE_SDIO1_PHYS_BASE + 0xff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_DOVE_SDIO1,
		.end	= IRQ_DOVE_SDIO1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device dove_sdio1 = {
	.name		= "sdhci-dove",
	.id		= 1,
	.dev		= {
		.dma_mask		= &sdio_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.resource	= dove_sdio1_resources,
	.num_resources	= ARRAY_SIZE(dove_sdio1_resources),
};

void __init dove_sdio1_init(void)
{
	platform_device_register(&dove_sdio1);
}
#define DOVE_SD0_START_GPIO	40
#define DOVE_SD1_START_GPIO	46

void __init dove_sd_card_int_wa_setup(int port)
{
	int	gpio = 0, i, irq = 0;
	char	*name;

	switch(port) {
	case 0:
		gpio = DOVE_SD0_START_GPIO;
		name = "sd0";
		dove_sdio0.dev.platform_data = &sdio0_data;
		irq = sdio0_data.irq = gpio_to_irq(sdio0_data.gpio);
		break;
	case 1:
		gpio = DOVE_SD1_START_GPIO;
		name = "sd1";
		dove_sdio1.dev.platform_data = &sdio1_data;
		irq = sdio1_data.irq = gpio_to_irq(sdio1_data.gpio);
		break;
	default:
		printk(KERN_ERR "dove_sd_card_int_wa_setup: bad port (%d)\n", 
		       port);
		return;
	}
	
	for (i = gpio; i < (gpio + 6); i++) {
		orion_gpio_set_valid(i, 1);
		
		if (gpio_request(i, name) != 0)
			printk(KERN_ERR "dove: failed to config gpio (%d) for %s\n",
			       i, name);

		gpio_direction_input(i);
	}
	irq_set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);
}

#ifdef CONFIG_PM
	static u32 dove_arbitration_regs[] = {
		DOVE_MC_VIRT_BASE + 0x280,
		DOVE_MC_VIRT_BASE + 0x510,
	};
#endif
void __init dove_config_arbitration(void)
{
	u32 sc_dec;

	sc_dec = readl(DOVE_MC_VIRT_BASE + 0x280);
	printk("DOVE_MC @ 0x280 is %08X\n", sc_dec);
	#ifdef CONFIG_DOVE_REV_Y0
	sc_dec &= 0xfff0ffff;
	sc_dec |= 0x000e0000;
	#endif
	writel(sc_dec, DOVE_MC_VIRT_BASE + 0x280);
	
        /* Dove Z0 and Z1
        * Master 0 - Vmeta
        * Master 1 - GC500
        * Master 2 - LCD
        * Master 3 - Upstream (SB)
        */

	/* Dove Y0
 	 * MC Master 0 - CPU
 	 * MC Master 1 - vmeta-GC-UP
 	 * MC Master 2 - LCD
 	 */
        /*
  	 * MC Master 1
         * Master 0 - Vmeta
	 * Master 1 - GC500
	 * Master 2 - LCD
	 * Master 3 - Upstream (SB)
	 */

        sc_dec = readl(DOVE_MC_VIRT_BASE + 0x510);
        printk("PLiao: DOVE_MC @ 0x510 is %08X\n", sc_dec);
	
	sc_dec &= 0xf0f0f0f0;
#ifdef CONFIG_DOVE_REV_Y0
	sc_dec |= 0x01010101;
#endif
        writel(sc_dec, DOVE_MC_VIRT_BASE + 0x510);
        /* End of supersection testing */
#ifdef CONFIG_PM
	pm_registers_add(dove_arbitration_regs,
			 ARRAY_SIZE(dove_arbitration_regs));
#endif
}


int __init dove_devclks_init(void);
int __init dove_clk_config(struct device *dev, const char *id, unsigned long rate);
void __init dove_init(void)
{
	int tclk;
	struct clk *clk;

	dove_devclks_init();
	clk = clk_get(NULL, "tclk");
	if (IS_ERR(clk)) {
	     printk(KERN_ERR "failed to get tclk \n");
	     return;
	}

	tclk = get_tclk();

	printk(KERN_INFO "Dove 88AP510 SoC, ");
	printk(KERN_INFO "TCLK = %dMHz\n", (tclk + 499999) / 1000000);

#ifdef CONFIG_CACHE_TAUROS2
	tauros2_init();
#endif
	dove_setup_cpu_mbus();
	dove_config_arbitration();

	dove_clk_config(NULL, "GCCLK", 500*1000000);
	dove_clk_config(NULL, "AXICLK", 333*1000000);

	/* internal devices that every board has */
	dove_rtc_init();
	dove_xor0_init();
	dove_xor1_init();
}

#ifdef CONFIG_PM
void dove_save_cpu_conf_regs(void)
{
	DOVE_DWNSTRM_BRDG_SAVE(CPU_CONFIG);
	DOVE_DWNSTRM_BRDG_SAVE(CPU_CONTROL);
	DOVE_DWNSTRM_BRDG_SAVE(RSTOUTn_MASK);
	DOVE_DWNSTRM_BRDG_SAVE(BRIDGE_MASK);
	DOVE_DWNSTRM_BRDG_SAVE(POWER_MANAGEMENT);
}

void dove_restore_cpu_conf_regs(void)
{
	DOVE_DWNSTRM_BRDG_RESTORE(CPU_CONFIG);
	DOVE_DWNSTRM_BRDG_RESTORE(CPU_CONTROL);
	DOVE_DWNSTRM_BRDG_RESTORE(RSTOUTn_MASK);
	DOVE_DWNSTRM_BRDG_RESTORE(BRIDGE_MASK);
	DOVE_DWNSTRM_BRDG_RESTORE(POWER_MANAGEMENT);
}

void dove_save_upstream_regs(void)
{
	DOVE_UPSTRM_BRDG_SAVE(UPSTRM_AXI_P_D_CTRL_REG);
	DOVE_UPSTRM_BRDG_SAVE(UPSTRM_D2X_ARB_LO_REG);
	DOVE_UPSTRM_BRDG_SAVE(UPSTRM_D2X_ARB_HI_REG);
}

void dove_restore_upstream_regs(void)
{
	DOVE_UPSTRM_BRDG_RESTORE(UPSTRM_AXI_P_D_CTRL_REG);
	DOVE_UPSTRM_BRDG_RESTORE(UPSTRM_D2X_ARB_LO_REG);
	DOVE_UPSTRM_BRDG_RESTORE(UPSTRM_D2X_ARB_HI_REG);
}

void dove_save_timer_regs(void)
{
	DOVE_DWNSTRM_BRDG_SAVE(TIMER_CTRL);
	DOVE_DWNSTRM_BRDG_SAVE(TIMER0_RELOAD);
	DOVE_DWNSTRM_BRDG_SAVE(TIMER1_RELOAD);
	DOVE_DWNSTRM_BRDG_SAVE(TIMER_WD_RELOAD);
}

void dove_restore_timer_regs(void)
{
	DOVE_DWNSTRM_BRDG_RESTORE(TIMER_CTRL);
	DOVE_DWNSTRM_BRDG_RESTORE(TIMER0_RELOAD);
	DOVE_DWNSTRM_BRDG_RESTORE(TIMER1_RELOAD);
	DOVE_DWNSTRM_BRDG_RESTORE(TIMER_WD_RELOAD);
}

void dove_save_int_regs(void)
{
	DOVE_DWNSTRM_BRDG_SAVE(IRQ_MASK_LOW);
	DOVE_DWNSTRM_BRDG_SAVE(FIQ_MASK_LOW);
        DOVE_DWNSTRM_BRDG_SAVE(ENDPOINT_MASK_LOW);
	DOVE_DWNSTRM_BRDG_SAVE(IRQ_MASK_HIGH);
	DOVE_DWNSTRM_BRDG_SAVE(FIQ_MASK_HIGH);
	DOVE_DWNSTRM_BRDG_SAVE(ENDPOINT_MASK_HIGH);
	DOVE_DWNSTRM_BRDG_SAVE(PCIE_INTERRUPT_MASK);
}

void dove_restore_int_regs(void)
{
	DOVE_DWNSTRM_BRDG_RESTORE(IRQ_MASK_LOW);
	DOVE_DWNSTRM_BRDG_RESTORE(FIQ_MASK_LOW);
	DOVE_DWNSTRM_BRDG_RESTORE(ENDPOINT_MASK_LOW);
	DOVE_DWNSTRM_BRDG_RESTORE(IRQ_MASK_HIGH);
	DOVE_DWNSTRM_BRDG_RESTORE(FIQ_MASK_HIGH);
	DOVE_DWNSTRM_BRDG_RESTORE(ENDPOINT_MASK_HIGH);
	DOVE_DWNSTRM_BRDG_RESTORE(PCIE_INTERRUPT_MASK);
}
#endif


void dove_restart(char mode, const char *cmd)
{
	/*
	 * Enable soft reset to assert RSTOUTn.
	 */
	writel(SOFT_RESET_OUT_EN, RSTOUTn_MASK);

	/*
	 * Assert soft reset.
	 */
	writel(SOFT_RESET, SYSTEM_SOFT_RESET);

	while (1)
		;
}


/*****************************************************************************
 * I2S/SPDIF
 ****************************************************************************/
static struct resource dove_i2s0_resources[] = {
	[0] = {
		.start  = DOVE_AUD0_PHYS_BASE,
		.end    = DOVE_AUD0_PHYS_BASE + SZ_16K -1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_DOVE_I2S0,
		.end    = IRQ_DOVE_I2S0,
		.flags  = IORESOURCE_IRQ,
	}
};

static u64 dove_i2s0_dmamask = 0xFFFFFFFFUL;

 
static struct platform_device dove_i2s0 = {
	.name           = "mv88fx_snd",
	.id             = 0,
	.dev            = {
		.dma_mask = &dove_i2s0_dmamask,
		.coherent_dma_mask = 0xFFFFFFFF,
	},
	.num_resources  = ARRAY_SIZE(dove_i2s0_resources),
	.resource       = dove_i2s0_resources,
};

static struct resource dove_i2s1_resources[] = {
	[0] = {
		.start  = DOVE_AUD1_PHYS_BASE,
		.end    = DOVE_AUD1_PHYS_BASE + SZ_16K -1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_DOVE_I2S1,
		.end    = IRQ_DOVE_I2S1,
		.flags  = IORESOURCE_IRQ,
	}
};

static u64 dove_i2s1_dmamask = 0xFFFFFFFFUL;

static struct platform_device dove_i2s1 = {
	.name           = "mv88fx_snd",
	.id             = 1,
	.dev            = {
		.dma_mask = &dove_i2s1_dmamask,
		.coherent_dma_mask = 0xFFFFFFFF,
	},
	.num_resources  = ARRAY_SIZE(dove_i2s1_resources),
	.resource       = dove_i2s1_resources,
};

static struct platform_device dove_mv88fx_i2s0 = {
	.name           = "mv88fx-i2s",
	.id             = 0,
};

static struct platform_device dove_mv88fx_i2s1 = {
	.name           = "mv88fx-i2s",
	.id             = 1,
};

static struct platform_device dove_pcm_device0 = {
	.name		= "mv88fx-pcm-audio",
	.id		= 0,
};
static struct platform_device dove_pcm_device1 = {
	.name		= "mv88fx-pcm-audio",
	.id		= 1,
};


void __init dove_i2s_init(int port, struct orion_i2s_platform_data *i2s_data)
{
	switch(port){
	case 0:
		platform_device_register(&dove_mv88fx_i2s0);
		i2s_data->dram = &orion_mbus_dram_info;
		dove_i2s0.dev.platform_data = i2s_data;
		platform_device_register(&dove_i2s0);
		platform_device_register(&dove_pcm_device0);
		return;
	case 1:
		platform_device_register(&dove_mv88fx_i2s1);
		i2s_data->dram = &orion_mbus_dram_info;
		dove_i2s1.dev.platform_data = i2s_data;
		platform_device_register(&dove_i2s1);
		platform_device_register(&dove_pcm_device1);
		return;
	default:
		BUG();
	}

		
}


/*****************************************************************************
 * VPU
 ****************************************************************************/
static struct resource dove_vmeta_resources[] = {
	[0] = {
		.start	= DOVE_VPU_PHYS_BASE,
		.end	= DOVE_VPU_PHYS_BASE + 0x280000 - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {		/* Place holder for reserved memory */
		.start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start  = IRQ_DOVE_VPRO_DMA1,
		.end    = IRQ_DOVE_VPRO_DMA1,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device dove_vmeta = {
	.name		= "ap510-vmeta",
	.id		= 0,
	.dev		= {
		.dma_mask		= DMA_BIT_MASK(32),
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.resource	= dove_vmeta_resources,
	.num_resources	= ARRAY_SIZE(dove_vmeta_resources),
};

void __init dove_vmeta_init(void)
{
#ifdef CONFIG_UIO_VMETA
	if (vmeta_size == 0) {
		printk("memory allocation for VMETA failed\n");
		return;
	}

	dove_vmeta_resources[1].start = dove_vmeta_memory_start;
	dove_vmeta_resources[1].end = dove_vmeta_memory_start + vmeta_size - 1;
	printk(" dove_vmeta_resources.start is 0x%x, size is 0x%x, end is 0x%x \n",
			 dove_vmeta_memory_start, vmeta_size, dove_vmeta_memory_start + vmeta_size - 1 );
	platform_device_register(&dove_vmeta);
#endif
}

#ifdef CONFIG_DOVE_VPU_USE_BMM
unsigned int dove_vmeta_get_memory_start(void)
{
	return dove_vmeta_memory_start;
}
EXPORT_SYMBOL(dove_vmeta_get_memory_start);

int dove_vmeta_get_memory_size(void)
{
	return vmeta_size;
}
EXPORT_SYMBOL(dove_vmeta_get_memory_size);
#endif

/*****************************************************************************
 * GPU
 ****************************************************************************/
static struct resource dove_gpu_resources[] = {
	{
		.name   = "gpu_irq",
		.start	= IRQ_DOVE_GPU,
		.end	= IRQ_DOVE_GPU,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "gpu_base",
		.start	= DOVE_GPU_PHYS_BASE,
		.end	= DOVE_GPU_PHYS_BASE + 0x40000 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "gpu_mem",
		.start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device dove_gpu = {
	.name		= "galcore",
	.id		= 0,
	.num_resources  = ARRAY_SIZE(dove_gpu_resources),
	.resource       = dove_gpu_resources,
};

void __init dove_gpu_init(void)
{
#ifdef CONFIG_DOVE_GPU
	if (gpu_size == 0) {
		printk("memory allocation for GPU failed\n");
		return;
	}

	dove_gpu_resources[2].start = dove_gpu_memory_start;
	dove_gpu_resources[2].end = dove_gpu_memory_start + gpu_size - 1;
	printk(" dove_gpu_resources.start is 0x%x, size is 0x%x, end is 0x%x \n",
		 dove_gpu_memory_start, gpu_size, dove_gpu_memory_start + gpu_size - 1 );
	platform_device_register(&dove_gpu);
#endif
}


/*****************************************************************************
 * CESA
 ****************************************************************************/
struct mv_cesa_addr_dec_platform_data dove_cesa_addr_dec_data = {
	.dram		= &orion_mbus_dram_info,
};

static struct platform_device dove_cesa_ocf = {
	.name           = "dove_cesa_ocf",
	.id             = -1,
	.dev		= {
		.platform_data	= &dove_cesa_addr_dec_data,
	},
};

static struct platform_device dove_cesadev = {
	.name           = "dove_cesadev",
	.id             = -1,
};


void __init dove_cesa_init(void)
{
	platform_device_register(&dove_cesa_ocf);
	platform_device_register(&dove_cesadev);
}

/*****************************************************************************
 * SoC hwmon Thermal Sensor
 ****************************************************************************/
void __init dove_hwmon_init(void)
{
	platform_device_register_simple("dove-temp", 0, NULL, 0);
}

// This fixup function is used to reserve memory for the GPU and VPU engines
    //as these drivers require large chunks of consecutive memory.
void __init dove_tag_fixup_mem32( struct tag *t,
		char **from, struct meminfo *meminfo)
{
	//we leave it empty, now. it's meaning in 2.6.32
#if 0
	//	struct tag *last_tag = NULL;
	int total_size = pvt_size + vmeta_size + gpu_size;
	struct membank *bank = &meminfo->bank[meminfo->nr_banks];
	int i;

	printk("pvt_size is 0x%x, vmet_size is 0x%x, gpu_size is 0x%x \n",
		pvt_size, vmeta_size, gpu_size);
	for (i = 0; i < meminfo->nr_banks; i++) {
		bank--;
		if (bank->size >= total_size)
			break;
	}
	if (i >= meminfo->nr_banks) {
		printk(KERN_WARNING "No suitable memory bank was found, "
				"required memory %d MB.\n", total_size);
		vmeta_size = 0;
		gpu_size = 0;
		return;
	}

	/* Resereve memory from last bank for PVT tests	*/
	bank->size -= pvt_size;

	/* Resereve memory from last bank for VPU usage.	*/
	bank->size -= vmeta_size;
	dove_vmeta_memory_start = bank->start + bank->size;

	/* Reserve memory for gpu usage */
	bank->size -= gpu_size;
	dove_gpu_memory_start = bank->start + bank->size;
#endif
}
void dove_reserve_mem(void)
{
#if 1
	phys_addr_t paddr;
	printk("pvt_size is 0x%x, vmet_size is 0x%x, gpu_size is 0x%x \n",
		pvt_size, vmeta_size, gpu_size);
		
	if (pvt_size)
	{	
		paddr = arm_memblock_steal(vmeta_size, SZ_1K);
		if (!paddr) { pvt_size =0; printk(" Allocate for pvt is failed!\n"); }
	}
	if (vmeta_size)
	{	
		paddr = arm_memblock_steal(vmeta_size, SZ_1M);
		if (!paddr) { vmeta_size = 0; printk(" Allocate for vmeta is failed!\n"); }
		else  dove_vmeta_memory_start = paddr;
	}

	if (gpu_size)
	{	
		paddr = arm_memblock_steal(gpu_size, SZ_1M);
		if (!paddr) { gpu_size = 0; printk(" Allocate for gpu is failed!\n"); }
		else dove_gpu_memory_start = paddr;
	}
#endif
}

