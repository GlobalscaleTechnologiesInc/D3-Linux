/*
 * arch/arm/mach-dove/common.h
 *
 * Core functions for Marvell Dove 88AP510 System On Chip
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __ARCH_DOVE_COMMON_H
#define __ARCH_DOVE_COMMON_H

/* The following is a list of Marvell status    */
#define MV_ERROR		    (-1)
#define MV_OK			    (0x00)  /* Operation succeeded                   */
#define MV_FAIL			    (0x01)	/* Operation failed                      */
#define MV_BAD_VALUE        (0x02)  /* Illegal value (general)               */
#define MV_OUT_OF_RANGE     (0x03)  /* The value is out of range             */
#define MV_BAD_PARAM        (0x04)  /* Illegal parameter in function called  */
#define MV_BAD_PTR          (0x05)  /* Illegal pointer value                 */
#define MV_BAD_SIZE         (0x06)  /* Illegal size                          */
#define MV_BAD_STATE        (0x07)  /* Illegal state of state machine        */
#define MV_SET_ERROR        (0x08)  /* Set operation failed                  */
#define MV_GET_ERROR        (0x09)  /* Get operation failed                  */
#define MV_CREATE_ERROR     (0x0A)  /* Fail while creating an item           */
#define MV_NOT_FOUND        (0x0B)  /* Item not found                        */
#define MV_NO_MORE          (0x0C)  /* No more items found                   */
#define MV_NO_SUCH          (0x0D)  /* No such item                          */
#define MV_TIMEOUT          (0x0E)  /* Time Out                              */
#define MV_NO_CHANGE        (0x0F)  /* Parameter(s) is already in this value */
#define MV_NOT_SUPPORTED    (0x10)  /* This request is not support           */
#define MV_NOT_IMPLEMENTED  (0x11)  /* Request supported but not implemented */
#define MV_NOT_INITIALIZED  (0x12)  /* The item is not initialized           */
#define MV_NO_RESOURCE      (0x13)  /* Resource not available (memory ...)   */
#define MV_FULL             (0x14)  /* Item is full (Queue or table etc...)  */
#define MV_EMPTY            (0x15)  /* Item is empty (Queue or table etc...) */
#define MV_INIT_ERROR       (0x16)  /* Error occured while INIT process      */
#define MV_HW_ERROR         (0x17)  /* Hardware error                        */
#define MV_TX_ERROR         (0x18)  /* Transmit operation not succeeded      */
#define MV_RX_ERROR         (0x19)  /* Recieve operation not succeeded       */
#define MV_NOT_READY	    (0x1A)	/* The other side is not ready yet       */
#define MV_ALREADY_EXIST    (0x1B)  /* Tried to create existing item         */
#define MV_OUT_OF_CPU_MEM   (0x1C)  /* Cpu memory allocation failed.         */
#define MV_NOT_STARTED      (0x1D)  /* Not started yet         */
#define MV_BUSY             (0x1E)  /* Item is busy.                         */
#define MV_TERMINATE        (0x1F)  /* Item terminates it's work.            */
#define MV_NOT_ALIGNED      (0x20)  /* Wrong alignment                       */
#define MV_NOT_ALLOWED      (0x21)  /* Operation NOT allowed                 */
#define MV_WRITE_PROTECT    (0x22)  /* Write protected                       */


#define MV_INVALID  (int)(-1)

#define MV_FALSE	0
#define MV_TRUE     (!(MV_FALSE))


#ifndef NULL
#define NULL ((void*)0)
#endif


struct mv643xx_eth_platform_data;
struct mv_sata_platform_data;

extern struct sys_timer dove_timer;
extern struct mbus_dram_target_info dove_mbus_dram_info;


void dove_pm_register (void);
void dove_mv_eth_init(void);

/*
 * Basic Dove init functions used early by machine-setup.
 */
void dove_pcie_id(u32 *dev, u32 *rev);
void dove_map_io(void);
void dove_init(void);
void dove_init_early(void);
void dove_init_irq(void);
void dove_setup_cpu_mbus(void);
void dove_ge00_init(struct mv643xx_eth_platform_data *eth_data);
void dove_sata_init(struct mv_sata_platform_data *sata_data);
void dove_pcie_init(int init_port0, int init_port1);
void dove_ehci0_init(void);
void dove_ehci1_init(void);
void dove_uart0_init(void);
void dove_uart1_init(void);
void dove_uart2_init(void);
void dove_uart3_init(void);
void dove_spi0_init(void);
void dove_spi1_init(void);
void dove_i2c_init(void);
void dove_sdio0_init(void);
void dove_sdio1_init(void);
void dove_sd_card_int_wa_setup(int port);
void dove_restart(char, const char *);
void dove_cesa_init(void);
void dove_hwmon_init(void);
void dove_vmeta_init(void);
void dove_gpu_init(void);

int dove_select_exp_port(unsigned int port_id);
void __init dove_i2s_init(int port, struct orion_i2s_platform_data *i2s_data);
void __init dove_i2c_exp_init(int nr);

void __init dove_tag_fixup_mem32( struct tag *t, char **from, struct meminfo *meminfo);
void __init dove_reserve_mem(void);
/*
 * Basic Dove PM functions
 */
#ifdef CONFIG_PM
void dove_save_cpu_wins(void);
void dove_restore_cpu_wins(void);
void dove_restore_pcie_regs(void);
void dove_save_cpu_conf_regs(void);
void dove_save_timer_regs(void);	
void dove_restore_timer_regs(void);	
void dove_restore_cpu_conf_regs(void);
void dove_save_int_regs(void);
void dove_restore_int_regs(void);
void dove_save_upstream_regs(void);
void dove_restore_upstream_regs(void);
#endif


#endif
