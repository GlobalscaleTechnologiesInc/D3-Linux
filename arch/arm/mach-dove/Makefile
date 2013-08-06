obj-y				+= clock.o common.o addr-map.o irq.o pcie.o mpp.o

#obj-$(CONFIG_MACH_DOVE_D2PLUG)  += dove-db-setup.o
obj-$(CONFIG_MACH_D2PLUG)  += d2plug.o
obj-$(CONFIG_MACH_DOVE_VIDEOPLUG)	+= dove-videoplug-setup.o
obj-$(CONFIG_I2C_MV64XXX_PORT_EXPANDER)	+= twsi.o
obj-$(CONFIG_DOVE_PM_SUPPORT)	+= pm.o
obj-$(CONFIG_CPU_FREQ_DOVE)	+= cpufreq.o
obj-$(CONFIG_HIBERNATION)	+= cpu.o swsusp.o
obj-$(CONFIG_SENSORS_DOVE)	+= hwmon.o
obj-$(CONFIG_FB_DOVE_CLCD)	+= clcd.o
obj-$(CONFIG_FB_DOVE_CLCD1)	+= clcd.o
obj-$(CONFIG_MV_INCLUDE_MPP_SYSFS)	+= avng_mpp.o
obj-$(CONFIG_CPU_IDLE)		+= cpuidle.o
obj-$(CONFIG_PROC_FS)		+= dump_cp15_regs.o
obj-$(CONFIG_MV_EXT_VGA_CLOCK)	+= idt5v49ee503.o

include $(srctree)/$(subst ",,$(CONFIG_MV_HAL_RULES_PATH))
#MV_DEFINE = -DMV_LINUX -DMV_CPU_LE -DMV_ARM -DLINUX
#LSP_PATH_I      = $(srctree)/arch/arm/mach-dove/tim
#EXTRA_CFLAGS 	+= -I$(LSP_PATH_I) -I$(LSP_PATH_I)/linux_oss -I$(LSP_PATH_I)/config -I$(LSP_PATH_I)/common $(MV_DEFINE)

obj-y           += mv_hal_support/
#obj-m		+= gc600_driver_dove/
obj-y		+= gc600_driver_dove/
