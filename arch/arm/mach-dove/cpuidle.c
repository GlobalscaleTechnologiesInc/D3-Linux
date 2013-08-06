/*
 * arch/arm/mach-dove/cpuidle.c
 *
 * CPU idle implementation for Marvell Dove SoCs
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/cpuidle.h>
#include <asm/io.h>
#include <asm/proc-fns.h>
#include <mach/pm.h>
#include <linux/module.h>

#define DOVE_IDLE_STATES	2

void dove_pm_cpuidle_deepidle (void);

/* Actual code that puts the SoC in different idle states */
static int dove_enter_idle(struct cpuidle_device *dev,
			       struct cpuidle_driver *drv, int index)
{
	struct timeval before, after;
	int idle_time;

	local_irq_disable();
	do_gettimeofday(&before);
	if (index == 0) {
		/* Wait for interrupt state */
		cpu_do_idle();
	}
	else if (index == 1) {
		/* Deep Idle or Ebook depending on LCD status */
		dove_pm_cpuidle_deepidle();
		//printk(" *** enter deepidle  *** \n");
		//cpu_do_idle();
	}
	do_gettimeofday(&after);
	local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
			(after.tv_usec - before.tv_usec);
	dev->last_residency = idle_time;
	return index;
}

extern int pm_disable;

static struct cpuidle_driver dove_idle_driver = {
	.name			= "dove_idle",
	.owner			= THIS_MODULE,
	//.en_core_tk_irqen	= 1,
	.states[0]		= {
		.enter			= dove_enter_idle,
		.exit_latency		= 1,
		.target_residency	= 1000,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "WFI",
		.desc			= "Wait for interrupt",
	},
	.states[1]		= {
		.enter			= dove_enter_idle,
		.exit_latency		= 300,
		.target_residency	= 10000,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "DEEP IDLE",
		.desc			= "Deep Idle (eBook if LCD is OFF)",
	},
	.state_count = DOVE_IDLE_STATES,
};

DEFINE_PER_CPU(struct cpuidle_device, dove_cpuidle_device);

/* 
 * Register Dove IDLE states
 */
int dove_init_cpuidle(void)
{
	struct cpuidle_device *device;

	cpuidle_register_driver(&dove_idle_driver);
	device = &per_cpu(dove_cpuidle_device, smp_processor_id());
	device->state_count = DOVE_IDLE_STATES;

	printk( "CPU Power Management is %s\n", pm_disable ? "disabled!" : "enabled!");
	if (pm_disable)
		return 0;

	if (cpuidle_register_device(device)) {
		printk(KERN_ERR "dove_init_cpuidle: Failed registering\n");
		return -EIO;
	}

	return 0;
}

device_initcall(dove_init_cpuidle);
