#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <linux/mtd/mtd.h>
#include <linux/slab.h>
#include <linux/platform_device.h>


#include <linux/workqueue.h>



//for dreamplug
#define GPIO_BTN 45
#define GPIO_BTN2 46  //dreamplug


/*
//for guruplug
#define GPIO_BTN 45
#define GPIO_BTN2 44
*/


static struct timer_list gpio_btn_timer; 
static int btn_time;
static irqreturn_t gpio_btn_handler(int irq, void *dev_id);
static void gpio_btn_timer_handler(unsigned long);


static struct workqueue_struct *gpio_btn_wq;
static struct work_struct *gpio_btn_wk;

struct platform_device gpio_dev_btn = {
 	.name="gpio", .id=GPIO_BTN, .num_resources=0, .resource=0, };


static void __init gpio_btn(void)
{
	int error,error2;  int irq; int ret;
	orion_gpio_set_valid(GPIO_BTN, GPIO_INPUT_OK);
	orion_gpio_set_valid(GPIO_BTN2, GPIO_OUTPUT_OK);
	
	error = gpio_request(GPIO_BTN, "gpio_btn");
	error2 = gpio_request(GPIO_BTN2, "gpio_btn2");
	if ( error<0 || error2<0 ) { pr_err("gpio_btn: failed to request GPIO %d, error %d; error2 %d \n",GPIO_BTN, error,error2); }

	error = gpio_direction_input(GPIO_BTN);
	if ( error<0 ) 
	{ pr_err("gpio_btn: failed to configure input for GPIO %d, error %d; error2 %d \n",GPIO_BTN, error,error2); }

	irq = gpio_to_irq(GPIO_BTN);
	if ( irq < 0) 
	{ pr_err("gpio_btn: Unable to get irq number for GPIO %d, error %d \n",GPIO_BTN, irq); }

    error = request_irq(irq, gpio_btn_handler, IRQF_TRIGGER_FALLING, "gpio_btn", NULL);
	if ( error ) 
	{ pr_err("gpio_btn: failed to request_irq for GPIO/irq %d %d \n",GPIO_BTN,irq); }

	//platform_device_register(&gpio_dev_btn);
	
	printk("gpio_btn is called sucessfully! the initial value is %d, return value is %d \n",__gpio_get_value(GPIO_BTN),ret);
}
static irqreturn_t gpio_btn_handler(int irq, void *dev_id)
{
	printk("  the gpio is triggered, the irq is %d ! \n",irq);
	btn_time=0;
	init_timer(&gpio_btn_timer);
    gpio_btn_timer.data = (unsigned long)NULL;
    gpio_btn_timer.function = gpio_btn_timer_handler;
    gpio_btn_timer.expires = jiffies + 1 * HZ;
	add_timer(&gpio_btn_timer);
	return IRQ_HANDLED;
}

static void gpio_btn_timer_handler(unsigned long data)
{
	
	gpio_btn_timer.expires = jiffies + 1 * HZ;
	printk("  the gpio timer is called ! time is %d \n", btn_time);
	if ( btn_time<5 )
	{  if (__gpio_get_value(GPIO_BTN)==1)
		{
			printk("***  Begin rebooting system!! \n"); 
			gpio_direction_output(GPIO_BTN2,0);
		}
		else 
		{
			btn_time++;
			mod_timer(&gpio_btn_timer,jiffies + 1 * HZ);
		}
	}
	else 
	{  if (__gpio_get_value(GPIO_BTN)==1)
		{	
			printk("*** You can realize restore functionality yourself! \n"); 
			//queue_work(gpio_btn_wq, gpio_btn_wk);
			del_timer(&gpio_btn_timer); 
		}
		else 
		{
			btn_time++;
			mod_timer(&gpio_btn_timer,jiffies + 1 * HZ); 
		}
	}
}


void __exit gpio_exit(void)
{
	//we can leave it empty; 
}
module_init(gpio_btn);
module_exit(gpio_exit);



