/**
 * This module can probe temperature for Loongson family CPU. Based timer
 * struct, if temperature over than POWEOFF_TEMP, the system will be halt.
 * If over than WARING_TEMP, the system will warn user.
 *
 * This module compatibled 3A and 3B.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <boot_param.h>
#include <asm/reboot.h>
#include "temperature.h"

#define CPU0_SENSOR_BASE		0x900000001fe00190
#define CPU1_SENSOR_BASE		0x900010001fe00190
#define CPU2_SENSOR_BASE		0x900020001fe00190
#define CPU3_SENSOR_BASE		0x900030001fe00190
#define CPU_TEMP_SENSOR0		0xc
#define CPU_TEMP_SENSOR1		0xd
#define DEVNAME				"temphandler"
#define WARNING_TEMP			700
#define POWEROFF_TEMP			850

static struct timer_list timer_temp;
static int probe_temp(struct platform_device *);
static int remove_temp(struct platform_device *);
extern enum loongson_cpu_type cputype;

#if PROC_PRINT
struct temperature cpu_temp;
EXPORT_SYMBOL(cpu_temp);
#endif

/**
 * temp_resource struct description cpu sensor's resource.
 * .start: sensor0
 * .end: sensor1
 *
 * 0: 3A desktop
 * 1: 3A server and 3B desktop
 * 2: 3B server and KD90
 */
struct resource temp_resource[] = {
	[0] = {
		.start	= CPU0_SENSOR_BASE + CPU_TEMP_SENSOR0,
		.end	= CPU0_SENSOR_BASE + CPU_TEMP_SENSOR1,
		.name	= DEVNAME,
		.flags	= IORESOURCE_MEM,
	},

	[1] = {
		.start	= CPU1_SENSOR_BASE + CPU_TEMP_SENSOR0,
		.end	= CPU1_SENSOR_BASE + CPU_TEMP_SENSOR1,
		.name	= DEVNAME,
		.flags	= IORESOURCE_MEM,
	},

	[2] = {
		.start	= CPU2_SENSOR_BASE + CPU_TEMP_SENSOR0,
		.end	= CPU2_SENSOR_BASE + CPU_TEMP_SENSOR1,
		.name	= DEVNAME,
		.flags	= IORESOURCE_MEM,
	},

	[3] = {
		.start	= CPU3_SENSOR_BASE + CPU_TEMP_SENSOR0,
		.end	= CPU3_SENSOR_BASE + CPU_TEMP_SENSOR1,
		.name	= DEVNAME,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device temp_device = {
	.name = DEVNAME,
	.id = -1,
	.num_resources = ARRAY_SIZE(temp_resource),
	.resource = temp_resource,
};

struct platform_driver temp_driver = {
	.probe	= probe_temp,
	.remove	= remove_temp,
	.driver = {
		.name	= DEVNAME,
		.owner	= THIS_MODULE,
	}
};

/**
 * Temperature handler.
 * @int node_num: NODE number.
 * Read sensor1, because the sensor0
 * can't read out right value.
 *
 * If open MODULE_ACTION, the halt and
 * warning value will be activation.
 *
 * Warning temperature: 70
 * Poweroff temerature: 85
 */
static void temp_handler(int node_num)
{
	unsigned char temp[4];
	int i;
        int temp_int[4];

	switch (cputype) {
		case Loongson_3A:
			for(i = 0; i < node_num; i++) {
				if ((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A3000) {
					temp_int[i] = *((volatile unsigned int *)temp_device.resource[i].start);
					temp[i] = (temp_int[i] & 0xffff)*731/0x4000 - 273;
				} else
					temp[i] = *((volatile unsigned char *)temp_device.resource[i].end);
				if ((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A2000)
					temp[i] -= 100;
				//if (temp[i] >= WARNING_TEMP)
				//	printk(KERN_INFO "Warning!! High CPU temperature! %d C\n", temp[i]);
#if 0 //MODULE_ACTION
				if (temp[i] >= POWEROFF_TEMP)
					goto poweroff;
#endif
#if PROC_PRINT
				cpu_temp.cpu_num = node_num;
				cpu_temp.temp[i] = temp[i];
#endif
			}
			break;
		case Loongson_3B:
			for(i = 0; i < node_num/2; i++) {
				temp[i] = *((unsigned char *)temp_device.resource[2*i].end) - 100;
				if (temp[i] >= WARNING_TEMP)
					printk(KERN_INFO "Warning!! High CPU temperature! %d C\n", temp[i]);
#if MODULE_ACTION
				if (temp[i] >= POWEROFF_TEMP)
					goto poweroff;
#endif
#if PROC_PRINT
				cpu_temp.cpu_num = node_num/2;
				cpu_temp.temp[i] = temp[i];
#endif
			}
			break;
		default:
			break;
	}
	mod_timer(&timer_temp, jiffies + 3 * HZ);

	return;

#if MODULE_ACTION
poweroff:
	printk(KERN_INFO "System will be poweroff.\n");
	pm_power_off();
#endif
}

static void check_temp_val(unsigned long data)
{
	temp_handler(nr_nodes_loongson);
}

static int probe_temp(struct platform_device *pdev)
{
	timer_temp.expires = jiffies + 3 * HZ;
	timer_temp.function = check_temp_val;
	timer_temp.data = (unsigned long)pdev;
	init_timer(&timer_temp);
	add_timer(&timer_temp);

	return 0;
}

static int remove_temp(struct platform_device *pdev)
{
	del_timer(&timer_temp);
	printk(KERN_INFO "Temperature protection function removed.\n");

	return 0;
}

static int __init platform_temp_init(void)
{
	if ((read_c0_prid() & 0xf) != PRID_REV_LOONGSON3A2000){
		platform_device_register(&temp_device);
		platform_driver_register(&temp_driver);
	}

	return 0;
}

static void __exit platform_temp_exit(void)
{
	if ((read_c0_prid() & 0xf) != PRID_REV_LOONGSON3A2000){
		platform_device_unregister(&temp_device);
		platform_driver_unregister(&temp_driver);
	}
}

MODULE_LICENSE("GPL");
module_init(platform_temp_init);
module_exit(platform_temp_exit);
