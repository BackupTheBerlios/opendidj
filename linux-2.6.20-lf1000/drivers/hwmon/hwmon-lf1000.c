/* hwmon-lf1000.c
 *
 * Power and Battery monitoring.  This driver provides battery and external
 * power information, as well as an input device for the Power button.
 * 
 * Andrey Yurovsky <andrey@cozybit.com>
 * Scott Esters <sesters@leapfrog.com>
 *
 * Copyright 2008 LeapFrog Enterprises Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/input.h>

#include <asm/arch/adc.h>
#include <asm/arch/gpio.h>
#include <asm/arch/gpio_hal.h>
#include <asm/arch/common.h>
#include <asm/arch/clkpwr.h>
#include <asm/arch/power.h>

#include <asm/io.h>
#include <asm/uaccess.h>

/*
 * pins
 */

#define POWER_PORT	GPIO_PORT_C	/* power button */
#define POWER_PIN	GPIO_PIN20
#define	LOW_BATT_PORT	GPIO_PORT_C	/* external power detect */
#define LOW_BATT_PIN	GPIO_PIN14

/*
 * configuration
 */

#define POWER_SAMPLING_J	(HZ/4)

/* Must be a power of 2 */
#define POWER_Q_SIZE 4
/* return val+1 or val-1 wrapped around.  size must be power of 2! */
#define INC_CIRCULAR(val, size) ((val+1)&(size-1))
#define DEC_CIRCULAR(val, size) ((val-1)&(size-1))

/* determine power source by measuring input voltage in milliVolts */

#define ADC_EXTERNAL_MV		8000	/* external power, above 8 volts */
					/* below 8 volts is battery      */

/*
 * model ADC as a line:
 *  milliVolts = ((ADC_SLOPE * 256) * READING) / 256 + ADC_CONSTANT
 */

#if defined (CONFIG_MACH_ME_MP2530F) || defined (CONFIG_MACH_LF_MP2530F)
#define ADC_SLOPE_256		1943	/* ADC slope * 256		*/
#define ADC_CONSTANT		1125	/* ADC constant			*/
#elif defined (CONFIG_MACH_ME_LF1000)
#define ADC_SLOPE_256		2322	/* ADC slope * 256		*/
#define ADC_CONSTANT		1267	/* ADC constant			*/
#elif defined (CONFIG_MACH_LF_LF1000)
#define ADC_SLOPE_256		2012	/* ADC slope * 256		*/
#define ADC_CONSTANT		0	/* ADC constant			*/
#else
#warning CPU type undefined
#endif

/* platform device data */
struct lf1000_hwmon {
	enum lf1000_power_status status;/* board status */

	int supply_mv;				/* power supply, in mV */
	unsigned have_external :1;		/* have external power */
	unsigned shutdown : 1;			/* shutdown requested */

	void *mem;

	struct work_struct battery_work;	/* monitor power */
	struct workqueue_struct *battery_tasks;
	struct timer_list battery_timer;

	/* input device interface */
	unsigned char buttons[2];
	struct input_dev *input;
};

static struct lf1000_hwmon *hwmon_dev = NULL;

#define ADC_TO_MV(r)	(((ADC_SLOPE_256 * reading) / 256) + ADC_CONSTANT)

/* return battery reading in millivolts */
int lf1000_get_battery_mv(void)
{
	int reading = adc_GetReading(LF1000_ADC_VBATSENSE);
	
	if(reading < 0) /* pass the error code down */
		return reading;

	return ADC_TO_MV(reading);
}
EXPORT_SYMBOL_GPL(lf1000_get_battery_mv);

/*
 * Convert battery reading to power status based on current status 
 */
static enum lf1000_power_status power_to_status(
		enum lf1000_power_status status, u8 have_external, int mv)
{
	if(have_external) /* no need to make measurements */
		return EXTERNAL;
	
	if(mv < 0)
		return UNKNOWN;

	if(mv < ADC_CRITICAL_BATTERY_MV)
		return CRITICAL_BATTERY;
	
	/* hysterisis between 'low' and 'normal' battery */
	if(mv < ADC_LOW_BATTERY_MV || 
		(mv < ADC_NORMAL_BATTERY_MV && status == LOW_BATTERY)) {
		return LOW_BATTERY;
	}
	if(mv < ADC_EXTERNAL_MV)
		return BATTERY;
	
	return EXTERNAL;
}

/*
 * Determine and report current power status
 */
enum lf1000_power_status lf1000_get_battery_status(void)
{
	if(!hwmon_dev)
		return UNKNOWN;
	return power_to_status(hwmon_dev->status, hwmon_dev->have_external,
			lf1000_get_battery_mv());
}
EXPORT_SYMBOL_GPL(lf1000_get_battery_status);

/*
 * sysfs Interface
 */

/* report whether shutdown was requested */
static ssize_t show_shutdown(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct lf1000_hwmon *priv = (struct lf1000_hwmon *)dev->driver_data;

	return sprintf(buf, "%d\n", priv->shutdown);
}
static DEVICE_ATTR(shutdown, S_IRUSR|S_IRGRP, show_shutdown, NULL);

/* report current battery voltage, in mV */
static ssize_t show_voltage(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct lf1000_hwmon *priv = (struct lf1000_hwmon *)dev->driver_data;

	return sprintf(buf, "%d\n", priv->supply_mv);
}
static DEVICE_ATTR(voltage, S_IRUSR|S_IRGRP, show_voltage, NULL);

/* report state of external power (1 = on external, 0 = on battery) */
static ssize_t show_external(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct lf1000_hwmon *priv = (struct lf1000_hwmon *)dev->driver_data;

	return sprintf(buf, "%d\n", priv->have_external);
}
static DEVICE_ATTR(external, S_IRUSR|S_IRGRP, show_external, NULL);

/* report power status as a number */
static ssize_t show_status(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct lf1000_hwmon *priv = (struct lf1000_hwmon *)dev->driver_data;

	return sprintf(buf, "%d\n", priv->status);
}
static DEVICE_ATTR(status, S_IRUSR|S_IRGRP, show_status, NULL);

static struct attribute *power_attributes[] = {
	&dev_attr_shutdown.attr,
	&dev_attr_voltage.attr,
	&dev_attr_external.attr,
	&dev_attr_status.attr,
	NULL
};

static struct attribute_group power_attr_group = {
	.attrs = power_attributes
};

/*
 * handle the Power Button and report it
 */
static irqreturn_t power_button_irq(enum gpio_port port, enum gpio_pin pin, 
					void *priv) 
{
	struct lf1000_hwmon *hw = (struct lf1000_hwmon *)priv;

	if(gpio_get_pend(POWER_PORT, POWER_PIN)) {
		gpio_set_int(POWER_PORT, POWER_PIN, 0);
		gpio_clear_pend(POWER_PORT, POWER_PIN);
		input_report_key(hw->input, KEY_POWER, 1);
		hw->shutdown = 1;
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

static void lf1000_set_battery(struct work_struct *work)
{
#ifdef CONFIG_MACH_LF_LF1000
	/* get external power status */
	hwmon_dev->have_external = gpio_get_val(LOW_BATT_PORT, LOW_BATT_PIN);
#endif
	/* read the current battery voltage */
	hwmon_dev->supply_mv = lf1000_get_battery_mv();
	/* determine the current power status */
	hwmon_dev->status = power_to_status(hwmon_dev->status, 
					    hwmon_dev->have_external, 
					    hwmon_dev->supply_mv);
	/* report critical battery, if we have it */
	if(hwmon_dev->status == CRITICAL_BATTERY)
		input_report_key(hwmon_dev->input, KEY_BATTERY, 1);
}

static void battery_monitoring_task(unsigned long data)
{
	struct lf1000_hwmon *priv = (struct lf1000_hwmon *)data;

	queue_work(priv->battery_tasks, &priv->battery_work);
	priv->battery_timer.expires += POWER_SAMPLING_J;
	priv->battery_timer.function = battery_monitoring_task;
	priv->battery_timer.data = data;
	add_timer(&priv->battery_timer);
}

/*
 * set up input device for power button and critical battery
 */
static int setup_power_button(struct platform_device *pdev)
{
	struct lf1000_hwmon *data = platform_get_drvdata(pdev);
	struct input_dev *input_dev;
	int ret;

	input_dev = input_allocate_device();
	if(!input_dev)
		return -ENOMEM;

	input_dev->name = "Power Button";
	input_dev->phys = "lf1000/power_button";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0001;
	input_dev->cdev.dev = &pdev->dev;
	input_dev->private = input_dev;
	
	input_dev->evbit[0] = BIT(EV_KEY);
	input_dev->keycode = data->buttons;
	input_dev->keycodesize = sizeof(unsigned char);
	input_dev->keycodemax = 2;

	data->input = input_dev;
	data->buttons[0] = KEY_POWER;	/* we only support power button */
	data->buttons[1] = KEY_BATTERY;	/* and critical battery warning */
	set_bit(data->buttons[0], input_dev->keybit);
	set_bit(data->buttons[1], input_dev->keybit);

	ret = input_register_device(data->input);
	if(ret)
		goto fail_register;

	/* claim the power button IRQ */

	ret = gpio_request_irq(POWER_PORT, POWER_PIN, power_button_irq, data);
	if(ret) {
		printk(KERN_ALERT "lf1000-power: failed to get button IRQ\n");
		goto fail_irq;
	}
	
	/* enable interrupt */

	gpio_set_fn(POWER_PORT, POWER_PIN, GPIO_GPIOFN);
	gpio_set_int_mode(POWER_PORT, POWER_PIN, GPIO_IMODE_RISING_EDGE);
	gpio_clear_pend(POWER_PORT, POWER_PIN);
	gpio_set_int(POWER_PORT, POWER_PIN, 1);

	return 0;

fail_irq:
	gpio_free_irq(POWER_PORT, POWER_PIN, power_button_irq);
fail_register:
	input_free_device(input_dev);
	return ret;
}

static int lf1000_power_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	struct lf1000_hwmon *priv;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res) {
		printk(KERN_ERR "%s: failed to get resource\n", __FUNCTION__);
		return -ENXIO;
	}

	if(!request_mem_region(res->start, (res->end - res->start) + 1,
				"lf1000-power")) {
		printk(KERN_ERR "%s: failed to map region\n", __FUNCTION__);
		return -EBUSY;
	}

	priv = kzalloc(sizeof(struct lf1000_hwmon), GFP_KERNEL);
	if(!priv) {
		ret = -ENOMEM;
		goto fail_alloc;
	}

	priv->mem = ioremap_nocache(res->start, (res->end - res->start) + 1);
	if(priv->mem == NULL) {
		printk(KERN_ERR "%s: failed to ioremap\n", __FUNCTION__);
		ret = -ENOMEM;
		goto fail_remap;
	}

	platform_set_drvdata(pdev, priv);

	hwmon_dev = priv;

	ret = setup_power_button(pdev);
	if(ret)
		goto fail_button;

#ifdef CONFIG_MACH_LF_LF1000
	gpio_set_fn(LOW_BATT_PORT, LOW_BATT_PIN, GPIO_GPIOFN);
	priv->have_external = gpio_get_val(LOW_BATT_PORT, LOW_BATT_PIN);
#else /* CONFIG_MACH_ME_LF1000 */
	priv->have_external = 1; /* always on external */
#endif

	/* grab initial battery setting */
	priv->supply_mv = lf1000_get_battery_mv();
	if(priv->supply_mv < 0) {
		printk(KERN_ERR "lf1000-power: initial battery read failed\n");
		goto fail_adc;
	}

	/* set initial power state */
	priv->status = power_to_status(priv->status, priv->have_external, 
					priv->supply_mv);

	/* set up work queue to monitor the power */
	priv->battery_tasks = create_singlethread_workqueue("battery tasks");
	INIT_WORK(&priv->battery_work, lf1000_set_battery);

	/* set up periodic sampling of the power */
	setup_timer(&priv->battery_timer, battery_monitoring_task,
		       (unsigned long)priv);
	priv->battery_timer.expires = get_jiffies_64() + POWER_SAMPLING_J;
	add_timer(&priv->battery_timer);

	sysfs_create_group(&pdev->dev.kobj, &power_attr_group);
	return 0;

fail_adc:
fail_button:
	iounmap(priv->mem);
fail_alloc:
	kfree(priv);
fail_remap:
	release_mem_region(res->start, (res->end - res->start) + 1);
	return ret;
}

static int lf1000_power_remove(struct platform_device *pdev)
{
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct lf1000_hwmon *priv = platform_get_drvdata(pdev);

	gpio_set_int(POWER_PORT, POWER_PIN, 0);
	gpio_free_irq(POWER_PORT, POWER_PIN, power_button_irq);

	destroy_workqueue(priv->battery_tasks);

	sysfs_remove_group(&pdev->dev.kobj, &power_attr_group);

	input_unregister_device(priv->input);

	if(priv->mem != NULL)
		iounmap(priv->mem);
	release_mem_region(res->start, (res->end - res->start) + 1);

	kfree(priv);

	return 0;
}

static struct platform_driver lf1000_power_driver = {
	.probe      = lf1000_power_probe,
	.remove     = lf1000_power_remove,
	.driver     = {
		.name	= "lf1000-power",
		.owner	= THIS_MODULE,
	},
};

/*
 * module stuff
 */
 
static int __init init_lf1000_power(void)
{
	return platform_driver_register(&lf1000_power_driver);
}

static void cleanup_lf1000_power(void)
{
	platform_driver_unregister(&lf1000_power_driver);
}

module_init(init_lf1000_power);
module_exit(cleanup_lf1000_power);

MODULE_AUTHOR("Scott Esters, Andrey Yurovsky");
MODULE_DESCRIPTION("LF1000 hardware monitoring");
MODULE_LICENSE("GPL");
