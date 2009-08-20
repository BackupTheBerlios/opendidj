/* 
 * arch/arm/mach-lf1000/gpio_main.c
 *
 * LF1000 General-Purpose IO (GPIO) Driver 
 *
 * Copyright 2007 LeapFrog Enterprises Inc.
 *
 * Andrey Yurovsky <andrey@cozybit.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/stat.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include "gpio_priv.h"
#include <asm/arch/gpio.h>
#include <asm/arch/common.h>
#include <asm/arch/gpio_hal.h>
#include <linux/lf1000/gpio_ioctl.h>

extern int lf1000_CalcDivider(unsigned int rate_hz, unsigned int desired_mhz);
extern u32 gpio_get_scratch(void);

/***********************
 * device private data *
 ***********************/

struct gpio_device gpio = {
	.mem = NULL,
	.mem_cur = NULL,
	.amem = NULL,
	.irq = -1,
};

/*******************
 * sysfs interface *
 *******************/

#ifdef CPU_LF1000
static ssize_t show_scratchpad(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%08X\n", (unsigned int)gpio_get_scratch());
}
static DEVICE_ATTR(scratchpad, S_IRUSR|S_IRGRP, show_scratchpad, NULL);
#endif

static ssize_t show_board_id(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%X\n", gpio_get_board_config());
}
static DEVICE_ATTR(board_id, S_IRUSR|S_IRGRP, show_board_id, NULL);

static ssize_t show_cart_id(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%X\n", gpio_get_cart_config());
}
static DEVICE_ATTR(cart_id, S_IRUSR|S_IRGRP, show_cart_id, NULL);

#ifdef CONFIG_LF1000_GPIO_DEBUG
static int get_port(u8 port, char *buf)
{
	int len = 0;
	int reg = port*0x40;
	char x = 'A'+port;
	void *base = gpio.mem;

	len += sprintf(buf+len,"GPIO%cOUT      = 0x%08X\n", x,
				   ioread32(base+GPIOAOUT+reg));
	len += sprintf(buf+len,"GPIO%cOUTENB   = 0x%08X\n", x,
				   ioread32(base+GPIOAOUTENB+reg));
	len += sprintf(buf+len,"GPIO%cDETMODE0 = 0x%08X\n", x,
				   ioread32(base+GPIOADETMODE0+reg));
	len += sprintf(buf+len,"GPIO%cDETMODE1 = 0x%08X\n", x,
				   ioread32(base+GPIOADETMODE1+reg));
	len += sprintf(buf+len,"GPIO%cINTENB   = 0x%08X\n", x,
				   ioread32(base+GPIOAINTENB+reg));
	len += sprintf(buf+len,"GPIO%cDET      = 0x%08X\n", x,
				   ioread32(base+GPIOADET+reg));
	len += sprintf(buf+len,"GPIO%cPAD      = 0x%08X\n", x,
				   ioread32(base+GPIOAPAD+reg));
	len += sprintf(buf+len,"GPIO%cPUENB    = 0x%08X\n", x,
				   ioread32(base+GPIOAPUENB+reg));
	len += sprintf(buf+len,"GPIO%cALTFN0   = 0x%08X\n", x,
				   ioread32(base+GPIOAALTFN0+reg));
	len += sprintf(buf+len,"GPIO%cALTFN1   = 0x%08X\n", x,
				   ioread32(base+GPIOAALTFN1+reg));

	return len;
}

static ssize_t show_portA(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return get_port(0, buf);
}
static DEVICE_ATTR(port_A, S_IRUSR|S_IRGRP, show_portA, NULL);

static ssize_t show_portB(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return get_port(1, buf);
}
static DEVICE_ATTR(port_B, S_IRUSR|S_IRGRP, show_portB, NULL);

static ssize_t show_portC(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return get_port(2, buf);
}
static DEVICE_ATTR(port_C, S_IRUSR|S_IRGRP, show_portC, NULL);

#ifndef CPU_LF1000
static ssize_t show_portD(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf,"GPIODALTFN0= 0x%08X\n",
				ioread32(gpio.mem+GPIODALTFN0));
}
static DEVICE_ATTR(port_D, S_IRUSR|S_IRGRP, show_portD, NULL);
#endif

static ssize_t show_alive(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int len = 0;

#ifdef CPU_LF1000
	len += sprintf(buf+len, "ALIVEPWRGATEREG     = 0x%08X\n",
			ioread32(gpio.amem+ALIVEPWRGATEREG));
	len += sprintf(buf+len, "ALIVEGPIORSTREG     = 0x%08X\n",
			ioread32(gpio.amem+ALIVEGPIORSTREG));
	len += sprintf(buf+len, "ALIVEGPIOSETREG     = 0x%08X\n",
			ioread32(gpio.amem+ALIVEGPIOSETREG));
	len += sprintf(buf+len, "ALIVEGPIOREADREG    = 0x%08X\n",
			ioread32(gpio.amem+ALIVEGPIOREADREG));
	len += sprintf(buf+len, "ALIVESCRATCHRSTREG  = 0x%08X\n",
			ioread32(gpio.amem+ALIVESCRATCHRSTREG));
	len += sprintf(buf+len, "ALIVESCRATCHSETREG  = 0x%08X\n",
			ioread32(gpio.amem+ALIVESCRATCHSETREG));
	len += sprintf(buf+len, "ALIVESCRATCHREADREG = 0x%08X\n",
			ioread32(gpio.amem+ALIVESCRATCHREADREG));
#elif defined CPU_MF2530F
	len += sprintf(buf+len, "GPIOALVOUT          = 0x%08X\n",
			ioread32(gpio.amem+GPIOALVOUT));
	len += sprintf(buf+len, "GPIOALVOUTENB       = 0x%08X\n",
			ioread32(gpio.amem+GPIOALVOUTENB));
	len += sprintf(buf+len, "GPIOALVDETMODE0     = 0x%08X\n",
			ioread32(gpio.amem+GPIOALVDETMODE0));
	len += sprintf(buf+len, "GPIOALVINTENB       = 0x%08X\n",
			ioread32(gpio.amem+GPIOALVINTENB));
	len += sprintf(buf+len, "GPIOALVPEND         = 0x%08X\n",
			ioread32(gpio.amem+GPIOALVPEND));
	len += sprintf(buf+len, "GPIOALVPAD          = 0x%08X\n",
			ioread32(gpio.amem+GPIOALVPAD));
	len += sprintf(buf+len, "GPIOALVPUENB        = 0x%08X\n",
			ioread32(gpio.amem+GPIOALVPUENB));
	len += sprintf(buf+len, "GPIOALVDEP          = 0x%08X\n",
			ioread32(gpio.amem+GPIOALVDEP));
#endif /* CPU_MF2530F */
	return len;
}
static DEVICE_ATTR(port_alive, S_IRUSR|S_IRGRP, show_alive, NULL);
#endif /* CONFIG_LF1000_GPIO_DEBUG */

static struct attribute *gpio_attributes[] = {
	&dev_attr_board_id.attr,
	&dev_attr_cart_id.attr,
#ifdef CONFIG_LF1000_GPIO_DEBUG
	&dev_attr_port_A.attr,
	&dev_attr_port_B.attr,
	&dev_attr_port_C.attr,
#ifdef CPU_MF2530F
	&dev_attr_port_D.attr,
#endif
#endif /* CONFIG_LF1000_GPIO_DEBUG */
	NULL
};

static struct attribute_group gpio_attr_group = {
	.attrs = gpio_attributes
};

static struct attribute *alive_attributes[] = {
#ifdef CONFIG_LF1000_GPIO_DEBUG
	&dev_attr_port_alive.attr,
#endif /* CONFIG_LF1000_GPIO_DEBUG */
#ifdef CPU_LF1000
	&dev_attr_scratchpad.attr,
#endif /* CPU_LF1000 */
	NULL,
};

static struct attribute_group alive_attr_group = {
	.attrs = alive_attributes
};

/*******************************
 * character device operations *
 *******************************/

int gpio_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
			  unsigned long arg)
{
	int retval = 0;
	void __user *argp = (void __user *)arg;
	union gpio_cmd c;

	switch(cmd) {
		case GPIO_IOCSOUTVAL:
		if(!(_IOC_DIR(cmd) & _IOC_WRITE))
			return -EFAULT;
		if(copy_from_user((void *)&c, argp, 
				sizeof(struct outvalue_cmd)))
			return -EFAULT;
		retval = gpio_set_val(c.outvalue.port, c.outvalue.pin, 
					c.outvalue.value);
		if(retval)
			retval = -EFAULT;
		break;

		case GPIO_IOCSOUTENB:
		if(!(_IOC_DIR(cmd) & _IOC_WRITE))
			return -EFAULT;
		if(copy_from_user((void *)&c, argp, sizeof(struct outenb_cmd)))
			return -EFAULT;
		retval = gpio_set_out_en(c.outenb.port, c.outenb.pin, 
				c.outenb.value);
		if(retval)
			retval = -EFAULT;
		break;

		case GPIO_IOCXINVAL:
		if(copy_from_user((void *)&c, argp, sizeof(struct invalue_cmd)))
			return -EFAULT;
		c.invalue.value = gpio_get_val(c.invalue.port, c.invalue.pin);
		if(copy_to_user(argp, (void *)&c, sizeof(struct invalue_cmd)))
			return -EFAULT;
		retval = 0;
		break;

		case GPIO_IOCSFUNC:
		if(!(_IOC_DIR(cmd) & _IOC_WRITE))
			return -EFAULT;
		if(copy_from_user((void *)&c, argp, sizeof(struct func_cmd)))
			return -EFAULT;
		retval = gpio_set_fn(c.func.port, c.func.pin, c.func.func);
		if(retval)
			retval = -EFAULT;
		break;

		case GPIO_IOCXFUNC:
		if(copy_from_user((void *)&c, argp, sizeof(struct func_cmd)))
			return -EFAULT;
		retval = gpio_get_fn(c.func.port, c.func.pin);
		if(retval < 0)
			return -EFAULT;
		c.func.func = retval;
		if(copy_to_user(argp, (void *)&c, sizeof(struct func_cmd)))
			return -EFAULT;
		retval = 0;
		break;

		default: /* unknown command */
		return -ENOTTY;
	}
	return retval;
}

struct file_operations gpio_fops = {
	.owner = THIS_MODULE,
	.ioctl = gpio_ioctl,
};

/*************************
 * interrupt hanlding    *
 *************************/
static irqreturn_t gpio_irq(int irq, void *dev_id)
{
	enum gpio_pin pin;
	enum gpio_port port;
	unsigned int pins;
	irqreturn_t ret = IRQ_NONE;
	unsigned long flags;
	struct gpio_handler *gh;

	/* Scan through all of the pins.  When you find the source, invoke the
	 * handler.  Return after handling one interrupt.  If others are
	 * pending, we'll be invoked again. 
	 */
	for( port = GPIO_PORT_A; port <= GPIO_PORT_C; port++) {
		pins = gpio_get_pend32(port);
		for( pin = GPIO_PIN0; pin <= GPIO_PIN31; pin++) {
			if(!gpio_get_int(port, pin))
				continue;
			if(pins & (0x1<<pin)) {
				spin_lock_irqsave(&gpio_handlers_lock, flags);
				gh = &gpio_handlers[port][pin];
				if(!gh->handler.handler) {
					/* Avoid spurious interrupts */
					gpio_clear_pend(port, pin);
					ret = IRQ_HANDLED;
				} else if(gh->mode_gpio) {
					ret = gh->handler.gpio_handler(port, 
								 pin, 
								 gh->priv);
				} else {
					ret = gh->handler.normal_handler(irq, 
								   gh->priv);
				}
				spin_unlock_irqrestore(&gpio_handlers_lock,
						       flags);
				break;
			}
		}
	}
	return ret;
}

/*************************
 * device initialization *
 *************************/

static int lf1000_alvgpio_remove(struct platform_device *pdev)
{
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	sysfs_remove_group(&pdev->dev.kobj, &alive_attr_group);

	if(gpio.amem != NULL) {
		iounmap(gpio.amem);
		release_mem_region(res->start, (res->end - res->start) + 1);
	}
	return 0;
}

static int lf1000_gpio_remove(struct platform_device *pdev)
{
	struct resource *res;

	sysfs_remove_group(&pdev->dev.kobj, &gpio_attr_group);

	if(gpio.irq != -1) {
		free_irq(gpio.irq, NULL);
		gpio.irq = -1;
	}

	cdev_del(&gpio.cdev);

#ifdef CPU_LF1000
	if(gpio.mem_cur != NULL) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		iounmap(gpio.mem_cur);
		release_mem_region(res->start, (res->end - res->start) + 1);
	}
#endif

	if(gpio.mem != NULL) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		iounmap(gpio.mem);
		release_mem_region(res->start, (res->end - res->start) + 1);
	}

	return 0;
}

static int lf1000_gpio_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
#ifdef CPU_LF1000
	struct resource *res_cur;
#endif
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res) {
		printk(KERN_ERR "gpio: failed to get resource\n");
		return -ENXIO;
	}

	if(!request_mem_region(res->start, (res->end - res->start)+1, 
			"lf1000_gpio")) {
		printk(KERN_ERR "gpio: failed to get region\n");
		return -EBUSY;
	}

	gpio.mem = ioremap_nocache(res->start, (res->end - res->start)+1);
	if(gpio.mem == NULL) {
		printk(KERN_ERR "gpio: failed to remap\n");
		ret = -ENOMEM;
		goto fail_remap;
	}

#ifdef CPU_LF1000
	res_cur = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if(!res_cur) {
		printk(KERN_ERR "gpio: failed to get resource\n");
		ret = -ENXIO;
		goto fail_remap;
	}

	if(!request_mem_region(res_cur->start,
			       (res_cur->end - res_cur->start) + 1, 
			       "lf1000_gpio_cur")) {
		printk(KERN_ERR "gpio: failed to get region\n");
		ret = -EBUSY;
		goto fail_remap;
	}

	gpio.mem_cur = ioremap_nocache(res_cur->start,
				       (res_cur->end - res_cur->start) + 1);
	if(gpio.mem_cur == NULL) {
		printk(KERN_ERR "gpio: failed to remap\n");
		ret = -ENOMEM;
		goto fail_remap_cur;
	}
#endif

	/* turn off GPIO interrupts */
	gpio_set_int32(GPIO_PORT_A, 0);
	gpio_set_int32(GPIO_PORT_B, 0);
	gpio_set_int32(GPIO_PORT_C, 0);

	gpio.devnum = MKDEV(GPIO_MAJOR, 0);
	cdev_init(&gpio.cdev, &gpio_fops);
	gpio.cdev.owner = THIS_MODULE;
	gpio.cdev.ops = &gpio_fops;
	ret = cdev_add(&gpio.cdev, gpio.devnum, 1);
	if(ret) {
		printk(KERN_ALERT "gpio: failed to get a device\n");
		goto fail_dev;
	}

	gpio.irq = platform_get_irq(pdev, 0);
	if(gpio.irq < 0) {
		printk(KERN_INFO "gpio: failed to get IRQ\n");
		ret = gpio.irq;
		goto fail_irq;
	}
	ret = request_irq(gpio.irq, gpio_irq, SA_INTERRUPT|SA_SAMPLE_RANDOM,
			"gpio", NULL);
	if(ret) {
		printk(KERN_ERR "gpio: requesting IRQ failed\n");
		goto fail_irq;
	}
	
	sysfs_create_group(&pdev->dev.kobj, &gpio_attr_group);

	return 0;

fail_irq:
	cdev_del(&gpio.cdev);
fail_dev:
	iounmap(gpio.mem);
#ifdef CPU_LF1000
fail_remap_cur:
	release_mem_region(res_cur->start, (res_cur->end - res_cur->start) + 1);
#endif
fail_remap:
	release_mem_region(res->start, (res->end - res->start) + 1);
	
	return ret;
}

static int lf1000_alvgpio_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res) {
		printk(KERN_ERR "alvgpio: failed to get resource\n");
		return -ENXIO;
	}

	if(!request_mem_region(res->start, (res->end - res->start)+1, 
			"Alive GPIO")) {
		printk(KERN_ERR "alvgpio: failed to get region\n");
		return -EBUSY;
	}
	gpio.amem = ioremap_nocache(res->start, (res->end - res->start)+1);
	if(!gpio.amem) {
		printk(KERN_ERR "alvgpio: failed to remap\n");
		ret = -ENOMEM;
		goto fail_remap;
	}

#ifdef CPU_MF2530F
	/* turn off interrupt */
	gpio_set_int32(GPIO_PORT_ALV, 0);

#else /* CPU_LF1000 */
	/*
	 * When ALIVEPWRGATEREG is 1, software can write ALV bits.  Otherwise,
	 * last written values are held in flip flops.  This should be power
	 * down/up function of gpio driver, transparent to the user of this API.
	 * Perhaps in suspend/resume?
	 */

	/* init ALIVE S/R input registers to reset value */
	iowrite32(1 << NPOWERGATING, gpio.amem + ALIVEPWRGATEREG);
	iowrite32(0, gpio.amem + ALIVEGPIOSETREG);
	iowrite32(0, gpio.amem + ALIVEGPIORSTREG);
	iowrite32(0 << NPOWERGATING, gpio.amem + ALIVEPWRGATEREG);
#endif
	
	/* clear out all handlers */
	memset(gpio_handlers, 0,
	       sizeof(struct gpio_handler)*(GPIO_PORT_ALV+1)*(GPIO_PIN31+1));

	sysfs_create_group(&pdev->dev.kobj, &alive_attr_group);
	return 0;

fail_remap:
	release_mem_region(res->start, (res->end - res->start) + 1);
	
	return ret;
}

#ifdef CONFIG_PM
static int lf1000_gpio_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int lf1000_gpio_resume(struct platform_device *pdev)
{
	return 0;
}

static int lf1000_alvgpio_suspend(struct platform_device *pdev, 
				  pm_message_t mesg)
{
	return 0;
}

static int lf1000_alvgpio_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define lf1000_gpio_suspend	NULL
#define lf1000_gpio_resume	NULL
#define lf1000_alvgpio_suspend	NULL
#define lf1000_alvgpio_resume	NULL
#endif

static struct platform_driver lf1000_gpio_driver = {
	.probe      = lf1000_gpio_probe,
	.remove     = lf1000_gpio_remove,
	.suspend    = lf1000_gpio_suspend,
	.resume     = lf1000_gpio_resume,
	.driver     = {
		.name   = "lf1000-gpio",
		.owner  = THIS_MODULE,
	},
};

static struct platform_driver lf1000_alvgpio_driver = {
	.probe      = lf1000_alvgpio_probe,
	.remove     = lf1000_alvgpio_remove,
	.suspend    = lf1000_alvgpio_suspend,
	.resume     = lf1000_alvgpio_resume,
	.driver     = {
		.name   = "lf1000-alvgpio",
		.owner  = THIS_MODULE,
	},
};

static int __init gpio_init(void)
{
	int ret = platform_driver_register(&lf1000_alvgpio_driver);
	if(ret != 0)
		return ret;
	return platform_driver_register(&lf1000_gpio_driver);
}

static void __exit gpio_cleanup(void)
{
	platform_driver_unregister(&lf1000_gpio_driver);
	platform_driver_unregister(&lf1000_alvgpio_driver);
}

module_init(gpio_init);
module_exit(gpio_cleanup);
MODULE_AUTHOR("Andrey Yurovsky");
MODULE_VERSION("1:2.0");
MODULE_LICENSE("GPL");
