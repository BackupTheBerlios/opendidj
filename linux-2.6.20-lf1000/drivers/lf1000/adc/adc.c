/* 
 * drivers/lf1000/adc/adc.c
 *
 * LF1000 Analog to Digital Converter (ADC) Driver
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
#include <linux/types.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <asm/arch/adc.h>
#include "adc_priv.h"

extern int lf1000_CalcDivider(unsigned int rate_hz, unsigned int desired_mhz);

static struct adc_device adc = {
	.mem = NULL,
	.irq = -1,
	.lock = SPIN_LOCK_UNLOCKED,
	.busy = 0,
	.conversionFinished = 1,
};

static int adc_available(void)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&adc.lock, flags);
	ret = !adc.busy;
	spin_unlock_irqrestore(&adc.lock, flags);

	return ret;
}

/*********************
 * ADC API Functions *
 *********************/

int adc_GetReading(u8 channel)
{
	unsigned long flags;
	u16 tmp;
	int reading;

	if(channel > LF1000_ADC_MAX_CHANNEL)
		return -EINVAL;

	/*
	 * wait for the ADC
	 */

	spin_lock_irqsave(&adc.lock, flags);
	while(adc.busy) {
		spin_unlock_irqrestore(&adc.lock, flags);
		if (wait_event_interruptible(adc.wait, (adc_available())))
			return -ERESTARTSYS;
		spin_lock_irqsave(&adc.lock, flags);
	}
	adc.busy = 1;
	spin_unlock_irqrestore(&adc.lock, flags);

	/* Setup ADC */
	tmp = ioread16(adc.mem+ADCCON);
	tmp &= ~(0x7<<ASEL);
	tmp |= (channel<<ASEL);
	iowrite16(tmp, adc.mem+ADCCON);		// set channel
	adc.conversionFinished = 0;		// clear semaphore
	iowrite16(1, adc.mem+ADCINTCLR);	// clear pending bit
	iowrite16((u16)(1), adc.mem+ADCINTENB); // enable interrupt
	tmp |= (1<<ADEN);
	iowrite16(tmp, adc.mem+ADCCON);		// start conversion

	/* wait for conversion to finish */
	if(wait_event_interruptible(adc.meas_busy, adc.conversionFinished))
		return -ERESTARTSYS;

	/*
	 * release ADC
	 */

	spin_lock_irqsave(&adc.lock, flags);
	reading = ioread16(adc.mem+ADCDAT);
	adc.busy=0;
	spin_unlock_irqrestore(&adc.lock, flags);
	wake_up_interruptible(&adc.wait);
	return reading;
}
EXPORT_SYMBOL(adc_GetReading);

/*
 * sysfs Interface
 */

static ssize_t get_channel(struct device *dev, struct device_attribute *attr,
				char *buf, u8 channel)
{
	int reading = adc_GetReading(channel);

	if(reading >= 0)
		return sprintf(buf, "%d\n", reading);
	return sprintf(buf, "ERROR: %d\n", reading);
}

static ssize_t show_channel0(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	return get_channel(dev, attr, buf, 0);
}
static DEVICE_ATTR(channel0, S_IRUSR|S_IRGRP, show_channel0, NULL);

static ssize_t show_channel1(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	return get_channel(dev, attr, buf, 1);
}
static DEVICE_ATTR(channel1, S_IRUSR|S_IRGRP, show_channel1, NULL);

static ssize_t show_channel2(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	return get_channel(dev, attr, buf, 2);
}
static DEVICE_ATTR(channel2, S_IRUSR|S_IRGRP, show_channel2, NULL);

static ssize_t show_channel3(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	return get_channel(dev, attr, buf, 3);
}
static DEVICE_ATTR(channel3, S_IRUSR|S_IRGRP, show_channel3, NULL);

static struct attribute *adc_attributes[] = {
	&dev_attr_channel0.attr,
	&dev_attr_channel1.attr,
	&dev_attr_channel2.attr,
	&dev_attr_channel3.attr,
	NULL
};

static struct attribute_group adc_attr_group = {
	.attrs = adc_attributes
};

/**********************
 * interrupt handling *
 **********************/

static irqreturn_t adc_irq(int irq, void *dev_id)
{
	/* clear the pending interrupt */
	iowrite16(1, adc.mem+ADCINTCLR);
	/* disable interrupt */
	iowrite16((u16)(0), adc.mem+ADCINTENB); 

	adc.conversionFinished = 1;

	wake_up_interruptible(&adc.meas_busy);
	return IRQ_HANDLED;
}

static int lf1000_adc_remove(struct platform_device *pdev)
{
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	sysfs_remove_group(&pdev->dev.kobj, &adc_attr_group);

	if(adc.irq != -1)
		free_irq(adc.irq, NULL);

	if(adc.mem != NULL) {
		iowrite16(0, adc.mem+ADCCON); /* turn off ADC */
		iounmap(adc.mem);
		release_mem_region(res->start, (res->end - res->start)+1);
	}
	return 0;
}

static int lf1000_adc_probe(struct platform_device *pdev)
{
	int ret = 0;
	int div;
	u16 tmp;
	struct resource *res;

	div = lf1000_CalcDivider(get_pll_freq(PLL1), 1000000);
	if(div < 0) {
		printk(KERN_ERR "adc: failed to get a ADC divider\n");
		return -EFAULT;
	}
	if(div > 0xFF) {
		printk(KERN_INFO "adc: warning, clipping ADC divider\n");
		div = 0xFF;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(res == NULL) {
		printk(KERN_ERR "adc: failed to get resource\n");
		return -ENXIO;
	}

	if(!request_mem_region(res->start, (res->end - res->start)+1, 
				"lf1000-adc")) {
		printk(KERN_ERR "adc: failed to map ADC region\n");
		return -EBUSY;
	}
	adc.mem = ioremap_nocache(res->start, (res->end - res->start)+1);
	if(!adc.mem) {
		printk(KERN_ERR "adc: failed to ioremap\n");
		goto fail_remap;
	}

	adc.irq = platform_get_irq(pdev, 0);
	if(adc.irq < 0) {
		printk(KERN_ERR "adc: failed to get IRQ\n");
		ret = adc.irq;
		goto fail_irq;
	}
	ret = request_irq(adc.irq, adc_irq, SA_INTERRUPT|SA_SAMPLE_RANDOM,
			"adc", NULL);
	if(ret != 0) {
		printk(KERN_ERR "adc: requesting IRQ failed\n");
		goto fail_irq;
	}

	init_waitqueue_head(&adc.wait);
	init_waitqueue_head(&adc.meas_busy);

	/***********************
	 * set up the hardware *
	 ***********************/

	/* enable PCLK, its only allowed mode is 'always' */
	iowrite32((u32)(1<<3), adc.mem+ADCCLKENB);
	/* disable converter */
	tmp = ioread16(adc.mem+ADCCON);
	tmp &= ~(1<<APEN);
	iowrite16(tmp, adc.mem+ADCCON);
	/* remove old prescaler */
	tmp &= ~(0xFF << APSV);
	/* power is off, set prescaler */
	tmp = (div<<APSV);
	iowrite16(tmp, adc.mem+ADCCON);
	/* enable clock */
	tmp |= (1<<APEN);
	iowrite16(tmp, adc.mem+ADCCON);
	/* power on the ADC */
	tmp &= ~(1<<STBY);
	iowrite16(tmp, adc.mem+ADCCON);

	sysfs_create_group(&pdev->dev.kobj, &adc_attr_group);
	return 0;

fail_irq:
	iounmap(adc.mem);
fail_remap:
	release_mem_region(res->start, (res->end - res->start) + 1);
	
	return ret;
}

#ifdef CONFIG_PM
static int lf1000_adc_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int lf1000_adc_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define lf1000_adc_suspend	NULL
#define lf1000_adc_resume	NULL
#endif

static struct platform_driver lf1000_adc_driver = {
	.probe      = lf1000_adc_probe,
	.remove     = lf1000_adc_remove,
	.suspend    = lf1000_adc_suspend,
	.resume     = lf1000_adc_resume,
	.driver     = {
		.name   = "lf1000-adc",
		.owner  = THIS_MODULE,
	},
};

static int __init adc_init(void)
{
	return platform_driver_register(&lf1000_adc_driver);
}

static void __exit adc_cleanup(void)
{
	platform_driver_unregister(&lf1000_adc_driver);
}

module_init(adc_init);
module_exit(adc_cleanup);

MODULE_AUTHOR("Andrey Yurovsky");
MODULE_LICENSE("GPL");
MODULE_VERSION("1:2.0");
