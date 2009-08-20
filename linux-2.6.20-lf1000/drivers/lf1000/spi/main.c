/* 
 * drivers/lf1000/spi/main.c
 *
 * LF1000 SPI/SSP Driver
 *
 * Copyright 2007 LeapFrog Enterprises Inc.
 *
 * Scott Esters <sesters@leapfrog.com>
 * Andrey Yurovsky <andrey@cozybit.com>
 *
 * TODO: integrate with kernel SPI API, see drivers/spi
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/lf1000/spi_ioctl.h>
#include <asm/arch/spi.h>
#include <asm/arch/gpio.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <asm/arch/platform.h>
#include <asm/arch/common.h>

#include "spi_hal.h"
#include "spi.h"

#define SPI_CHANNEL	CONFIG_LF1000_SPI_CHANNEL

struct spi_device spi;

/*******************
 * sysfs Interface *
 *******************/

#ifdef CONFIG_LF1000_SPI_DEBUG
static ssize_t show_registers(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t len = 0;

	len += sprintf(buf+len,"SPI_CONT0  = 0x%04X\n",
		ioread16(spi.mem+SSPSPICONT0));
	len += sprintf(buf+len,"SPI_CONT1  = 0x%04X\n",
		ioread16(spi.mem+SSPSPICONT1));
	len += sprintf(buf+len,"SPI_DATA   = 0x%04X\n",
		ioread16(spi.mem+SSPSPIDATA));
	len += sprintf(buf+len,"SPI_STATE  = 0x%04X\n",
		ioread16(spi.mem+SSPSPISTATE));
	len += sprintf(buf+len,"SPI_CLKENB = 0x%08X\n",
		ioread32(spi.mem+SSPSPICLKENB));
	len += sprintf(buf+len,"SPI_CLKGEN = 0x%04X\n",
		ioread16(spi.mem+SSPSPICLKGEN));

	return len;
}

static DEVICE_ATTR(registers, S_IRUSR|S_IRGRP, show_registers, NULL);

static struct attribute *spi_attributes[] = {
	&dev_attr_registers.attr,
	NULL
};

static struct attribute_group spi_attr_group = {
	.attrs = spi_attributes
};
#endif /* CONFIG_LF1000_SPI_DEBUG */

/**********************
 * Interrupt Handling *
 *********************/

static void lf1000_spi_interrupt(u8 en) {
	u16 reg = ioread16(spi.mem+SSPSPISTATE);

	if(en) {
		BIT_SET(reg, IRQEENB);			// RX, TX completed
		BIT_SET(reg, IRQWENB);			// TX Buffer Empty
		BIT_SET(reg, IRQRENB);			// RX Buffer Full
		BIT_SET(reg, IRQE);			// clear status bits
		BIT_SET(reg, IRQW);
		BIT_SET(reg, IRQR);
	}
	else {
		BIT_CLR(reg, IRQEENB);			// RX, TX completed
		BIT_CLR(reg, IRQWENB);			// TX Buffer Empty
		BIT_CLR(reg, IRQRENB);			// RX Buffer Full
	}
	iowrite16(reg, spi.mem+SSPSPISTATE);
}

static irqreturn_t spi_irq(int irq, void *dev_id)
{
	u16 reg;

	reg = ioread16(spi.mem+SSPSPISTATE);
	
	if (IS_CLR(reg,IRQE)) {	/* interrupt not from SPI */
		return(IRQ_NONE);
	}

	lf1000_spi_interrupt(0);
	wake_up_interruptible(&spi.intqueue);	/* wake any tasks */
	return(IRQ_HANDLED);
}


static int spi_bus_available(void) {
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&spi.spi_spinlock, flags);
	ret = spi.isBusy ? 0 : 1;
	spin_unlock_irqrestore(&spi.spi_spinlock, flags);
	return(ret);
}

/*
 * spi_xfer -- transfer data via SPI
 *
 * Write 16 bits, read 16 bits.
 *
 * Serialize access to SPI bus.  Clear any data in FIFOs, TX data then RX data
 */
int lf1000_spi_xfer(u16 data)
{
	unsigned long flags;

	if(spi.mem == NULL) {
		printk(KERN_ALERT "SPI register pointer is NULL\n");
		return -1;
	}

	/*
	 *  wait for our turn on the SPI bus
	 */

	spin_lock_irqsave(&spi.spi_spinlock, flags);
	while(spi.isBusy) {
		spin_unlock_irqrestore(&spi.spi_spinlock, flags); 

		if (wait_event_interruptible(spi.wqueue,	
					(spi_bus_available())))
			return -ERESTARTSYS;
		spin_lock_irqsave(&spi.spi_spinlock, flags);	
	}

	spi.isBusy = 1;	/* got the bus */
	spin_unlock_irqrestore(&spi.spi_spinlock, flags);

	/* 
	 * Clear TX and RX data from SPI device 
	 */

	if (IS_CLR(ioread16(spi.mem+SSPSPISTATE), WFFEMPTY)) {
		/* wait for tx buffer empty */
		lf1000_spi_interrupt(1);
		if (wait_event_interruptible(spi.intqueue,  
			(IS_SET(ioread16(spi.mem+SSPSPISTATE), WFFEMPTY)))) {
			printk(KERN_ERR "SPI: TX Empty error\n");
			spi.isBusy = 0;
			spin_unlock_irqrestore(&spi.spi_spinlock, flags); 
			return -ERESTARTSYS;
		}
	}

	while (!IS_SET(ioread16(spi.mem+SSPSPISTATE), RFFEMPTY)) {
		ioread16(spi.mem+SSPSPIDATA);
	}

	/* 
	 * Transmit Data 
	 */
	gpio_set_val(SPI_GPIO_PORT, SPI_SSPFRM_PIN, 0);
	lf1000_spi_interrupt(1);
	iowrite16(data, spi.mem+SSPSPIDATA);

	/* 
	 * Wait for Receive Data 
	 */
	if (wait_event_interruptible(spi.intqueue,
		(IS_CLR(ioread16(spi.mem+SSPSPISTATE), RFFEMPTY)))) {
		printk(KERN_ERR "SPI: RX error\n");
		spi.isBusy = 0;
		spin_unlock_irqrestore(&spi.spi_spinlock, flags); 
		return -ERESTARTSYS;
	}
	gpio_set_val(SPI_GPIO_PORT, SPI_SSPFRM_PIN, 1);

	/* 
	 * release SPI bus 
	 */	
	spin_lock_irqsave(&spi.spi_spinlock, flags);
	spi.isBusy = 0;	
	spin_unlock_irqrestore(&spi.spi_spinlock, flags); 

	wake_up_interruptible(&spi.wqueue);
	return(ioread16(spi.mem+SSPSPIDATA));
}
EXPORT_SYMBOL(lf1000_spi_xfer);

/****************************************
 *  module functions and initialization *
 ****************************************/

static int lf1000_spi_probe(struct platform_device *pdev)
{
	int ret = 0;
	int div;
	struct resource *res;
       
	res = platform_get_resource(pdev, IORESOURCE_MEM, SPI_CHANNEL);
	if(!res) {
		printk(KERN_ERR "spi: failed to get resource\n");
		return -ENXIO;
	}

	div = lf1000_CalcDivider(get_pll_freq(SPI_CLK_SRC), SPI_SRC_HZ);
	if(div < 0) {
		printk(KERN_ERR "spi: failed to find a clock divider!\n");
		return -EFAULT;
	}

	if(!request_mem_region(res->start, (res->end - res->start)+1,
				"lf1000-spi")) {
		printk(KERN_ERR "spi: failed to map region\n");
		return -EBUSY;
	}

	spi.mem = ioremap_nocache(res->start, (res->end - res->start)+1);
	if(spi.mem == NULL) {
		printk(KERN_ERR "spi: failed to ioremap\n");
		ret = -ENOMEM;
		goto fail_remap;
	}

	/* set SPI for 1 MHz devices */
	iowrite16(((div-1)<<SPI_CLKDIV)|(SPI_CLK_SRC<<SPI_CLKSRCSEL), 
			spi.mem+SSPSPICLKGEN);
	/* start clock generation */
	iowrite32((1<<SPI_PCLKMODE)|(1<<SPI_CLKGENENB), 
			spi.mem+SSPSPICLKENB);
	/* output only for transmission, enable, 16 bits, divide internal 
	   clock by 20 to get ~1MHz */
	iowrite16((0<<ZENB)|(0<<RXONLY)|(1<<ENB)|((16-1)<<NUMBIT)|(20<<DIVCNT),
		  spi.mem+SSPSPICONT0);
	/* invert SPI clock, Format B, SPI type */
	iowrite16((0<<SCLKPOL)|(1<<SCLKSH)|(1<<TYPE), spi.mem+SSPSPICONT1);
	/* clear status flags */
	iowrite16(0x0000, spi.mem+SSPSPISTATE);

	/* configure SPI IO pins */
	gpio_configure_pin(SPI_GPIO_PORT, SPI_MOSI_PIN, SPI_MOSI_FN, 1, 0, 0);
	gpio_configure_pin(SPI_GPIO_PORT, SPI_MISO_PIN, SPI_MISO_FN, 0, 0, 0);
	gpio_configure_pin(SPI_GPIO_PORT, SPI_SCLK_PIN, SPI_SCLK_FN, 1, 0, 0);
	gpio_configure_pin(SPI_GPIO_PORT, SPI_SSPFRM_PIN, SPI_SSPFRM_FN,1,0,1);

	init_waitqueue_head(&spi.wqueue);
	init_waitqueue_head(&spi.intqueue);
	spi.isBusy = 0;		
	spi.spi_spinlock = SPIN_LOCK_UNLOCKED;

	spi.irq = platform_get_irq(pdev, SPI_CHANNEL);
	if(spi.irq < 0) {
		printk(KERN_ERR "spi: failed to get an IRQ\n");
		ret = spi.irq;
		goto fail_irq;
	}
	ret = request_irq(spi.irq, spi_irq, SA_INTERRUPT|SA_SAMPLE_RANDOM,
			"spi", NULL);
	if(ret) {
		printk(KERN_INFO "spi: requesting IRQ failed\n");
		goto fail_irq;
	}

#ifdef CONFIG_LF1000_SPI_DEBUG
	sysfs_create_group(&pdev->dev.kobj, &spi_attr_group);
#endif

	return 0;

fail_irq:
	iounmap(spi.mem);
fail_remap:
	release_mem_region(res->start, (res->end - res->start) + 1);
	return ret;
}

static int lf1000_spi_remove(struct platform_device *pdev)
{
	struct resource *res  = platform_get_resource(pdev, IORESOURCE_MEM, 0);

#ifdef CONFIG_LF1000_SPI_DEBUG
	sysfs_remove_group(&pdev->dev.kobj, &spi_attr_group);
#endif
	if(spi.irq != -1)
		free_irq(spi.irq, NULL);

	if(spi.mem != NULL)
		iounmap(spi.mem);
	
	release_mem_region(res->start, (res->end - res->start) + 1);
	return 0;
}

static struct platform_driver lf1000_spi_driver = {
	.probe		= lf1000_spi_probe,
	.remove		= lf1000_spi_remove,
	.driver		= {
		.name	= "lf1000-spi",
		.owner	= THIS_MODULE,
	},
};

static void __exit spi_cleanup(void)
{
	platform_driver_unregister(&lf1000_spi_driver);
}

static int __init spi_init(void)
{
	return platform_driver_register(&lf1000_spi_driver);
}

module_init(spi_init);
module_exit(spi_cleanup);
MODULE_AUTHOR("Scott Esters");
MODULE_VERSION("1:1.0");
MODULE_LICENSE("GPL");
