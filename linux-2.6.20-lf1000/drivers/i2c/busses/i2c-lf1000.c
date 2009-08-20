/*
 * drivers/i2c/bus/i2c-lf1000.c
 *
 * Copyright 2007 LeapFrog Enterprises Inc.
 *
 * Andrey Yurovsky <andrey@cozybit.com>
 *
 * I2C bus driver for the LF1000 and MP2530F CPUs.  Based heavily on 
 * i2c-at91.c and i2c-mpc.c
 *
 * TODO: more pin configurations (for the rest of the channels)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/arch/platform.h>
#include <asm/arch/common.h>
#include <asm/arch/i2c.h>
#include <asm/arch/gpio.h>

#define I2C_CHANNEL		CONFIG_I2C_LF1000_CHANNEL
#define LF1000_I2C_TIMEOUT	10 /* (in jiffies) */
#define LF1000_I2C_RATE_HZ	100000

/* FIXME: add channel 0 settings, choose based on I2C_CHANNEL */
#ifdef CPU_LF1000
#define I2C_SCL0_PORT	GPIO_PORT_A
#define I2C_SCL0_PIN	26
#define I2C_SCL0_FN	GPIO_ALT1
#define I2C_SDA0_PORT	GPIO_PORT_A
#define I2C_SDA0_PIN	27
#define I2C_SDA0_FN	GPIO_ALT1
#endif

enum lf1000_i2c_state {
	I2C_SEND_ADDR,
	I2C_SEND_DATA,
	I2C_SEND_DONE,
};

struct lf1000_i2c {
	void __iomem *base;
	int irq;
	int div;

	/* device access */
	wait_queue_head_t wait;
	int ready;

	/* bus access */
	wait_queue_head_t bus_access;
	int busy;
};

static struct lf1000_i2c dev = {
	.base = NULL,
	.irq = -1,
	.ready = 0,
	.div = 16,
	.busy = 0,
};

static irqreturn_t lf1000_i2c_irq(int irq, void *dev_id)
{
	u32 tmp = ioread32(dev.base+IRQ_PEND);

	if(!(tmp & (1<<PEND))) /* sanity check */
		return IRQ_NONE;

	/* clear pending interrupt */
	tmp |= (1<<PEND);
	iowrite32(tmp, dev.base+IRQ_PEND);

	dev.ready = 1;
	wake_up_interruptible(&dev.wait); /* wake up anyone that is waiting */
	return IRQ_HANDLED;
}

static int lf1000_i2c_wait(void)
{
	int ret = wait_event_interruptible_timeout(dev.wait, (dev.ready),
						   LF1000_I2C_TIMEOUT);

	if(unlikely(ret < 0))
		printk(KERN_INFO "i2c: interrupted\n");
	else if(unlikely(!(dev.ready)))
		return -ETIMEDOUT;

	dev.ready = 0;
	return 0;
}

static int i2c_bus_available(void)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&dev.bus_access, flags);
	ret = !(dev.busy);
	spin_unlock_irqrestore(&dev.bus_access, flags);

	return ret;
}

static void start_stop_condition(void)
{
	u32 tmp = ioread32(dev.base+IRQ_PEND);
	tmp |= (1<<OP_HOLD); /* generate a condition */
	iowrite32(tmp, dev.base+IRQ_PEND);
}

static void lf1000_i2c_clock(char en)
{
	u32 tmp = ioread32(dev.base+I2C_CLKENB);

	en ? BIT_SET(tmp, 3) :BIT_CLR(tmp, 3);

	iowrite32(tmp, dev.base+I2C_CLKENB);
}

/* initialize the I2C hardware, see page 17-6 in the MP2530 data book */
static void lf1000_i2c_hwinit(void)
{
	/* clear control registers */
	iowrite32(0, dev.base+ICCR);
	iowrite32(0, dev.base+BURST_CTRL);

	/* Pclk/256/div, enable interrupts */
	iowrite32((1<<CLK_SRC)|((dev.div-1)<<CLK_SCALER)|(1<<IRQ_ENB), 
			dev.base+ICCR);

	iowrite32((1<<CNT_MAX), dev.base+QCNT_MAX);

	iowrite32(0x1010, dev.base+ICSR);

	start_stop_condition(); /* STOP */
}

/* write to a slave device, see page 17-6 in the MP2530 data book */
static int xfer_write(struct i2c_adapter *adap, unsigned char *buf, int length)
{
	u32 tmp;
	int ret;
	enum lf1000_i2c_state state = I2C_SEND_ADDR;

	/* configure for master transmitter mode */
	tmp = (ioread32(dev.base+ICSR) & 0x10F0);
	tmp |= ((1<<IRQ_ENB)|(1<<ST_ENB)|(1<<MASTER_SLV)|(1<<TX_RX)|
			(1<<ST_BUSY)|(1<<TXRX_ENB));
	iowrite32(tmp, dev.base+ICSR);

	start_stop_condition(); /*START*/	

	while(1) {
		switch(state) {
			case I2C_SEND_ADDR:
			ret = lf1000_i2c_wait();
			if(ret != 0)
				goto done_write;
			tmp = ioread32(dev.base+ICSR); /* check for an ACK */
			if(tmp & (1<<ACK_STATUS)) {
				printk(KERN_INFO "i2c: no ACK in xfer_write\n");
				ret = -EFAULT;
				goto done_write;
			}
			state = I2C_SEND_DATA;
			break;

			case I2C_SEND_DATA:
			iowrite32(*buf++, dev.base+IDSR); /* write data */
			start_stop_condition(); /* START */
			ret = lf1000_i2c_wait(); /* wait for IRQ */
			if(ret != 0)
				goto done_write;
			if(--length <= 0)
				state = I2C_SEND_DONE;
			break;

			case I2C_SEND_DONE:
			ret = 0;
			goto done_write;
		}
	}

done_write:
	/* set up to generate STOP condition */
	tmp = (ioread32(dev.base+ICSR) & 0x1F0F);
	tmp |= ((1<<ST_ENB)|(1<<MASTER_SLV)|(1<<TX_RX)|(1<<TXRX_ENB));
	iowrite32(tmp, dev.base+ICSR);

	start_stop_condition(); /* STOP */

	iowrite32(0, dev.base+ICSR); /* turn off I2C controller */
	return 0;
}

/* read from a slave device, see page 17-7 in the MP2530 data book */
static int xfer_read(struct i2c_adapter *adap, unsigned char *buf, int length)
{
	u32 tmp;
	int ret;
	enum lf1000_i2c_state state = I2C_SEND_ADDR;

	/* configure for master receiver mode */
	tmp = (ioread32(dev.base+ICSR) & 0x1F0F);
	tmp |= ((1<<IRQ_ENB)|(1<<ST_ENB)|(1<<MASTER_SLV)|
			(1<<ST_BUSY)|(1<<TXRX_ENB));
	iowrite32(tmp, dev.base+ICSR);

	start_stop_condition(); /*START*/	

	while(1) {
		switch(state) {
			case I2C_SEND_ADDR:
			ret = lf1000_i2c_wait();
			if(ret != 0)
				goto done_read;
			tmp = ioread32(dev.base+ICSR); /* check for an ACK */
			if(tmp & (1<<ACK_STATUS)) {
				printk(KERN_INFO "i2c: no ACK in xfer_read\n");
				ret = -EFAULT;
				goto done_read;
			}
			state = I2C_SEND_DATA;
			break;

			case I2C_SEND_DATA:
			*buf++ = ioread32(dev.base+IDSR); /* get data */
			start_stop_condition(); /* START (request more data) */
			ret = lf1000_i2c_wait(); /* wait for IRQ */
			if(ret != 0)
				goto done_read;

			if(--length <= 0)
				state = I2C_SEND_DONE;
			break;

			case I2C_SEND_DONE:
			ret = 0;
			goto done_read;
		}
	}

done_read:
	/* set up to generate STOP condition */
	tmp = ioread32(dev.base+ICSR);
	tmp &= ~(0x1F0F);
	tmp |= ((1<<ST_ENB)|(1<<MASTER_SLV)|(1<<TXRX_ENB));
	iowrite32(tmp, dev.base+ICSR);

	start_stop_condition(); /* STOP */

	iowrite32(0, dev.base+ICSR); /* turn off I2C controller */
	return ret;
}

/* generic I2C master transfer entrypoint */
static int lf1000_xfer(struct i2c_adapter *adap, struct i2c_msg *pmsg, int num)
{
	int i, ret;
	unsigned long flags;

	/* wait to get access to the bus */
	spin_lock_irqsave(&dev.bus_access, flags);
	while(dev.busy) {
		spin_unlock_irqrestore(&dev.bus_access, flags);
		if(wait_event_interruptible(dev.bus_access, 
					    i2c_bus_available())) {
			dev.busy = 0; 
			spin_unlock_irqrestore(&dev.bus_access, flags);
			return -ERESTARTSYS;
		}
		spin_lock_irqsave(&dev.bus_access, flags);
	}
	dev.busy = 1; /* got the bus */
	spin_unlock_irqrestore(&dev.bus_access, flags);

	lf1000_i2c_clock(1);

	for(i = 0; i < num; i++) {
		lf1000_i2c_hwinit();

		/* set slave device address */
		iowrite32(pmsg->addr | ((pmsg->flags & I2C_M_RD) ? 1 : 0), 
				  dev.base+IDSR);

		if(pmsg->len && pmsg->buf) {
			if(pmsg->flags & I2C_M_RD) 
				ret = xfer_read(adap, pmsg->buf, pmsg->len);
			else
				ret = xfer_write(adap, pmsg->buf, pmsg->len);

			if(ret != 0)
				goto xfer_done;
		}

		pmsg++; /* next message */
	}
	ret = i;

xfer_done:
	lf1000_i2c_clock(0);
	/* realease the bus */
	spin_lock_irqsave(&dev.bus_access, flags);
	dev.busy = 0;
	spin_unlock_irqrestore(&dev.bus_access, flags);
	return ret;
}

/*
 * Return list of supported functionality.
 */
static u32 lf1000_func(struct i2c_adapter *adapter)
{
    return I2C_FUNC_I2C;
}

static struct i2c_algorithm lf1000_algorithm = {
    .master_xfer    = lf1000_xfer,
    .functionality  = lf1000_func,
};

static int lf1000_i2c_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct i2c_adapter *adapter;
	int ret = 0;
	unsigned int pclk_hz;

	res = platform_get_resource(pdev, IORESOURCE_MEM, I2C_CHANNEL);
	if(!res) {
		printk(KERN_ERR "i2c: failed to get resource\n");
		return -ENXIO;
	}

	if(!request_mem_region(res->start, 
			       (res->end - res->start) + 1,
			       "lf1000_i2c")) {
		printk(KERN_ERR "i2c: failed to request_mem_region\n");
		return -EBUSY;
	}

	dev.base = ioremap(res->start, (res->end - res->start) + 1);
	if(dev.base == NULL) {
		printk(KERN_ERR "i2c: failed to ioremap\n");
		ret = -ENOMEM;
		goto fail_remap;
	}

	adapter = kzalloc(sizeof(struct i2c_adapter), GFP_KERNEL);
	if(adapter == NULL) {
		printk(KERN_ERR "i2c: failed to allocate interface\n");
		ret = -ENOMEM;
		goto fail_adapter;
	}

	sprintf(adapter->name, "LF1000");
	adapter->algo = &lf1000_algorithm;
	adapter->class = I2C_CLASS_HWMON;
	adapter->dev.parent = &pdev->dev;

	platform_set_drvdata(pdev, adapter);

	/* set up IRQ handler */
	dev.irq = platform_get_irq(pdev, I2C_CHANNEL);
	if(dev.irq < 0) {
		printk(KERN_ERR "i2c: failed to get an IRQ\n");
		ret = dev.irq;
		goto fail_irq;
	}
	ret = request_irq(dev.irq, lf1000_i2c_irq, 
			SA_INTERRUPT|SA_SAMPLE_RANDOM, "i2c", NULL);
	if(ret) {
		printk(KERN_ERR "i2c: requesting IRQ failed\n" );
		goto fail_irq;
	}

	/* set up I2C IRQ wait queue */
	init_waitqueue_head(&dev.wait);

	/* set up I2C bus access queue */
	init_waitqueue_head(&dev.bus_access);

	pclk_hz = get_pll_freq(PCLK_PLL)/2;
	dev.div = lf1000_CalcDivider(pclk_hz/265, LF1000_I2C_RATE_HZ);
	if(dev.div < 0) {
		printk(KERN_ALERT "i2c: failed to get divider, using 16\n");
		dev.div = 16;
	}
	else if(dev.div > 16) {
		printk(KERN_ALERT "i2c: divider too high, using 16\n");
		dev.div = 16;
	}

	/* set up IO pins */
	#ifdef I2C_SCL0_PORT
	gpio_configure_pin(I2C_SCL0_PORT, I2C_SCL0_PIN, I2C_SCL0_FN, 1, 0, 0);
	#endif
	#ifdef I2C_SDA0_PORT
	gpio_configure_pin(I2C_SDA0_PORT, I2C_SDA0_PIN, I2C_SDA0_FN, 1, 0, 0);
	#endif

	/* initialize I2C hardware */
	lf1000_i2c_hwinit();
	lf1000_i2c_clock(1);

	ret = i2c_add_adapter(adapter);
	if(ret != 0) {
		printk(KERN_ERR "i2c: failed to add adapter\n");
		goto fail_register;
	}

	return 0;

fail_register:
	lf1000_i2c_clock(0);
	platform_set_drvdata(pdev, NULL);
	kfree(adapter);
fail_irq:
	free_irq(dev.irq, NULL);
	dev.irq = -1;
fail_adapter:
	iounmap(dev.base);
fail_remap:
	release_mem_region(res->start, (res->end - res->start) + 1);

	return ret;
}

static int lf1000_i2c_remove(struct platform_device *pdev)
{
	struct i2c_adapter *adapter = platform_get_drvdata(pdev);
	struct resource *res;
	int ret;

	if(adapter != NULL) {
		ret = i2c_del_adapter(adapter);
		platform_set_drvdata(pdev, NULL);

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		iounmap(dev.base);
		release_mem_region(res->start, (res->end - res->start) + 1);
		return ret;
	}

	lf1000_i2c_clock(0);
	return 0;
}

#ifdef CONFIG_PM
static int lf1000_i2c_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	lf1000_i2c_clock(0);
	return 0;
}

static int lf1000_i2c_resume(struct platform_device *pdev)
{
	lf1000_i2c_clock(1);
	return 0;
}
#else
#define lf1000_i2c_suspend	NULL
#define lf1000_i2c_resume	NULL
#endif

static struct platform_driver lf1000_i2c_driver = {
	.probe      = lf1000_i2c_probe,
	.remove     = lf1000_i2c_remove,
	.suspend    = lf1000_i2c_suspend,
	.resume     = lf1000_i2c_resume,
	.driver     = {
		.name   = "lf1000-i2c",
		.owner  = THIS_MODULE,
	},
};

static int __init lf1000_i2c_init(void)
{
	return platform_driver_register(&lf1000_i2c_driver);
}

static void __exit lf1000_i2c_exit(void)
{
	return platform_driver_unregister(&lf1000_i2c_driver);
}

module_init(lf1000_i2c_init);
module_exit(lf1000_i2c_exit);

MODULE_AUTHOR("Andrey Yurovsky");
MODULE_DESCRIPTION("I2C driver for LF1000");
MODULE_LICENSE("GPL");
