/* 
 * drivers/lf1000/pwm/pwm.c
 * 
 * LF1000 Pulse Width Modulator (PWM) Driver
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
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <asm/arch/platform.h>
#include <asm/arch/common.h>
#include <asm/arch/pwm.h>
#include <asm/arch/gpio.h>
#include "pwm_hal.h"
#include "pwm_config.h"

static struct pwm_device {
	void *mem;
	int clock_rate;
} pwm;

/*********************
 * PWM API Functions *
 *********************/

int pwm_ConfigurePin(enum pwm_chan channel)
{
	enum gpio_port port;
	enum gpio_pin pin;

	switch(channel) {
#ifdef CPU_LF1000
		case PWM_CHAN0:
		port = GPIO_PORT_A;
		pin = 30;
		break;
		case PWM_CHAN1:
		port = GPIO_PORT_A;
		pin = 31;
		break;
		case PWM_CHAN2:
		port = GPIO_PORT_C;
		pin = 7;
		break;
#else /* CPU_MF2530F */
		case PWM_CHAN0:
		port = GPIO_PORT_B;
		pin = 28;
		break;
		case PWM_CHAN1:
		port = GPIO_PORT_B;
		pin = 29;
		break;
		case PWM_CHAN2:
		port = GPIO_PORT_B;
		pin = 30;
		break;
		case PWM_CHAN3:
		port = GPIO_PORT_B;
		pin = 31;
		break;
#endif
		default:
		return -1;
		break;
	}
	gpio_set_fn(port, pin, GPIO_ALT1);
	gpio_set_pu(port, pin, 0);
	gpio_set_out_en(port, pin, 1);
	return 0;
}
EXPORT_SYMBOL(pwm_ConfigurePin);

int pwm_SetPrescale(enum pwm_chan channel, u32 prescale)
{
	u16 tmp;
	u8 shift = 0;
	void *reg;

	if(channel >= PWM_CHAN_INVALID || prescale >= 128)
		return -EINVAL;

	if(channel <= 1) { /* 0/1 */
		reg = pwm.mem+PWM01PRES;
	}
	else { /* 2/3 */
		reg = pwm.mem+PWM23PRES;
		channel /= 2; /* treat 2 like 0, 3 like 1 */
	}

	shift = channel == 0 ? PWM0PRESCALE : PWM1PRESCALE;

	tmp = ioread16(reg);
	tmp &= ~(0x3F<<shift); /* clear prescaler bits */
	tmp |=  (prescale<<shift); /* set prescaler */
	iowrite16(tmp, reg);
	return 0;
}
EXPORT_SYMBOL(pwm_SetPrescale);

int pwm_SetPolarity(enum pwm_chan channel, u8 polarity)
{
	void *reg;
	u16 tmp;
	u8 shift = 0;

	if( channel >= PWM_CHAN_INVALID || polarity >= POL_INVALID )
		return -EINVAL;

	if( channel <= 1 ) { /* 0/1 */
		reg = pwm.mem+PWM01PRES;
	}
	else { /* 2/3 */
		reg = pwm.mem+PWM23PRES;
		channel /= 2; /* treat 2 like 0, 3 like 1 */
	}

	shift = channel == 0 ? PWM0POL : PWM1POL;

	tmp = ioread16(reg);
	if(polarity == POL_INV)
		tmp &= ~(1<<shift);
	else
		tmp |= (1<<shift);
	return 0;
}
EXPORT_SYMBOL(pwm_SetPolarity);

int pwm_SetPeriod(enum pwm_chan channel, u32 period)
{
	void *reg = NULL;

	if(channel >= PWM_CHAN_INVALID || period >= 1024)
		return -EINVAL;

	switch(channel) {
		case PWM_CHAN0:
		reg = pwm.mem+PWM0PERIOD;
		break;
		case PWM_CHAN1:
		reg = pwm.mem+PWM1PERIOD;
		break;
		case PWM_CHAN2:
		reg = pwm.mem+PWM2PERIOD;
		break;
#ifdef CPU_MF2530F
		case PWM_CHAN3:
		reg = pwm.mem+PWM3PERIOD;
		break;
#endif
		default:
		return -1;
		break;
	}

	if(reg == NULL)
		return -EINVAL;

	iowrite16(period,reg);
	return 0;

}
EXPORT_SYMBOL(pwm_SetPeriod);

int pwm_SetDutyCycle(enum pwm_chan channel, u32 duty)
{
	void *reg = NULL;

	if(channel >= PWM_CHAN_INVALID || duty >= 1024)
		return -EINVAL;

	switch(channel) {
		case PWM_CHAN0:
		reg = pwm.mem+PWM0DUTY;
		break;
		case PWM_CHAN1:
		reg = pwm.mem+PWM1DUTY;
		break;
		case PWM_CHAN2:
		reg = pwm.mem+PWM2DUTY;
		break;
#ifdef CPU_MF2530F
		case PWM_CHAN3:
		reg = pwm.mem+PWM3DUTY;
		break;
#endif
		default:
		return -1;
		break;
	}

	if(reg == NULL)
		return -EINVAL;

	iowrite16(duty,reg);
	return 0;
}
EXPORT_SYMBOL(pwm_SetDutyCycle);

int pwm_SetClock(u8 source, u8 div, u8 mode, u8 enable)
{
	u32 tmp;

	if(source >= PWM_CLK_INVALID || div > 63)
		return -EINVAL;

	iowrite16((u16)((div<<PWMCLKDIV)|(source<<PWMCLKSRCSEL)),
		  pwm.mem+PWMCLKGEN);

	tmp = ioread32(pwm.mem+PWMCLKENB);
	mode ? BIT_SET(tmp, PWMPCLKMODE) : BIT_CLR(tmp, PWMPCLKMODE);
	enable ? BIT_SET(tmp, PWMCLKGENENB) : BIT_CLR(tmp, PWMCLKGENENB);
	iowrite32(tmp,pwm.mem+PWMCLKENB);
	return 0;
}
EXPORT_SYMBOL(pwm_SetClock);

int pwm_GetClockRate(void)
{
	return pwm.clock_rate;
}
EXPORT_SYMBOL(pwm_GetClockRate);

/*******************
 * sysfs interface *
 *******************/

static ssize_t get_rate(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", pwm_GetClockRate());
}
static DEVICE_ATTR(rate, S_IRUSR|S_IRGRP, get_rate, NULL);

static ssize_t set_period(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int channel;
	u32 period;

	if(sscanf(buf, "%d,%d", &channel, &period) != 2)
		return -EINVAL;
	
	if(pwm_SetPeriod((u8)channel, period) != 0)
		return -EINVAL;

	return count;
}
static DEVICE_ATTR(period, S_IWUSR|S_IWGRP, NULL, set_period);

static ssize_t set_prescale(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int channel;
	u32 prescale;

	if(sscanf(buf, "%d,%d", &channel, &prescale) != 2)
		return -EINVAL;

	if(pwm_SetPrescale((u8)channel, prescale) != 0)
		return -EINVAL;

	return count;
}
static DEVICE_ATTR(prescale, S_IWUSR|S_IWGRP, NULL, set_prescale);

static ssize_t set_duty(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int channel;
	u32 duty;

	if(sscanf(buf, "%d,%d", &channel, &duty) != 2)
		return -EINVAL;

	if(pwm_SetDutyCycle(channel, duty) != 0)
		return -EINVAL;

	return count;
}
static DEVICE_ATTR(duty, S_IWUSR|S_IWGRP, NULL, set_duty);

static struct attribute *pwm_attributes[] = {
	&dev_attr_period.attr,
	&dev_attr_prescale.attr,
	&dev_attr_duty.attr,
	&dev_attr_rate.attr,
	NULL
};

static struct attribute_group pwm_attr_group = {
	.attrs = pwm_attributes
};

/****************
 * module stuff *
 ****************/

static int lf1000_pwm_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;
	int div;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res) {
		printk(KERN_ERR "pwm: failed to get resource\n");
		return -ENXIO;
	}

	if(!request_mem_region(res->start, (res->end - res->start)+1,
				"lf1000-pwm")) {
		printk(KERN_ERR "pwm: failed to map memory region.");
		return -EBUSY;
	}

	div = lf1000_CalcDivider(get_pll_freq(PWM_CLK_SRC), PWM_CLOCK_HZ);
	if(div < 0) {
		printk(KERN_ERR "pwm: failed to get a clock divider\n");
		return -EFAULT;
	}

	pwm.mem = ioremap_nocache(res->start, (res->end - res->start)+1);
	if(pwm.mem == NULL) {
		printk(KERN_ERR "pwm: failed to ioremap\n");
		ret = -ENOMEM;
		goto fail_remap;
	}

	ret = pwm_SetClock(PWM_CLK_SRC, div, 1, 1);
	if(ret < 0) {
		printk(KERN_ALERT "pwm: divider too high, using 63\n");
		pwm_SetClock(PWM_CLK_SRC, 63, 1, 1);
		div = 63;
	}

	pwm.clock_rate = get_pll_freq(PWM_CLK_SRC)/(div+1);

	sysfs_create_group(&pdev->dev.kobj, &pwm_attr_group);
	return 0;

fail_remap:
	release_mem_region(res->start, (res->end - res->start) + 1);
	return ret;
}

static int lf1000_pwm_remove(struct platform_device *pdev)
{
	struct resource *res  = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	sysfs_remove_group(&pdev->dev.kobj, &pwm_attr_group);

	if(pwm.mem != NULL)
		iounmap(pwm.mem);
	release_mem_region(res->start, (res->end - res->start) + 1);
	return 0;
}

static struct platform_driver lf1000_pwm_driver = {
	.probe		= lf1000_pwm_probe,
	.remove		= lf1000_pwm_remove,
	.driver		= {
		.name	= "lf1000-pwm",
		.owner	= THIS_MODULE,
	},
};

static void __exit pwm_cleanup(void)
{
	platform_driver_unregister(&lf1000_pwm_driver);
}

static int __init pwm_init(void)
{
	return platform_driver_register(&lf1000_pwm_driver);
}

module_init(pwm_init);
module_exit(pwm_cleanup);

MODULE_AUTHOR("Andrey Yurovsky");
MODULE_LICENSE("GPL");
MODULE_VERSION("1:2.0");
