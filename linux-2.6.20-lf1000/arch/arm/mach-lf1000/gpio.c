/* 
 * arch/arm/mach-lf1000/gpio.c
 *
 * Copyright 2007 LeapFrog Enterprises Inc.
 *
 * LF1000 General-Purpose IO (GPIO) API, see also 
 * include/asm/arch-lf1000/gpio.h
 *
 * Andrey Yurovsky <andrey@cozybit.com>
 * Brian Cavagnolo <brian@cozybit.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include "gpio_priv.h"
#include <asm/arch/gpio.h>
#include <asm/arch/common.h>
#include <asm/arch/gpio_hal.h>

#define IS_GPIO_PORT(p)	(p >= GPIO_PORT_A && p <= GPIO_PORT_ALV)

extern struct gpio_device gpio;

spinlock_t gpio_handlers_lock = SPIN_LOCK_UNLOCKED;
struct gpio_handler gpio_handlers[GPIO_PORT_ALV+1][GPIO_PIN31+1];

/* Set the pin function. */
int gpio_set_fn(enum gpio_port port, enum gpio_pin pin,
		 enum gpio_function f)
{
	void *reg;
	unsigned long tmp;

	if(!gpio.mem || !IS_GPIO_PORT(port) || pin >= 32)
		return -EINVAL;
   	reg = gpio.mem + GPIOAALTFN0 + port*0x40;

	if(pin >= 16) { /* use GPIOnALTFN1 */
		reg += 4;
		pin -= 16;
	}

	pin *= 2; /* setting two bits per pin */
	tmp = ioread32(reg);
	tmp &= ~(3<<pin);
	tmp |= (f<<pin);
	iowrite32(tmp, reg);

	return 0;
}

/* Get the pin function */
int gpio_get_fn(enum gpio_port port, enum gpio_pin pin)
{
	void *reg;

	if(!gpio.mem || !IS_GPIO_PORT(port) || pin >= 32)
		return -EINVAL;
	reg = gpio.mem + GPIOAALTFN0 + port*0x40;

	if(pin >= 16) { /* use GPIOnALTFN1 */
		reg +=4;
		pin -= 16;
	}

	/* getting two bits per pin */
	return ((ioread32(reg) >> (pin*2)) & 0x3);
}

/* set or clear output enable.  Clearing output enable means this pin is an
 * input.
 */
int gpio_set_out_en(enum gpio_port port, enum gpio_pin pin, unsigned char en)
{
	void *reg;
	unsigned long tmp;
	if(port == GPIO_PORT_ALV && gpio.amem != NULL) {
#ifdef CPU_MF2530F
		/* only GPIO Alive 0 and 3-6 may be used as outputs */
		if(!(pin == 0 || (pin >= 3 && pin <= 6)))
			return -EINVAL;
		reg = gpio.amem + GPIOALVOUTENB;
#else /* CPU_LF1000 */
		if(!en)
			printk(KERN_WARNING
			       "gpio: LF1000 ALIVE pins are outputs only!\n");
		return -EINVAL;
#endif
	}
	else if(gpio.mem != NULL && IS_GPIO_PORT(port))
		reg = gpio.mem + GPIOAOUTENB + port*0x40;
	else
		return -EINVAL;
	tmp = ioread32(reg);

	en ? BIT_SET(tmp, pin) : BIT_CLR(tmp, pin);
	iowrite32(tmp, reg);

	return 0;
}

/* set or clear the pull-up enable */
void gpio_set_pu(enum gpio_port port, enum gpio_pin pin, unsigned char en)
{
	void *reg;
	unsigned long tmp;

	if(pin >= 32)
		return;

	if(port == GPIO_PORT_ALV && gpio.amem != NULL) {
#ifdef CPU_MF2530F
		reg = gpio.amem + GPIOALVPUENB;
#else /* CPU_LF1000 */
		return;
#endif
	}
	else if(gpio.mem != NULL && IS_GPIO_PORT(port))
		reg = gpio.mem + GPIOAPUENB + port*0x40;
	else
		return;
   	tmp = ioread32(reg);

	en ? BIT_SET(tmp, pin) : BIT_CLR(tmp, pin);
	iowrite32(tmp, reg);
}

#ifdef CPU_LF1000
/* set the output value of the pin */
int gpio_set_val(enum gpio_port port, enum gpio_pin pin, unsigned char en)
{
	void *reg, *set_reg, *clr_reg;
	unsigned long tmp;

	if(pin >= 32)
		return -EINVAL;

	if(port == GPIO_PORT_ALV) {
		if(gpio.amem == NULL)
			return -EINVAL;

		/* On the LF1000, we're operating an SR flip-flop, not a GPIO
		 * pin.
		 */

		/* enable writing to GPIO ALIVE registers */
		iowrite32(1 << NPOWERGATING, gpio.amem + ALIVEPWRGATEREG);
		
		if (en) { /* set R/S bit */
			set_reg = gpio.amem + ALIVEGPIOSETREG;
			clr_reg = gpio.amem + ALIVEGPIORSTREG;
		} else {  /* clear R/S bit */
			clr_reg = gpio.amem + ALIVEGPIOSETREG;
			set_reg = gpio.amem + ALIVEGPIORSTREG;
		}

		tmp = ioread32(clr_reg); /* clear bit first */
		BIT_CLR(tmp,pin);
		iowrite32(tmp,clr_reg);
		tmp = ioread32(set_reg); /* then set bit */
		BIT_SET(tmp,pin);
		iowrite32(tmp,set_reg);
	
		/* disable writing to GPIO ALIVE registers */
		iowrite32(0 << NPOWERGATING, gpio.amem + ALIVEPWRGATEREG);
		return 0;
	}
	else if(gpio.mem != NULL && IS_GPIO_PORT(port))
		reg = gpio.mem + GPIOAOUT + port*0x40;
	else
		return - EINVAL;
	
	tmp = ioread32(reg);
	en ? BIT_SET(tmp, pin) : BIT_CLR(tmp, pin);
	iowrite32(tmp, reg);

	return 0;
}
#else /* CPU_MF2530F */
/* set the output value of the pin */
int gpio_set_val(enum gpio_port port, enum gpio_pin pin, unsigned char en)
{
	void *reg;
	unsigned long tmp;

	if(pin >= 32)
		return -EINVAL;

	if(port == GPIO_PORT_ALV && gpio.amem != NULL) {
		/* only GPIO Alive 0 and 3-6 can be used as outputs */
		if(!(pin == 0 || (pin >= 3 && pin <= 6)))
			return -EINVAL;
		reg = gpio.amem + GPIOALVOUT;
	}
	else if(gpio.mem != NULL && IS_GPIO_PORT(port))
		reg = gpio.mem + GPIOAOUT + port*0x40;
	else
		return -EINVAL;
	
	tmp = ioread32(reg);
	en ? BIT_SET(tmp, pin) : BIT_CLR(tmp, pin);
	iowrite32(tmp, reg);

	return 0;
}
#endif

/* get the input value of the pin */
int gpio_get_val(enum gpio_port port, enum gpio_pin pin)
{
	void *reg;
	unsigned long tmp;

	if(pin >= 32)
		return -EINVAL;

	if(port == GPIO_PORT_ALV && gpio.amem != NULL) {
#ifdef CPU_MF2530F
		reg = gpio.amem + GPIOALVPAD;
#else /* CPU_LF1000 */
		/* LF1000 ALIVE pins are only outputs, but we can still read
		 * them.
		 */
		reg = gpio.amem + ALIVEGPIOREADREG;
#endif
	}
	else if(gpio.mem != NULL && IS_GPIO_PORT(port))
		reg = gpio.mem + GPIOAPAD + port*0x40;
	else
		return -EINVAL;
   	tmp = ioread32(reg);

	return (int)((ioread32(reg) >> pin) & 0x1);
}

/* request an interrupt handler for a given pin. Returns -EBUSY if that pin
 * already has a handler.
 */
int gpio_request_irq(enum gpio_port port, enum gpio_pin pin,
		     gpio_irq_handler_t handler, void *priv)
{
	unsigned long flags;
	struct gpio_handler *gh;
	int ret;

	if(pin >= 32)
		return -EBUSY;
#ifdef CPU_LF1000
	if(!IS_GPIO_PORT(port))
		return - EBUSY;
#endif

	spin_lock_irqsave(&gpio_handlers_lock, flags);
	gh = &gpio_handlers[port][pin];
	if(gh->handler.handler != NULL)
		ret = -EBUSY;
	else {
		gh->handler.gpio_handler = handler;
		gh->priv = priv;
		gh->mode_gpio = 1;
		ret = 0;
	}
	spin_unlock_irqrestore(&gpio_handlers_lock, flags);
	return ret;
}

int gpio_request_normal_irq(enum gpio_port port, enum gpio_pin pin,
		irq_handler_t handler, void *priv)
{
	unsigned long flags;
	struct gpio_handler *gh;
	int ret;

	if(pin >= 32)
		return -EBUSY;
#ifdef CPU_LF1000
	if(!IS_GPIO_PORT(port))
		return -EBUSY;
#endif

	spin_lock_irqsave(&gpio_handlers_lock, flags);
	gh = &gpio_handlers[port][pin];
	if(gh->handler.handler != NULL)
		ret = -EBUSY;
	else {
		gh->handler.normal_handler = handler;
		gh->priv = priv;
		gh->mode_gpio = 0;
		ret = 0;
	}
	spin_unlock_irqrestore(&gpio_handlers_lock, flags);
	return ret;
}

/* free the irq requested using gpio_request_irq.  To prevent accidental
 * freeing of somebody else's gpio irq, the handler must match the one that was
 * passed to gpio_request_irq.
 */
void gpio_free_irq(enum gpio_port port, enum gpio_pin pin,
		   gpio_irq_handler_t handler)
{
	unsigned long flags;
	struct gpio_handler *gh;

	if(pin >= 32)
		return;
#ifdef CPU_LF1000
	if(!IS_GPIO_PORT(port))
		return;
#endif

	spin_lock_irqsave(&gpio_handlers_lock, flags);
	gh = &gpio_handlers[port][pin];
	if(gh->handler.handler == handler) {
		gh->handler.handler = NULL;
		gh->priv = NULL;
	}
	spin_unlock_irqrestore(&gpio_handlers_lock, flags);
}

/* get the interrupt mode for a given pin */
enum gpio_interrupt_mode gpio_get_int_mode(enum gpio_port port,
					   enum gpio_pin pin)
{
	void *reg;
	if(port == GPIO_PORT_ALV && gpio.amem != NULL) {
#ifdef CPU_MF2530F
		/* GPIOALV only has 7 pins */
		if(pin >= 7)
			return 0;
		reg = gpio.amem + GPIOALVDETMODE0;
#else /* CPU_LF1000 */
		printk(KERN_ALERT "gpio: ALV in gpio_get_int_mode\n");
		return 0;
#endif
	} else if(gpio.mem != NULL && IS_GPIO_PORT(port) && pin < 32) {
		reg = gpio.mem + port*0x40;
		if(pin < 16) {
			reg += GPIOADETMODE0;
		} else {
			reg += GPIOADETMODE1;
			pin -= 16;
		}
	}
	else
		return 0; /*XXX*/
	return (enum gpio_interrupt_mode)((ioread32(reg) >> (pin<<1)) & 0x3);
}

/* set the interrupt mode for a given pin */
void gpio_set_int_mode(enum gpio_port port, enum gpio_pin pin,
		       enum gpio_interrupt_mode mode)
{
	void *reg;
	unsigned long tmp;

	if(port == GPIO_PORT_ALV && gpio.amem != NULL) {
#ifdef CPU_MF2530F
		/* GPIOALV only has 7 pins */
		if(pin >= 7)
			return;
		reg = gpio.amem + GPIOALVDETMODE0;
#else /* CPU_LF1000 */
		printk(KERN_ALERT "gpio: ALV in gpio_set_int_mode\n");
		return;
#endif
	}
	else if(gpio.mem != NULL && IS_GPIO_PORT(port) && pin < 32) {
		reg = gpio.mem + port*0x40;
		if(pin < 16) {
			reg += GPIOADETMODE0;
		} else {
			reg += GPIOADETMODE1;
			pin -= 16;
		}
	}
	else
		return;

	tmp = ioread32(reg);
	tmp &= ~(0x3 << (pin<<1));
	tmp |= (mode << (pin<<1));
	iowrite32(tmp, reg);
}

/* toggle the interrupt mode for a pin.  If the mode is currently
 * IMODE_RISING_EDGE it becmoes IMODE_FALLING_EDGE and vice versa.  If the mode
 * is IMODE_HIGH_LEVEL it becomes IMODE_LOW_LEVEL
 */
void gpio_toggle_int_mode(enum gpio_port port, enum gpio_pin pin)
{
	void *reg;
	unsigned long tmp;

	if(port == GPIO_PORT_ALV && gpio.amem != NULL) {
#ifdef CPU_MF2530F
		/* GPIOALV only has 7 pins */
		if(pin >= 7)
			return;
		reg = gpio.amem + GPIOALVDETMODE0;
#else /* CPU_LF1000 */
		printk(KERN_ALERT "gpio: ALV in gpio_toggle_int_mode\n");
		return;
#endif
	}
	else if(gpio.mem != NULL && IS_GPIO_PORT(port) && pin < 32) {
		reg = gpio.mem + port*0x40;
		if(pin < 16) {
			reg += GPIOADETMODE0;
		} else {
			reg += GPIOADETMODE1;
			pin -= 16;
		}
	}
	else
		return;

	tmp = ioread32(reg);
	tmp ^= (0x1 << (pin<<1));
	iowrite32(tmp, reg);
}

/* enable or disable interrupt for a given pin */
void gpio_set_int(enum gpio_port port, enum gpio_pin pin, unsigned char en)
{
	void *reg;
	unsigned long tmp;

	if(port == GPIO_PORT_ALV && gpio.amem != NULL) {
#ifdef CPU_MF2530F
		reg = gpio.amem + GPIOALVINTENB;
#else /* CPU_LF1000 */
		printk(KERN_ALERT "gpio: ALV in gpio_set_int\n");
		return;
#endif
	}
	else if(gpio.mem != NULL && IS_GPIO_PORT(port) && pin < 32) {
		reg = gpio.mem + GPIOAINTENB + port*0x40;
	}
	else {
		printk(KERN_WARNING "gpio: gpio_set_int when uninitialized\n");
		return;
	}

	tmp = ioread32(reg);
	en ? BIT_SET(tmp, pin) : BIT_CLR(tmp, pin);
	iowrite32(tmp, reg);

}

/* get the interrupt enable bit for a given pin */
unsigned char gpio_get_int(enum gpio_port port, enum gpio_pin pin)
{
	void *reg;
	if(port == GPIO_PORT_ALV && gpio.amem != NULL) {
#ifdef CPU_MF2530F
		reg = gpio.amem + GPIOALVINTENB;
#else /* CPU_LF1000 */
		printk(KERN_ALERT "gpio: ALV in gpio_get_int\n");
		return 0;
#endif
	}
	else if(gpio.mem != NULL && IS_GPIO_PORT(port) && pin < 32)
		reg = gpio.mem + GPIOAINTENB + port*0x40;
	else
		return 0; /*XXX*/

	return (unsigned char)((ioread32(reg) >> pin) & 0x1);
}

/* get interrupt enable bits for all 32 pins in a given port */
unsigned long gpio_get_int32(enum gpio_port port)
{
	void *reg;

	if(port == GPIO_PORT_ALV && gpio.amem != NULL) {
#ifdef CPU_MF2530F
		reg = gpio.amem + GPIOALVINTENB;
#else /* CPU_LF1000 */
		printk(KERN_ALERT "gpio: ALV in gpio_get_int32\n");
		return 0;
#endif
	}
	else if(gpio.mem != NULL && IS_GPIO_PORT(port))
		reg = gpio.mem + GPIOAINTENB + port*0x40;
	else
		return 0; /*XXX*/

	return ioread32(reg);
}

/* set the interrupt enable bits for all 32 pins in a given port.  Use this
 * function in conjunction with gpio_get_int32 to enable or disable
 * interrupts on many pins at a time.
 */
void gpio_set_int32(enum gpio_port port, unsigned long en)
{
	void *reg;

	if(port == GPIO_PORT_ALV && gpio.amem != NULL) {
#ifdef CPU_MF2530F
		reg = gpio.amem + GPIOALVINTENB;
#else /* CPU_LF1000 */
		printk(KERN_ALERT "gpio: ALV in gpio_set_int32\n");
		return;
#endif
	}
	else if(gpio.mem != NULL && IS_GPIO_PORT(port))
		reg = gpio.mem + GPIOAINTENB + port*0x40;
	else
		return;

	iowrite32(en, reg);
}

/* clear the interrupt pending bit for a given pin */
void gpio_clear_pend(enum gpio_port port, enum gpio_pin pin)
{
	void *reg;

	if(port == GPIO_PORT_ALV && gpio.amem != NULL) {
#ifdef CPU_MF2530F
		reg = gpio.amem + GPIOALVDEP;
#else /* CPU_LF1000 */
		printk(KERN_ALERT "gpio: ALV in gpio_clear_pend\n");
		return;
#endif
	}
	else if(gpio.mem != NULL && IS_GPIO_PORT(port) && pin < 32)
		reg = gpio.mem + GPIOADET + port*0x40;
	else
		return;
	
	iowrite32(1<<pin, reg);
}

/* get the interrupt pending bit for a given pin */
unsigned char gpio_get_pend(enum gpio_port port, enum gpio_pin pin)
{
	void *reg;
	if(port == GPIO_PORT_ALV && gpio.amem != NULL) {
#ifdef CPU_MF2530F
		reg = gpio.amem + GPIOALVPEND;
#else /* CPU_LF1000 */
		printk(KERN_ALERT "gpio: ALV in gpio_get_pend\n");
		return 0;
#endif
	}
	else if(gpio.mem != NULL && IS_GPIO_PORT(port) && pin < 32)
		reg = gpio.mem + GPIOADET + port*0x40;
	else
		return 0; /*XXX*/

	return (unsigned char)((ioread32(reg) >> pin) & 0x1);
}

/* get the interrupt pending bits for all pins in a given port */
unsigned long gpio_get_pend32(enum gpio_port port)
{
	void *reg;

	if(port == GPIO_PORT_ALV && gpio.amem != NULL) {
#ifdef CPU_MF2530F
		reg = gpio.amem + GPIOALVPEND;
#else /* CPU_LF1000 */
		printk(KERN_ALERT "gpio: ALV in gpio_clear_pend32\n");
		return 0;
#endif
	}
	else if(gpio.mem != NULL && IS_GPIO_PORT(port))
		reg = gpio.mem + GPIOADET + port*0x40;
	else
		return 0; /*XXX*/

	return ioread32(reg);
}

/* clear the interrupt pending bits for all pins in a given port.  Use this
 * function in conjunction with gpio_get_pend32 to clear all pending interrupts
 * at once.
 */
void gpio_clear_pend32(enum gpio_port port, unsigned long flag)
{
	void *reg;

	if(port == GPIO_PORT_ALV && gpio.amem != NULL) {
#ifdef CPU_MF2530F
		reg = gpio.amem + GPIOALVDEP;
#else /* CPU_LF1000 */
		printk(KERN_ALERT "gpio: ALV in gpio_clear_pend32\n");
		return;
#endif
	}
	else if(gpio.mem != NULL && IS_GPIO_PORT(port))
		reg = gpio.mem + GPIOADET + port*0x40;
	else
		return;

	iowrite32(flag, reg);
}


#ifdef CPU_LF1000
/* get gpio pin drive current setting */
unsigned long gpio_get_cur(enum gpio_port port, enum gpio_pin pin)
{
	void *reg;

	if((port == GPIO_PORT_ALV) ||
	    ((port == GPIO_PORT_C) && (pin > GPIO_PIN19))) {
		/* We can't set current for GPIO ALIVE block or for GPIOC pins
		 * above 19 
		 */
		return 0;
	} else if(gpio.mem_cur != NULL && IS_GPIO_PORT(port) && pin <= 32) {
		reg = gpio.mem_cur + port*8;
		if(pin < 16) {
			reg += GPIOAPADSTRENGTH_L;
		} else {
			reg += GPIOAPADSTRENGTH_H;
			pin -= 16;
		}
	}
	else
		return 0; /*XXX*/
	return ((ioread32(reg) >> (pin<<1)) & 0x3);
}

/* set the drive current for the gpio pin */
void gpio_set_cur(enum gpio_port port, enum gpio_pin pin, enum gpio_current cur)
{
	void *reg;
	unsigned long tmp;

	if( (port == GPIO_PORT_ALV) ||
	    ((port == GPIO_PORT_C) && (pin > GPIO_PIN19)) ) {
		/* We can't set current for GPIO ALIVE block or for GPIOC pins
		 * above 19 
		 */
		return;
	} else if(gpio.mem_cur != NULL && IS_GPIO_PORT(port) && pin < 32) {
		reg = gpio.mem_cur + port*8;
		if(pin < 16) {
			reg += GPIOAPADSTRENGTH_L;
		} else {
			reg += GPIOAPADSTRENGTH_H;
			pin -= 16;
		}
	}
	else
		return;

	tmp = ioread32(reg);
	tmp &= ~(0x3 << (pin<<1));
	tmp |= (cur << (pin<<1));
	iowrite32(tmp, reg);

}

/* gpio_get_scratch() -- get ALIVE scratch register value */
unsigned long gpio_get_scratch(void)
{
	return(ioread32(gpio.amem + ALIVESCRATCHREADREG));
}

/* gpio_set_scratch() -- set ALIVE scratch register value */
void gpio_set_scratch(unsigned long value)
{
	unsigned int reg32 = 0;

	if(gpio.amem == NULL)
		return;

	/* enable writing to GPIO ALIVE registers */
	reg32 = ioread32(gpio.amem + ALIVEPWRGATEREG);
	BIT_SET(reg32, NPOWERGATING);	
	iowrite32(reg32, gpio.amem + ALIVEPWRGATEREG);

	iowrite32(0, gpio.amem + ALIVESCRATCHRSTREG);
	iowrite32(0, gpio.amem + ALIVESCRATCHSETREG);

	iowrite32(~value, gpio.amem + ALIVESCRATCHRSTREG);
	iowrite32(value, gpio.amem + ALIVESCRATCHSETREG);

	iowrite32(0, gpio.amem + ALIVESCRATCHRSTREG);	
	iowrite32(0, gpio.amem + ALIVESCRATCHSETREG);
	
	/* disable writing to GPIO ALIVE registers */
	reg32 = ioread32(gpio.amem + ALIVEPWRGATEREG);
	BIT_CLR(reg32, NPOWERGATING);	
	iowrite32(reg32, gpio.amem + ALIVEPWRGATEREG);
}

/* gpio_get_scratch_power() -- get power bits of register */
unsigned long gpio_get_scratch_power(void)
{
	return((gpio_get_scratch() & SCRATCH_POWER_MASK) >> SCRATCH_POWER_POS);
}

/* gpio_set_scratch_power() -- set power bits of register */
void gpio_set_scratch_power(unsigned long value)
{
	unsigned long scratch = gpio_get_scratch();

	scratch = (scratch & ~SCRATCH_POWER_MASK) |
		  ((value << SCRATCH_POWER_POS) & SCRATCH_POWER_MASK);
	gpio_set_scratch(scratch);
}
#endif

void gpio_configure_pin(enum gpio_port port, enum gpio_pin pin, 
		enum gpio_function f, unsigned char out_en, 
		unsigned char pu_en, unsigned char val)
{
	gpio_set_fn(port, pin, f);
	gpio_set_out_en(port, pin, out_en);
	gpio_set_pu(port, pin, pu_en);
	gpio_set_val(port, pin, val);
}

u8 gpio_get_board_config(void)
{
#ifdef CPU_LF1000
	return ((gpio_get_scratch() >> SCRATCH_BOARD_ID_POS) & 
			BIT_MASK(GPIO_CFG_HIGH-GPIO_CFG_LOW+1));
#else
	return 0;
#endif
}

u8 gpio_get_cart_config(void)
{
#ifdef CPU_LF1000
	return ((gpio_get_scratch() >> SCRATCH_CART_ID_POS) & 
			BIT_MASK(GPIO_CART_CFG_HIGH-GPIO_CART_CFG_LOW+1));
#else
	return 0;
#endif
}

EXPORT_SYMBOL(gpio_set_fn);
EXPORT_SYMBOL(gpio_get_fn);
EXPORT_SYMBOL(gpio_set_out_en);
EXPORT_SYMBOL(gpio_set_pu);
EXPORT_SYMBOL(gpio_set_val);
EXPORT_SYMBOL(gpio_get_val);
EXPORT_SYMBOL(gpio_request_irq);
EXPORT_SYMBOL(gpio_free_irq);
EXPORT_SYMBOL(gpio_get_int_mode);
EXPORT_SYMBOL(gpio_set_int_mode);
EXPORT_SYMBOL(gpio_toggle_int_mode);
EXPORT_SYMBOL(gpio_set_int);
EXPORT_SYMBOL(gpio_get_int);
EXPORT_SYMBOL(gpio_get_int32);
EXPORT_SYMBOL(gpio_set_int32);
EXPORT_SYMBOL(gpio_get_pend);
EXPORT_SYMBOL(gpio_clear_pend);
EXPORT_SYMBOL(gpio_get_pend32);
EXPORT_SYMBOL(gpio_clear_pend32);
EXPORT_SYMBOL(gpio_configure_pin);
EXPORT_SYMBOL(gpio_get_board_config);
EXPORT_SYMBOL(gpio_get_cart_config);
#ifdef CPU_LF1000
EXPORT_SYMBOL(gpio_get_cur);
EXPORT_SYMBOL(gpio_set_cur);
EXPORT_SYMBOL(gpio_get_scratch_power);
EXPORT_SYMBOL(gpio_set_scratch_power);
#endif
