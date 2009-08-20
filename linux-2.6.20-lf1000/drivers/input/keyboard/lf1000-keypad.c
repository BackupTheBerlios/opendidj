/*
 * drivers/input/keyboard/lf1000.c
 *
 * Keyboard/Buttons driver for the LF1000 Development Board
 *
 * Copyright 2008 LeapFrog Enterprises Inc.
 *
 * Andrey Yurovsky <andrey@cozybit.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/platform_device.h>

#include <asm/arch/platform.h>
#include <asm/arch/gpio.h>
#include <asm/arch/boards.h>

/* software debouncing: number of ISRs for which a button must be held before 
 * we change state. */
#define BUTTON_DELAY	4

/*
 * Key Map
 */

struct button_entry {
	enum gpio_port port;
	enum gpio_pin pin;
	unsigned char key;
	unsigned int type;			/* event: EV_KEY or EV_SW */

	enum gpio_interrupt_mode push;		/* IRQ for 'pushed' */
	enum gpio_interrupt_mode release;	/* IRQ for 'released' */
	unsigned int debounce;
};

/* the physical button map */
static struct button_entry button_map[] = {
	{GPIO_PORT_C, GPIO_PIN2, KEY_UP, EV_KEY},
	{GPIO_PORT_C, GPIO_PIN3, KEY_DOWN, EV_KEY},
	{GPIO_PORT_C, GPIO_PIN1, KEY_RIGHT, EV_KEY},
	{GPIO_PORT_C, GPIO_PIN0, KEY_LEFT, EV_KEY},
	{GPIO_PORT_C, GPIO_PIN8, KEY_A, EV_KEY},
	{GPIO_PORT_C, GPIO_PIN9, KEY_B, EV_KEY},
#if defined(CONFIG_MACH_ME_LF1000)
	{GPIO_PORT_C, GPIO_PIN7, KEY_L, EV_KEY},	/* L 'shoulder' */
	{GPIO_PORT_C, GPIO_PIN4, KEY_R, EV_KEY},	/* R 'shoulder' */
#else /* CONFIG_MACH_LF_LF1000 */
	{GPIO_PORT_C, GPIO_PIN4, KEY_L, EV_KEY},
	{GPIO_PORT_C, GPIO_PIN7, KEY_R, EV_KEY},
#endif
	{GPIO_PORT_C, GPIO_PIN10, KEY_M, EV_KEY},	/* menu / start */
	{GPIO_PORT_C, GPIO_PIN11, KEY_H, EV_KEY},	/* hint */
	{GPIO_PORT_C, GPIO_PIN12, KEY_P, EV_KEY},	/* pause */
	{GPIO_PORT_C, GPIO_PIN13, KEY_X, EV_KEY},	/* brightness */

	{GPIO_PORT_B, GPIO_PIN10, SW_HEADPHONE_INSERT, EV_SW},
	{GPIO_PORT_A, GPIO_PIN18, SW_TABLET_MODE, EV_SW}, /* cartridge ins. */
};

/* Keycodes that we can generate.  Although codes like KEY_MENU are defined, it
 * is easier to test with the usual alphabet keys, so we do not use the special
 * definitions. */
static unsigned char lf1000_keycode[]= {
	KEY_UP, KEY_DOWN, KEY_RIGHT, KEY_LEFT, 
	KEY_A, KEY_B, KEY_L, KEY_R, KEY_M, KEY_H, KEY_P, KEY_X};

/*
 * device
 */

struct lf1000_kp {
	unsigned char keycode[ARRAY_SIZE(lf1000_keycode)];
	struct input_dev *input;
};

/*
 * IRQ handling
 */

static irqreturn_t button_irq(enum gpio_port port, enum gpio_pin pin, 
				void *priv)
{
	struct lf1000_kp *lf1000_kp_dev = (struct lf1000_kp *)priv;
	char key;
	enum gpio_interrupt_mode mode;
	struct button_entry *bme;
	int i;

	/* check for buttons to report */
	for(i = 0; i < ARRAY_SIZE(button_map); i++) {
		bme = &button_map[i];
		if(gpio_get_pend(bme->port, bme->pin)) {
			if(++bme->debounce < BUTTON_DELAY) {
				gpio_clear_pend(bme->port, bme->pin);
				continue;
			}
			bme->debounce = 0;
			key = bme->key;
			mode = gpio_get_int_mode(bme->port, bme->pin);
			
			if(mode == bme->push)
				input_event(lf1000_kp_dev->input, bme->type, 
						key, 1);
			else if(mode == bme->release)
				input_event(lf1000_kp_dev->input, bme->type, 
						key, 0);

			gpio_toggle_int_mode(bme->port, bme->pin);
			gpio_clear_pend(bme->port, bme->pin);
		}
	}

	return IRQ_HANDLED;
}

/*
 * platform device
 */

static int lf1000_kp_probe(struct platform_device *pdev)
{
	struct lf1000_kp *lf1000_kp_dev;
	struct input_dev *input_dev;
	int i;
	int ret;

	lf1000_kp_dev = kzalloc(sizeof(struct lf1000_kp), GFP_KERNEL);
	if(!lf1000_kp_dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, lf1000_kp_dev);

	input_dev = input_allocate_device();
	if(!input_dev) {
		ret = -ENOMEM;
		goto fail_input;
	}

	memcpy(lf1000_kp_dev->keycode, lf1000_keycode, 
			sizeof(lf1000_kp_dev->keycode));

	input_dev->name = "LF1000 Keyboard";
	input_dev->phys = "lf1000/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0001;
	input_dev->cdev.dev = &pdev->dev;
	input_dev->private = lf1000_kp_dev;
	lf1000_kp_dev->input = input_dev;	

	/* event types that we support */
	input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_REP) | BIT(EV_SW);

	input_dev->keycode = lf1000_kp_dev->keycode;
	input_dev->keycodesize = sizeof(unsigned char);
	input_dev->keycodemax = ARRAY_SIZE(lf1000_keycode);

	for(i = 0; i < ARRAY_SIZE(lf1000_keycode); i++)
		set_bit(lf1000_kp_dev->keycode[i], input_dev->keybit);

	/* audio jack */
	set_bit(SW_HEADPHONE_INSERT, input_dev->swbit);
	/* reusing this for cartridge detect: */
	set_bit(SW_TABLET_MODE, input_dev->swbit);

	ret = input_register_device(lf1000_kp_dev->input);
	if(ret)
		goto fail_register;

	/* 
	 * initialize and claim the buttons/switches, enable interrupts 
	 */

	for(i = 0; i < ARRAY_SIZE(button_map); i++) {
		button_map[i].debounce = 0;
		button_map[i].push = GPIO_IMODE_LOW_LEVEL;
		button_map[i].release = GPIO_IMODE_HIGH_LEVEL;
		if(button_map[i].key == SW_HEADPHONE_INSERT) {
			/* reverse headphone jack on EP1 and newer boards */
			if(gpio_get_board_config() >= LF1000_BOARD_EP1) {
				button_map[i].push = GPIO_IMODE_HIGH_LEVEL;
				button_map[i].release = GPIO_IMODE_LOW_LEVEL;
			}
		}
		gpio_configure_pin(button_map[i].port, button_map[i].pin,
			GPIO_GPIOFN, 0, 0, 0);
		gpio_set_int_mode(button_map[i].port, button_map[i].pin,
				button_map[i].push); 
		ret = gpio_request_irq(button_map[i].port, button_map[i].pin,
				button_irq, lf1000_kp_dev);
		if(ret) {
			printk(KERN_ALERT "lf1000-kp: can't claim pin %d.%d\n",
					button_map[i].port, button_map[i].pin);
			goto fail_irq;
		}
		gpio_set_int(button_map[i].port, button_map[i].pin, 1);
	}

	return 0;

fail_irq:
	i--;
	for(; i >= 0; i--) /* free any IRQs that we managed to request */
		gpio_free_irq(button_map[i].port, button_map[i].pin,
				button_irq);
fail_register:
	input_free_device(input_dev);
fail_input:
	kfree(lf1000_kp_dev);
	return ret;
}

static int lf1000_kp_remove(struct platform_device *pdev)
{
	struct lf1000_kp *lf1000_kp_dev = platform_get_drvdata(pdev);
	int i;

	/* free pin IRQs */
	for(i = 0; i < ARRAY_SIZE(button_map); i++) {
		gpio_free_irq(button_map[i].port, button_map[i].pin,
				button_irq);
	}

	input_unregister_device(lf1000_kp_dev->input);
	kfree(lf1000_kp_dev);

	return 0;
}

static struct platform_driver lf1000_kp_driver = {
	.probe		= lf1000_kp_probe,
	.remove		= lf1000_kp_remove,
	.driver		= {
		.name	= "lf1000-keypad",
	},
};

/*
 * module stuff
 */

static int __devinit lf1000_kp_init(void)
{
	return platform_driver_register(&lf1000_kp_driver);
}

static void __exit lf1000_kp_exit(void)
{
	platform_driver_unregister(&lf1000_kp_driver);
}

module_init(lf1000_kp_init);
module_exit(lf1000_kp_exit);

MODULE_AUTHOR("Andrey Yurovsky <andrey@cozybit.com>");
MODULE_DESCRIPTION("LF1000 Development Board buttons driver");
MODULE_LICENSE("GPL");
