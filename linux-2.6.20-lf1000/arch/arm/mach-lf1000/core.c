/*
 * arch/arm/mach-lf1000/core.c
 *
 * Copyright LeapFrog Enterprises Inc.
 *
 * Yakov A. Blyakhman <YBlyakhman@leapfrog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/interrupt.h>
#include <linux/console.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>

#include <asm/arch/platform.h>
#include <asm/arch/common.h>
#include <asm/arch/uart.h>
#include <asm/arch/nand.h>
#include <asm/arch/clkpwr.h>
#include <asm/arch/gpio.h>
#include <asm/arch/gpio_hal.h>
#include <asm/arch/timer.h>
#include <asm/arch/ic.h>
#include <asm/cnt32_to_63.h>
#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/leds.h>
#include <asm/hardware/lf1000.h>
#include <asm/hardware/vic.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>

#include "core.h"
#include "clock.h"

/*
 * All IO addresses are mapped onto VA 0xFFFx.xxxx, where x.xxxx
 * is the (PA >> 12).
 *
 * Setup a VA for the Lf1000 Vectored Interrupt Controller.  Note that INTMODEL
 * is the start of the Interrupt Controller registers, LF1000_IC_BASE starts
 * with a coupe of words that are reserved.
 */
#define LF1000_VA_IC_BASE	IO_ADDRESS(LF1000_IC_BASE+INTMODEL)

#define	BP( bit_position)	( (u64)1ULL << bit_position)
void __init lf1000_init_irq(void)
{
	ic_init(__io(LF1000_VA_IC_BASE), IRQ_IC_START,
			BP(LF1000_SYS_TIMER_IRQ)|
			BP(LF1000_DMA_IRQ)	|
			BP(LF1000_ADC_IRQ)	|
			BP(LF1000_AUDIO_IRQ)	|
			BP(LF1000_SPI0_IRQ)	|
			BP(LF1000_GPIO_IRQ)	|
			BP(LF1000_I2C0_IRQ)	|
			BP(LF1000_RTC_IRQ)	|
			BP(LF1000_SYS_UART_IRQ)	|
			BP(LF1000_UDC_IRQ)	|
			BP(LF1000_AUDIO_IRQ)	|
#if defined CONFIG_MMC_LF1000 || defined CONFIG_MMC_LF1000_MODULE
			BP(LF1000_SDIO0_IRQ)	|
#	ifdef CPU_LF1000
			BP(LF1000_SDIO1_IRQ)	|
#	endif
#endif /* CONFIG_MMC_LF1000 || CONFIG_MMC_LF1000_MODULE */
			BP(LF1000_EXTINT1));
}

static struct map_desc lf1000_io_desc[] __initdata = {
	{
		.virtual	=  IO_ADDRESS(LF1000_NAND_BASE),
		.pfn		= __phys_to_pfn(LF1000_NAND_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	}, {
		.virtual	=  IO_ADDRESS(LF1000_SYS_BASE),
		.pfn		= __phys_to_pfn(LF1000_SYS_BASE),
		.length		= SZ_4K*32,
		.type		= MT_DEVICE
	}, {
		.virtual	=  IO_ADDRESS(LF1000_3DGE_BASE),
		.pfn		= __phys_to_pfn(LF1000_3DGE_BASE),
		.length		= SZ_4K*2,
		.type		= MT_DEVICE
	}, 
#ifndef CONFIG_MACH_LF_LF1000
	{
		.virtual	=  IO_ADDRESS(LF1000_ETH_BASE),
		.pfn		= __phys_to_pfn(LF1000_ETH_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
#endif
};

void __init lf1000_map_io(void)
{
	iotable_init(lf1000_io_desc, ARRAY_SIZE(lf1000_io_desc));
#ifdef CONFIG_ARCH_LF1000
	// need early clock initialization
	lf1000_clock_init();
#endif

}

struct resource lf1000_audio_resources[] = {
	[0] = {
		.start		= LF1000_AUDIO_BASE,
		.end		= LF1000_AUDIO_END,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= LF1000_AUDIO_IRQ,
		.end		= LF1000_AUDIO_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device lf1000_audio_device = {
	.name			= "lf1000-audio",
	.id			= -1,
	.num_resources		= ARRAY_SIZE(lf1000_audio_resources),
	.resource		= lf1000_audio_resources,
};

struct resource lf1000_i2c_resources[] = {
	[0] = {
		.start		= LF1000_I2C0_BASE,
		.end		= LF1000_I2C0_END,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= LF1000_I2C0_IRQ,
		.end		= LF1000_I2C0_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
	[2] = {
		.start		= LF1000_I2C1_BASE,
		.end		= LF1000_I2C1_END,
		.flags		= IORESOURCE_MEM,
	},
	[3] = {
		.start		= LF1000_I2C1_IRQ,
		.end		= LF1000_I2C1_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device lf1000_i2c_device = {
	.name			= "lf1000-i2c",
	.id			= -1,
	.num_resources		= ARRAY_SIZE(lf1000_i2c_resources),
	.resource		= lf1000_i2c_resources,
};

#if defined CONFIG_MMC_LF1000 || defined CONFIG_MMC_LF1000_MODULE
struct resource lf1000_sdio_resources[] = {
	[0] = {
		.start		= LF1000_SDIO0_BASE,
		.end		= LF1000_SDIO0_END,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= LF1000_SDIO0_IRQ,
		.end		= LF1000_SDIO0_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
#ifdef CPU_LF1000
	[2] = {
		.start		= LF1000_SDIO1_BASE,
		.end		= LF1000_SDIO1_END,
		.flags		= IORESOURCE_MEM,
	},
	[3] = {
		.start		= LF1000_SDIO1_IRQ,
		.end		= LF1000_SDIO1_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
#endif /* CPU_LF1000 */
};

struct platform_device lf1000_sdio_device = {
	.name			= "lf1000-sdio",
	.id			= -1,
	.num_resources		= ARRAY_SIZE(lf1000_sdio_resources),
	.resource		= lf1000_sdio_resources,
};
#endif /* CONFIG_MMC_LF1000 || CONFIG_MMC_LF1000_MODULE */

struct resource lf1000_nand_resource = {
	.start			= LF1000_NAND_BASE,
	.end			= LF1000_NAND_END,
	.flags			= IORESOURCE_MEM,
};

struct platform_device lf1000_nand_device = {
	.name			= "lf1000-nand",
	.id			= -1,
	.num_resources		= 1,
	.resource		= &lf1000_nand_resource,
};

struct resource lf1000_power_resource = {
	.start			= LF1000_CLKPWR_BASE,
	.end			= LF1000_CLKPWR_END,
	.flags			= IORESOURCE_MEM,
};

struct platform_device lf1000_power_device = {
	.name			= "lf1000-power",
	.id			= -1,
	.num_resources		= 1,
	.resource		= &lf1000_power_resource,
};

struct resource lf1000_rtc_resource = {
	.start			= LF1000_RTC_BASE,
	.end			= LF1000_RTC_END,
	.flags			= IORESOURCE_MEM,
};

struct platform_device lf1000_rtc_device = {
	.name			= "lf1000-rtc",
	.id			= -1,
	.num_resources		= 1,
	.resource		= &lf1000_rtc_resource,
};

struct resource lf1000_dpc_resource = {
	.start			= LF1000_DPC_BASE,
	.end			= LF1000_DPC_END,
	.flags			= IORESOURCE_MEM,
};

struct platform_device lf1000_dpc_device = {
	.name			= "lf1000-dpc",
	.id			= -1,
	.num_resources		= 1,
	.resource		= &lf1000_dpc_resource,
};

struct resource lf1000_mlc_resource = {
	.start			= LF1000_MLC_BASE,
	.end			= LF1000_MLC_END,
	.flags			= IORESOURCE_MEM,
};

struct platform_device lf1000_mlc_device = {
	.name			= "lf1000-mlc",
	.id			= -1,
	.num_resources		= 1,
	.resource		= &lf1000_mlc_resource,
};

struct resource lf1000_adc_resources[] = {
	[0] = {
		.start		= LF1000_ADC_BASE,
		.end		= LF1000_ADC_END,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= LF1000_ADC_IRQ,
		.end		= LF1000_ADC_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device lf1000_adc_device = {
	.name			= "lf1000-adc",
	.id			= -1,
	.num_resources		= ARRAY_SIZE(lf1000_adc_resources),
	.resource		= lf1000_adc_resources,
};

struct resource lf1000_gpio_resources[] = {
	[0] = {
		.start		= LF1000_GPIO_BASE,
		.end		= LF1000_GPIO_END,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= LF1000_GPIOCURRENT_BASE,
		.end		= LF1000_GPIOCURRENT_END,
		.flags		= IORESOURCE_MEM,
	},
	[2] = {
		.start		= LF1000_GPIO_IRQ,
		.end		= LF1000_GPIO_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device lf1000_gpio_device = {
	.name			= "lf1000-gpio",
	.id			= -1,
	.num_resources		= ARRAY_SIZE(lf1000_gpio_resources),
	.resource		= lf1000_gpio_resources,
};

struct resource lf1000_alvgpio_resource = {
	.start			= LF1000_ALIVE_BASE,
	.end			= LF1000_ALIVE_END,
	.flags			= IORESOURCE_MEM,
};

struct platform_device lf1000_alvgpio_device = {
	.name			= "lf1000-alvgpio",
	.id			= -1,
	.num_resources		= 1,
	.resource		= &lf1000_alvgpio_resource,
};

#if defined CONFIG_KEYBOARD_LF1000 || defined CONFIG_KEYBOARD_LF1000_MODULE
struct platform_device lf1000_kp_device = {
	.name			= "lf1000-keypad",
	.id			= -1,
	.num_resources		= 0,
};
#endif

struct resource lf1000_pwm_resource = {
	.start			= LF1000_PWM_BASE,
	.end			= LF1000_PWM_END,
	.flags			= IORESOURCE_MEM,
};

struct platform_device lf1000_pwm_device = {
	.name			= "lf1000-pwm",
	.id			= -1,
	.num_resources		= 1,
	.resource		= &lf1000_pwm_resource,
};

struct resource lf1000_ga3d_resource = {
	.start			= LF1000_GA3D_BASE,
	.end			= LF1000_GA3D_END,
	.flags			= IORESOURCE_MEM,
};

struct platform_device lf1000_ga3d_device = {
	.name			= "lf1000-ga3d",
	.id			= -1,
	.num_resources		= 1,
	.resource		= &lf1000_ga3d_resource,
};

struct resource lf1000_idct_resource = {
	.start			= LF1000_IDCT_BASE,
	.end			= LF1000_IDCT_END,
	.flags			= IORESOURCE_MEM,
};

struct platform_device lf1000_idct_device = {
	.name			= "lf1000-idct",
	.id			= -1,
	.num_resources		= 1,
	.resource		= &lf1000_idct_resource,
};

struct resource lf1000_spi_resources[] = {
	[0] = {
		.start		= LF1000_SPI0_BASE,
		.end		= LF1000_SPI0_END,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= LF1000_SPI0_IRQ,
		.end		= LF1000_SPI0_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
	[2] = {
		.start		= LF1000_SPI1_BASE,
		.end		= LF1000_SPI1_END,
		.flags		= IORESOURCE_MEM,
	},
	[3] = {
		.start		= LF1000_SPI1_IRQ,
		.end		= LF1000_SPI1_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
#ifdef CPU_MF2530F
	[4] = {
		.start		= LF1000_SPI2_BASE,
		.end		= LF1000_SPI2_END,
		.flags		= IORESOURCE_MEM,
	},
	[5] = {
		.start		= LF1000_SPI2_IRQ,
		.end		= LF1000_SPI2_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
#endif
};

struct platform_device lf1000_spi_device = {
	.name			= "lf1000-spi",
	.id			= -1,
	.num_resources		= ARRAY_SIZE(lf1000_spi_resources),
	.resource		= lf1000_spi_resources,
};

struct resource lf1000_dma_resources[] = {
	[0] = {
		.start		= LF1000_DMA_BASE,
		.end		= LF1000_DMA_BASE + 2*1024 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= LF1000_DMA_IRQ,
		.end		= LF1000_DMA_IRQ,
		.flags		= IORESOURCE_IRQ,
	},

};

struct platform_device lf1000_dma_device = {
	.name			= "lf1000-dma",
	.id			= -1,
	.num_resources		= ARRAY_SIZE(lf1000_dma_resources),
	.resource		= lf1000_dma_resources,
};

/* Platform driver for USB Device Controller */
struct resource lf1000_udc_resources[] = {
	[0] = {
		.start		= LF1000_UDC_BASE,
		.end		= LF1000_UDC_END,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= LF1000_UDC_IRQ,
		.end		= LF1000_UDC_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device lf1000_udc_device = {
	.name			= "lf1000-usbgadget",
	.id			= -1,
	.num_resources		= ARRAY_SIZE(lf1000_udc_resources),
	.resource		= lf1000_udc_resources,
};

static struct platform_device *devices[] __initdata = {
	&lf1000_gpio_device,
	&lf1000_alvgpio_device,
	&lf1000_nand_device,
	&lf1000_pwm_device,
	&lf1000_i2c_device,
	&lf1000_spi_device,
	&lf1000_adc_device,
	&lf1000_dpc_device,
	&lf1000_mlc_device,
	&lf1000_dma_device,
	&lf1000_udc_device,
	&lf1000_rtc_device,
	&lf1000_audio_device,
	&lf1000_ga3d_device,
	&lf1000_power_device,
#if defined CONFIG_MMC_LF1000 || defined CONFIG_MMC_LF1000_MODULE
	&lf1000_sdio_device,
#endif
#if defined CONFIG_KEYBOARD_LF1000 || defined CONFIG_KEYBOARD_LF1000_MODULE
	&lf1000_kp_device,
#endif
	&lf1000_idct_device,
};


/*
 * Poweroff and Reset
 */

static void lf1000_poweroff(void)
{
	gpio_set_val(GPIO_PORT_ALV, GPIO_PIN0, 0);      // nCOM1_ENA
	gpio_set_val(GPIO_PORT_ALV, GPIO_PIN1, 0);      // NAND_nWP
	gpio_set_val(GPIO_PORT_ALV, GPIO_PIN3, 0);      // USBH_PWR
	gpio_set_val(GPIO_PORT_ALV, GPIO_PIN4, 0);      // nDAC_ENA
	gpio_set_val(GPIO_PORT_ALV, GPIO_PIN5, 0);      // LCD_nRES
	gpio_set_val(GPIO_PORT_ALV, GPIO_PIN6, 0);      // nDAC_RES

	// power-down LED_PWM (PWMOUT[0]) and LED_BRIGHT (PWMOUT[1])
	gpio_set_val(GPIO_PORT_A, GPIO_PIN30, 0);       // PWMOUT[0]
	gpio_set_val(GPIO_PORT_A, GPIO_PIN31, 0);       // PWMPUT[1]

	/* cut the power */
	gpio_set_val(GPIO_PORT_ALV, VDDPWRONSET, 0);

	arm_machine_restart('h');
}

static void lf1000_restart(char mode)
{
	u32 tmp = REG32(IO_ADDRESS(LF1000_CLKPWR_BASE+PWRMODE));

	BIT_SET(tmp, GPIOSWRSTENB);
	BIT_CLR(tmp, SWRST);
	REG32(IO_ADDRESS(LF1000_CLKPWR_BASE+PWRMODE)) = tmp;
	BIT_SET(tmp, SWRST);
	REG32(IO_ADDRESS(LF1000_CLKPWR_BASE+PWRMODE)) = tmp;

	arm_machine_restart('r');
}

/*
 * Machine Initialization
 */

void __init lf1000_init(void)
{
	pm_power_off = lf1000_poweroff;
	arm_pm_restart = lf1000_restart;

	platform_add_devices(devices, ARRAY_SIZE(devices));
}

/* configure static bus timings as needed */
static void __init lf1000_acc_time_init( void)
{
	volatile unsigned int test;

#if defined CONFIG_MACH_ME_LF1000 || defined CONFIG_MACH_LF_MP2530F
	/* set up static bus timings for CS8900 Ethernet controller on the 
	 * ME_LF1000 develpment board */
	SET_MRS(LF1000_ETH, 1, LF1000_MEMBW_OFF,	0x01);
	SET_MRS(LF1000_ETH, 2, LF1000_MEMTIMEACS_OFF,	0x02);
	SET_MRS(LF1000_ETH, 2, LF1000_MEMTIMECOS_OFF,	0x02);
	SET_MRS(LF1000_ETH, 4, LF1000_MEMTIMEACCL_OFF,	0x0f);
	SET_MRS(LF1000_ETH, 4, LF1000_MEMTIMESACCL_OFF,	0x0f);
	SET_MRS(LF1000_ETH, 2, LF1000_MEMTIMECOH_OFF,	0x02);
	SET_MRS(LF1000_ETH, 2, LF1000_MEMTIMECAH_OFF,	0x02);
	SET_MRS(LF1000_ETH, 4, LF1000_MEMTIMEBURSTL_OFF,0x00);
	SET_MRS(LF1000_ETH, 2, LF1000_MEMWAIT_OFF,	0x03);
#endif
}

//////////////////////////// UART setup ///////////////////////////////////////
/* default settings for system UART */
static int sys_uart	= LF1000_SYS_UART;
static int sys_uart_br = LF1000_SYS_UART_BR;

static void __init lf1000_console_setup( char* name, int idx, char* options)
{
	if(!strcmp(name, "ttyS")) {
		if(idx < UART_MAX) {
			sys_uart = idx;
			sys_uart_br = simple_strtoul( options, NULL, 10);
			set_uart_baud( sys_uart, sys_uart_br);
		}
	}
}

static int __init console_setup(char *str)
{
//	char name[sizeof(console_cmdline[0].name)];
	char name[16];	// really is 8 bytes
	char *s, *options;
	int idx;

	/*
	 * Decode str into name, index, options.
	 */
	if (str[0] >= '0' && str[0] <= '9') {
		strcpy(name, "ttyS");
		strncpy(name + 4, str, sizeof(name) - 5);
	} else {
		strncpy(name, str, sizeof(name) - 1);
	}
	name[sizeof(name) - 1] = 0;
	if ((options = strchr(str, ',')) != NULL)
		*(options++) = 0;

	for (s = name; *s; s++)
		if ((*s >= '0' && *s <= '9') || *s == ',')
			break;
	idx = simple_strtoul(s, NULL, 10);
	*s = 0;

	lf1000_console_setup( name, idx, options);
	add_preferred_console(name, idx, options);
	return 1;
}
__setup("console=", console_setup);

////////////////////////////// PLL setup /////////////////////////////////////
extern int __init clk_init(void);

void __init lf1000_clock_init( void)
{
	// early clock initialization
	clk_init();
	// ether/nand flash access time init
	lf1000_acc_time_init();
}

#define LF1000_TIMER0_VA_BASE		IO_ADDRESS(LF1000_TIMER0_BASE)
#define LF1000_TIMER1_VA_BASE		IO_ADDRESS(LF1000_TIMER1_BASE)
#define LF1000_TIMER2_VA_BASE		IO_ADDRESS(LF1000_TIMER2_BASE)
#define LF1000_TIMER3_VA_BASE		IO_ADDRESS(LF1000_TIMER3_BASE)

/*
 * Returns number of usecs since last clock interrupt.  Note that interrupts
 * will have been disabled by do_gettimeoffset()
 *
 * Preserve tick resolution by scaling tick count up then dividing.
 */

static unsigned long lf1000_gettimeoffset(void)
{
	unsigned long ticks;

	/*
	 * Number of ticks since last interrupt.
	 */
	ticks = get_timer_cnt(LF1000_INTERTICK_TIMER);

	/*
	 * Convert the ticks to usecs.
	 */

	return ((ticks * LF1000_INTERVAL_IN_USEC) / TIMER_SYS_TICK);
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t lf1000_timer_interrupt(int irq, void *dev_id)
{
	unsigned int timer_control, intertick_control;
	unsigned int timer_count, intertick_count;
	struct lf1000_timer* timer_p = get_timer_pnt(LF1000_SYS_TIMER);
	struct lf1000_timer* tick_p = get_timer_pnt(LF1000_INTERTICK_TIMER);
	
	write_seqlock(&xtime_lock);

	// ...clear the interrupt
	clear_timer_irq(irq);

	/* setup for TIMER count read */
	timer_control = ioread32(&timer_p->tmrcontrol);
	timer_control &= ~(1<<INTPEND);    // do not clear pend
	timer_control |=  (1<<LDCNT);      // latch count
	iowrite32( timer_control, &timer_p->tmrcontrol);
	timer_count = ioread32(&timer_p->tmrcount);

	/* setup for TICK count read */
	intertick_control = ioread32(&tick_p->tmrcontrol);
	intertick_control &= ~(1<<INTPEND);	// do not clear pend
	intertick_control |=  (1<<LDCNT);	// latch count

	/* read current TICK count */
	iowrite32(intertick_control, &tick_p->tmrcontrol);// snapshot tick count
	intertick_count = ioread32(&tick_p->tmrcount);

	/* update TICK count with TIMER count */	
	iowrite32(timer_control, &timer_p->tmrcontrol); // snapshot timer count
	timer_count = ioread32(&timer_p->tmrcount);
	iowrite32(timer_count, &tick_p->tmrcount);	// catchup to SYS_TIMER
	
	/* 
	 * have at least one timer tick, account for additional
	 * ticks recorded by intertick timer
	 */
	do {
		timer_tick();
		if (intertick_count > TIMER_SYS_TICK)
			intertick_count -= TIMER_SYS_TICK;
	} while (intertick_count > TIMER_SYS_TICK);

	write_sequnlock(&xtime_lock);

	return IRQ_HANDLED;
}

static struct irqaction lf1000_timer_irq = {
	.name		= "Lf1000 Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER,
	.handler	= lf1000_timer_interrupt,
};

/*
 * Set up timer interrupt, and return the current time in seconds.
 */
static void __init lf1000_timer_init(void)
{
	int i=0;
	volatile struct lf1000_timer* timer_p;

	while((timer_p = get_timer_pnt(i))) {
		/* enable writes to timer module */
		iowrite32((1<<TCLKMODE)|(1<<TCLKGENENB), &timer_p->tmrclkenb);	
		// start from known values
		iowrite32(0, &timer_p->tmrcount); /* TimerLoad */

		iowrite32(ioread32(&timer_p->tmrcontrol) & ~SELTCLK_MASK,
				&timer_p->tmrcontrol);
		iowrite32(ioread32(&timer_p->tmrcontrol)|SELTCLK,
				&timer_p->tmrcontrol);

		iowrite32(ioread32(&timer_p->tmrclkgen)  & ~(0xff<<TCLKDIV),
				&timer_p->tmrclkgen);
		iowrite32(ioread32(&timer_p->tmrclkgen)|
			((CLKDIVR<<TCLKDIV)|(TIMER_PLL<<TCLKSRCSEL)),
			&timer_p->tmrclkgen);

		/* 
		 * Make IRQs happen for the system timer
		 */
		if(LF1000_SYS_TIMER == i) {
			iowrite32(TIMER_SYS_TICK, &timer_p->tmrmatch);
			setup_irq(get_timer_irq(i), &lf1000_timer_irq);
			/* run timer with interrupts enabled */
			iowrite32(ioread32(&timer_p->tmrcontrol)|
				((1<<RUN)|(1<<INTPEND)|(1<<INTENB_T)),
				&timer_p->tmrcontrol);
		} else {
			iowrite32(TIMER_FREE_RUN, &timer_p->tmrmatch);
			iowrite32(ioread32(&timer_p->tmrcontrol)|
				((1<<RUN)|(1<<INTPEND)),
				&timer_p->tmrcontrol);
		}
		printk("TIM%d=%3d.%03d MHz   ", i, 
				(get_timer_freq(i)/1000)/1000,
				(get_timer_freq(i)/1000)%1000);
		i++;
	}
	printk("\n");
}

struct sys_timer lf1000_timer = {
	.init		= lf1000_timer_init,
	.offset		= lf1000_gettimeoffset,
};
