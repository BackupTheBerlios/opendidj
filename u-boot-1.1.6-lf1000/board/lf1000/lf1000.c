/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * (C) Copyright 2002
 * David Mueller, ELSOFT AG, <d.mueller@elsoft.ch>
 *
 * (C) Copyright 2003
 * Texas Instruments, <www.ti.com>
 * Kshitij Gupta <Kshitij@ti.com>
 *
 * (C) Copyright 2004
 * ARM Ltd.
 * Philippe Robin, <philippe.robin@arm.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include "autoconf.h"
#include <arch-lf1000/platform.h>
#include <arch-lf1000/common.h>
#include <arch-lf1000/gpio.h>
#include <arch-lf1000/gpio_hal.h>
#include <common.h>
#include <asm/mach-types.h>
#include <arch-lf1000/mem_controller.h>
#include <arch-lf1000/clkpwr.h>

/* GPIO pins */
#define ETHER_RST	30 /* ETHER_RST signal, on port B */
#define BACKLIGHT	28 /* LED_PWM signal (LCD backlight), on port B */

/* XXX this is a temporary hack to allow us to use the SET_MRS macros when not
 *     running virtually. */
#undef mrs_read
#undef mrs_write
#define mrs_read(off)           (*((u32*)(LF1000_SYS_IO+(off))))
#define mrs_write(off, val)     (*((u32*)(LF1000_SYS_IO+(off))) = (val))

DECLARE_GLOBAL_DATA_PTR;

void ether__init (void);
void lcd__init(void);
void peripheral_power_enable (void);

#if defined(CONFIG_SHOW_BOOT_PROGRESS)
void show_boot_progress(int progress)
{
    printf("Boot reached stage %d\n", progress);
}
#endif

#define COMP_MODE_ENABLE ((unsigned int)0x0000EAEF)

static inline void delay (unsigned long loops)
{
	__asm__ volatile ("1:\n"
		"subs %0, %1, #1\n"
		"bne 1b":"=r" (loops):"0" (loops));
}

/*
 * Miscellaneous platform dependent initialisations
 */
int board_init (void)
{
	volatile unsigned int test;

	icache_enable ();
	ether__init ();
	lcd__init();

	gd->bd->bi_arch_number = MACH_TYPE_LF1000;
	gd->bd->bi_boot_params = CONFIG_LF1000_BOOT_PARAMS_ADDR;

	return 0;
}

#define MAX_MTD_PARTS 512
static char mtdparts[MAX_MTD_PARTS];
int misc_init_r (void)
{
	char *ptr = mtdparts;
	int len = 0;

	setenv("verify", "n");

	/* set up the mtd partitions like in Linux.  The CFG_JFFS_CUSTOM_PART
	 * feature appears to be unimplemented.  Wouldn't it be great if we
	 * could simply have a list of partition sizes in the .config, instead
	 * of many individual partition sizes?
	 */
#define LF_P0 (CONFIG_NAND_LF1000_P0_SIZE)
#define LF_P1 (CONFIG_NAND_LF1000_P1_SIZE)
#define LF_P2 (CONFIG_NAND_LF1000_P2_SIZE)
#define LF_P3 (CONFIG_NAND_LF1000_P3_SIZE)
#define LF_P4 (CONFIG_NAND_LF1000_P4_SIZE)
#define LF_P5 (CONFIG_NAND_LF1000_P5_SIZE)
#define LF_P6 (CONFIG_NAND_LF1000_P6_SIZE)

	len = sprintf(ptr, "mtdparts=");
	if(len < MAX_MTD_PARTS)
		len += sprintf(ptr + len, "%s:0x%x@0x%x,",
			       NAND_CHIP_NAME, LF_P0, 0);
	if(len < MAX_MTD_PARTS)
		len += sprintf(ptr + len, "0x%x@0x%x,",
			       LF_P1, LF_P0);
	if(len < MAX_MTD_PARTS)
		len += sprintf(ptr + len, "0x%x@0x%x,",
			       LF_P2, LF_P1 + LF_P0);
	if(len < MAX_MTD_PARTS)
		len += sprintf(ptr + len, "0x%x@0x%x,",
			       LF_P3, LF_P0 + LF_P1 + LF_P2);
	if(len < MAX_MTD_PARTS)
		len += sprintf(ptr + len, "0x%x@0x%x,",
			       LF_P4, LF_P0 + LF_P1 + LF_P2 + LF_P3);
	if(len < MAX_MTD_PARTS)
		len += sprintf(ptr + len, "0x%x@0x%x,",
			       LF_P5, LF_P0 + LF_P1 + LF_P2 + LF_P3 + LF_P4);
	if(len < MAX_MTD_PARTS)
		len += sprintf(ptr + len, "0x%x@0x%x",
			       LF_P6, LF_P0 + LF_P1 + LF_P2 + LF_P3 + LF_P4 + LF_P5);

	setenv("mtdparts", ptr);
	setenv("partition", "nand0,3");

#undef LF_P0
#undef LF_P1
#undef LF_P2
#undef LF_P3

	return (0);
}

void lcd__init(void)
{
#if 1
#ifndef CPU_LF1000
	/* set LED_PWM (backlight pin) as output */
	REG32(LF1000_GPIO_BASE+GPIOBOUTENB) |= (1<<BACKLIGHT);

	/* assert LED_PWM to power the backlight */
	REG32(LF1000_GPIO_BASE+GPIOBOUT) |= (1<<BACKLIGHT);
#endif
#endif
}

/*************************************************************
 Routine:ether__init
 Description: take the Ethernet controller out of reset and wait
	  		   for the EEPROM load to complete.
*************************************************************/
void ether__init (void)
{
	volatile unsigned int test;

#ifdef CONFIG_MACH_LF_MP2530F
	/* set up nSCS1 (chip select) */
	REG32(LF1000_GPIO_BASE+GPIODALTFN0) &= ~(0x3<<GPIO_PIN0);
	REG32(LF1000_GPIO_BASE+GPIODALTFN0) |= (GPIO_ALT1<<GPIO_PIN0);
	/* take CS8900 out of reset */
	REG32(LF1000_GPIO_BASE+GPIOBOUTENB) |= (1<<ETHER_RST);
	REG32(LF1000_GPIO_BASE+GPIOBOUT) &= ~(1<<ETHER_RST);

	/* set bus for 16-bit mode */
	//REG32(LF1000_MCU_S_BASE+MEMBW) |= (1<<LF1000_ETH);
	/* slow down accesses to meet CS8900 timing */
	//REG32(LF1000_MCU_S_BASE+MEMTIMEACCL) |= (0xF<<(4*LF1000_ETH));

	/* set up static timing for Ethernet chip */
	SET_MRS(LF1000_ETH, 1, LF1000_MEMBW_OFF,		    0x01);
	SET_MRS(LF1000_ETH, 2, LF1000_MEMTIMEACS_OFF,		0x02);
	SET_MRS(LF1000_ETH, 2, LF1000_MEMTIMECOS_OFF,		0x02);
	SET_MRS(LF1000_ETH, 4, LF1000_MEMTIMEACCL_OFF,		0x0f);
	SET_MRS(LF1000_ETH, 4, LF1000_MEMTIMESACCL_OFF,		0x0f);
	SET_MRS(LF1000_ETH, 2, LF1000_MEMTIMECOH_OFF,		0x02);
	SET_MRS(LF1000_ETH, 2, LF1000_MEMTIMECAH_OFF,		0x02);
	SET_MRS(LF1000_ETH, 4, LF1000_MEMTIMEBURSTL_OFF,	0x00);
	SET_MRS(LF1000_ETH, 2, LF1000_MEMWAIT_OFF,		    0x03);
#endif

#if defined CONFIG_MACH_ME_LF1000 || defined CONFIG_MACH_LF_LF1000
	/* set up nSCS2 (chip select) */
	REG32(LF1000_GPIO_BASE+GPIOCALTFN0) &= ~(0x3<<GPIO_PIN15*2);
	REG32(LF1000_GPIO_BASE+GPIOCALTFN0) |= (GPIO_ALT1<<GPIO_PIN15*2);

	/* set up static timing for Ethernet chip */
	SET_MRS(LF1000_ETH, 1, LF1000_MEMBW_OFF,		    0x01);
	SET_MRS(LF1000_ETH, 2, LF1000_MEMTIMEACS_OFF,		0x02);
	SET_MRS(LF1000_ETH, 2, LF1000_MEMTIMECOS_OFF,		0x02);
	SET_MRS(LF1000_ETH, 4, LF1000_MEMTIMEACCL_OFF,		0x0f);
	SET_MRS(LF1000_ETH, 4, LF1000_MEMTIMESACCL_OFF,		0x0f);
	SET_MRS(LF1000_ETH, 2, LF1000_MEMTIMECOH_OFF,		0x02);
	SET_MRS(LF1000_ETH, 2, LF1000_MEMTIMECAH_OFF,		0x02);
	SET_MRS(LF1000_ETH, 4, LF1000_MEMTIMEBURSTL_OFF,	0x00);
	SET_MRS(LF1000_ETH, 2, LF1000_MEMWAIT_OFF,		    0x03);
#endif
}

/******************************
 Routine:
 Description:
******************************/
int dram_init (void)
{
	return 0;
}
