/*
 * (C) Copyright 2006 DENX Software Engineering
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

#include <configs/lf1000.h>
#include <nand.h>
#include "autoconf.h"
#include <arch-lf1000/platform.h>
#include <arch-lf1000/common.h>
#include <arch-lf1000/nand.h>
#include <arch-lf1000/gpio_hal.h>

#if (CONFIG_COMMANDS & CFG_CMD_NAND)

/* access to WP output level register */
#define WP_OUT	REG32(NAND_WP_OUT+NAND_WP_PIN)

#ifdef CPU_LF1000
/* reset Alive GPIO Flip-Flop */
#define RESET_ALIVE_FF()	\
	REG32(LF1000_ALIVE_BASE+ALIVEGPIOSETREG) = 0;	\
	REG32(LF1000_ALIVE_BASE+ALIVEGPIORSTREG) = 0
#endif

/* 
 * track address and command latch usage 
 */
#define LF1000_CLE	0
#define LF1000_ALE	1
static unsigned int lf1000_latch = 0;

#define NAND_BIG_DELAY_US	25	// NAND chip delay

// pointers to intercepted NAND drivers, used to insert WP commands 
// before/after NAND write/erase routines
static int (*sys_erase) (struct mtd_info *mtd, struct erase_info *instr);
static int (*sys_write) (struct mtd_info *mtd, loff_t to, size_t len, size_t *retlen, const u_char *buf);
static int (*sys_write_ecc) (struct mtd_info *mtd, loff_t to, size_t len, size_t *retlen, const u_char *buf, u_char *eccbuf, struct nand_oobinfo *oobsel);
static int (*sys_write_oob) (struct mtd_info *mtd, loff_t to, size_t len, size_t *retlen, const u_char *buf);


/* NAND WP control:
 *
 * The WP signal is an active-low pin, it is set up as an output pin by
 * board_nand_init */
static void lf1000_NAND_wp(int state)
{
	if(state == 0) { /* disable WP */
#ifdef CPU_MF2530F
		NAND_WP_LEVEL ? BIT_CLR(WP_OUT,NAND_WP_PIN) : 
				BIT_SET(WP_OUT,NAND_WP_PIN);
#elif defined CPU_LF1000
		RESET_ALIVE_FF();
		NAND_WP_LEVEL ? 
		REG32(LF1000_ALIVE_BASE+ALIVEGPIORSTREG) |= (1<<NAND_WP_PIN) :
		REG32(LF1000_ALIVE_BASE+ALIVEGPIOSETREG) |= (1<<NAND_WP_PIN);
		RESET_ALIVE_FF();
#endif
	} else { /* enable WP */
#ifdef CPU_MF2530F
		NAND_WP_LEVEL ? BIT_SET(WP_OUT,NAND_WP_PIN) : 
				BIT_CLR(WP_OUT,NAND_WP_PIN);
#elif defined CPU_LF1000
		RESET_ALIVE_FF();
		NAND_WP_LEVEL ? 
		REG32(LF1000_ALIVE_BASE+ALIVEGPIOSETREG) |= (1<<NAND_WP_PIN) :
		REG32(LF1000_ALIVE_BASE+ALIVEGPIORSTREG) |= (1<<NAND_WP_PIN);
		RESET_ALIVE_FF();
#endif
	}
}

/*
 * local version of intercepted NAND drivers, used to insert WP commands 
 * before/after NAND write/erase routines 
 */
static int lf1000_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int retval;

	lf1000_NAND_wp(0);
	retval = sys_erase(mtd, instr);
	lf1000_NAND_wp(1);
	return retval;
}

static int lf1000_write(struct mtd_info *mtd, loff_t to, size_t len, size_t *retlen, const u_char *buf)
{
	int retval;

	lf1000_NAND_wp(0);
	retval = sys_write(mtd, to, len, retlen, buf);
	lf1000_NAND_wp(1);
	return retval;
}

static int lf1000_write_ecc(struct mtd_info *mtd, loff_t to, size_t len, size_t *retlen, const u_char *buf, u_char *eccbuf, struct nand_oobinfo *oobsel)
{
	int retval;

	lf1000_NAND_wp(0);
	retval = sys_write_ecc(mtd, to, len, retlen, buf, eccbuf, oobsel);
	lf1000_NAND_wp(1);
	return retval;
}

static int lf1000_write_oob(struct mtd_info *mtd, loff_t to, size_t len, size_t *retlen, const u_char *buf)
{
	int retval;

	lf1000_NAND_wp(0);
	retval = sys_write_oob(mtd, to, len, retlen, buf);
	lf1000_NAND_wp(1);
	return retval;
}

/* The LF1000 NAND controller handles the CLE and ALE signals for us.  We just
 * need to write to the appropriate register when we write the data byte.  Here
 * we just set a flag to tell lf1000_write_byte which register to use. */
static void lf1000_hwcontrol(struct mtd_info *mtdinfo, int cmd)
{
	switch(cmd) {
		case NAND_CTL_SETCLE:
		lf1000_latch |= (1<<LF1000_CLE);
		break;
		case NAND_CTL_CLRCLE:
		lf1000_latch &= ~(1<<LF1000_CLE);
		break;
		case NAND_CTL_SETALE:
		lf1000_latch |= (1<<LF1000_ALE);
		break;
		case NAND_CTL_CLRALE:
		lf1000_latch &= ~(1<<LF1000_ALE);
		break;
	};
}

/* the LF1000's NAND controller handles the ALE and CLE signals for us, we
 * just need to write to the appropriate register. */
static void lf1000_write_byte(struct mtd_info *mtdinfo, unsigned char byte)
{
	struct nand_chip *chip = mtdinfo->priv;

	if(lf1000_latch & (1<<LF1000_CLE))
		REG8(chip->IO_ADDR_W+NFCMD) = byte;
	else if(lf1000_latch & (1<<LF1000_ALE))
		REG8(chip->IO_ADDR_W+NFADDR) = byte;
}

/*
 * read device ready pin
 * function +/- borrowed from Linux 2.6 (drivers/mtd/nand_base.c)
 */
static int lf1000_device_ready(struct mtd_info *mtdinfo)
{
	if(REG32(LF1000_MCU_S_BASE+NFCONTROL) & (1<<RnB))
		return 1;	/* NAND ready */
	return 0;		/* NAND busy */
}

/*
 * Board-specific NAND initialization. The following members of the
 * argument are board-specific (per include/linux/mtd/nand.h):
 * - IO_ADDR_R?: address to read the 8 I/O lines of the flash device
 * - IO_ADDR_W?: address to write the 8 I/O lines of the flash device
 * - hwcontrol: hardwarespecific function for accesing control-lines
 * - dev_ready: hardwarespecific function for  accesing device ready/busy line
 * - enable_hwecc?: function to enable (reset)  hardware ecc generator. Must
 *   only be provided if a hardware ECC is available
 * - eccmode: mode of ecc, see defines
 * - chip_delay: chip dependent delay for transfering data from array to
 *   read regs (tR)
 * - options: various chip options. They can partly be set to inform
 *   nand_scan about special functionality. See the defines for further
 *   explanation
 * Members with a "?" were not set in the merged testing-NAND branch,
 * so they are not set here either.
 */
void board_nand_init(struct nand_chip *nand)
{
	nand->hwcontrol = lf1000_hwcontrol;
	nand->dev_ready = lf1000_device_ready;
	nand->write_byte = lf1000_write_byte;
	nand->eccmode = NAND_ECC_SOFT;
	nand->chip_delay = NAND_BIG_DELAY_US;

	/* enable NAND_WP GPIO as an output pin */
#ifdef CPU_MF2530F
	REG32(NAND_WP_BASE+NAND_WP_OUT_ENB) |= (1<<NAND_WP_PIN);
#elif defined CPU_LF1000
	/* enable writing to Alive GPIO pads */
	REG32(LF1000_ALIVE_BASE+ALIVEPWRGATEREG) = 1;
#endif
	/* and start with write protection disabled */
	lf1000_NAND_wp(0);

	/* clear local function pointers for intercepted NAND commands */
	sys_erase = NULL;
	sys_write = NULL;
	sys_write_ecc = NULL;
	sys_write_oob = NULL;
}

void board_nand_select_device(struct nand_chip *nand, int chip)
{
	// make sure that this is a valid chip select
	if( chip < 0 || chip >= NAND_MAX_CHIPS )
		return;	// invalid chip number, don't change anything

	// check if NAND function interceptors are in use (not NULL), if not, 
	// re-direct NAND function (write/erase) to local function with
	// NAND WP control
	if( sys_erase == NULL )
	{	
		sys_erase = nand_info[chip].erase;
		nand_info[chip].erase = &lf1000_erase;
	}
	if( sys_write == NULL )
	{	
		sys_write = nand_info[chip].write;
		nand_info[chip].write = &lf1000_write;
	}
	if( sys_write_ecc == NULL )
	{	
		sys_write_ecc = nand_info[chip].write_ecc;
		nand_info[chip].write_ecc = &lf1000_write_ecc;
	}
	if( sys_write_oob == NULL )
	{	
		sys_write_oob = nand_info[chip].write_oob;
		nand_info[chip].write_oob = &lf1000_write_oob;
	}
}

#endif /* (CONFIG_COMMANDS & CFG_CMD_NAND) */
