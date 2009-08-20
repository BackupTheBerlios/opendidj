/*
 * (C) Copyright 2003
 * Texas Instruments <www.ti.com>
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Alex Zuepke <azu@sysgo.de>
 *
 * (C) Copyright 2002-2004
 * Gary Jennejohn, DENX Software Engineering, <gj@denx.de>
 *
 * (C) Copyright 2004
 * Philippe Robin, ARM Ltd. <philippe.robin@arm.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <arm926ejs.h>

/* any writes( long, short or byte) to timer module will produce EXCEPTION ???? */
//#define	WRONG_HARDWARE

#define TIMER_LOAD_VAL 0xffffffff

/* macro to read the 32 bit timer */
#ifndef	WRONG_HARDWARE
  #define INIT_READ_TIMER()	(*(volatile ulong  *)(CFG_TIMERBASE+ 8)) |= 0x40
#else
  #define INIT_READ_TIMER()
#endif
#define READ_TIMER()		(*(volatile ulong  *)(CFG_TIMERBASE+ 0))
#define	RUN_TIMER		0x08

static ulong timestamp;
static ulong lastdec;

/* nothing really to do with interrupts, just starts up a counter. */
/* NOTE: timer input clk is divided down by 256.  Timer will wrap about every
 * 90 minutes.
 * NOTE: the *_masked functions are not called and likely do not work!
 */
int timer_init (void)
{
// enable writes to timer module
	*(volatile ulong *)(CFG_TIMERBASE + 0x40)  = 0x0c;		/* TCLKMODE = CLKGENENB = 1 */
// start from known values
	*(volatile ulong *)(CFG_TIMERBASE + 0)  = 0;			/* TimerLoad */
	*(volatile ulong *)(CFG_TIMERBASE + 4)  = TIMER_LOAD_VAL;	/* TimerValue */
	*(volatile ulong *)(CFG_TIMERBASE + 0x44)  |= 0x0ff0; /* set divider to 256 */
// run timer
	*(volatile ulong *)(CFG_TIMERBASE + 0x08) |= RUN_TIMER;		/* RUN = 1 */

	reset_timer_masked();

	return 0;
}

/*
 * timer without interrupts
 */

void reset_timer (void)
{
	*(volatile ulong *)(CFG_TIMERBASE + 0)  = 0;
}

ulong get_timer (ulong base)
{
	INIT_READ_TIMER();
	return *(volatile ulong *)(CFG_TIMERBASE + 0) - base;
}

void set_timer (ulong t)
{
	*(volatile ulong *)(CFG_TIMERBASE + 0)  = t;
}

/* delay x useconds AND perserve advance timstamp value */
void udelay (unsigned long usec)
#ifndef	WRONG_HARDWARE
{
	ulong tmo, tmp, cur;
	if(usec >= 1000){		/* if "big" number, spread normalization to seconds */
		tmo = usec / 1000;	/* start to normalize for usec to ticks per sec */
		tmo *= CFG_HZ;		/* find number of "ticks" to wait to achieve target */
		tmo /= 1000;		/* finish normalize. */
	}else{				/* else small number, don't kill it prior to HZ multiply */
		tmo = usec * CFG_HZ;
		tmo /= (1000*1000);
	}

#ifdef	WRONG_HARDWARE
	tmp = get_timer (0);		/* get current timestamp */
	if( (tmo + tmp + 1) < tmp )	/* if setting this fordward will roll time stamp */
		reset_timer_masked ();	/* reset "advancing" timestamp to 0, set lastdec value */
	else
		tmo += tmp;		/* else, set advancing stamp wake up time */

	while (get_timer_masked () < tmo)/* loop till event */
		/*NOP*/;
#else
	INIT_READ_TIMER();
	tmp = READ_TIMER();		/* current tick value */

	do {
	    INIT_READ_TIMER();
	    cur = READ_TIMER() - tmp;
	} while( cur < tmo);
#endif
}
#else
{
// enabling timer to run 0xc0001808 |= 0x8 kills the system
    while( usec--);
}
#endif

void reset_timer_masked (void)
{
	/* reset time */
	INIT_READ_TIMER();
	lastdec = READ_TIMER();  /* capure current decrementer value time */
	timestamp = 0;	       /* start "advancing" time stamp from 0 */
}

ulong get_timer_masked (void)
{
    ulong now;

	INIT_READ_TIMER();
	now = READ_TIMER();		/* current tick value */

	if (lastdec >= now) {		/* normal mode (non roll) */
		/* normal mode */
		timestamp += lastdec - now; /* move stamp fordward with absoulte diff ticks */
	} else {			/* we have overflow of the count down timer */
		/* nts = ts + ld + (TLV - now)
		 * ts=old stamp, ld=time that passed before passing through -1
		 * (TLV-now) amount of time after passing though -1
		 * nts = new "advancing time stamp"...it could also roll and cause problems.
		 */
		timestamp += lastdec + TIMER_LOAD_VAL - now;
	}
	lastdec = now;

	return timestamp;
}

/* waits specified delay value and resets timestamp */
void udelay_masked (unsigned long usec)
{
	ulong tmo;
#ifdef	WRONG_HARDWARE
	ulong endtime;
	signed long diff;
#else
	ulong	tmp, cur;
#endif

	if (usec >= 1000) {		/* if "big" number, spread normalization to seconds */
		tmo = usec / 1000;	/* start to normalize for usec to ticks per sec */
		tmo *= CFG_HZ;		/* find number of "ticks" to wait to achieve target */
		tmo /= 1000;		/* finish normalize. */
	} else {			/* else small number, don't kill it prior to HZ multiply */
		tmo = usec * CFG_HZ;
		tmo /= (1000*1000);
	}

#ifdef	WRONG_HARDWARE
	endtime = get_timer_masked () + tmo;

	do {
		ulong now = get_timer_masked ();
		diff = endtime - now;
	} while (diff >= 0);
#else
	INIT_READ_TIMER();
	tmp = READ_TIMER();		/* current tick value */

	do {
	    INIT_READ_TIMER();
	    cur = READ_TIMER() - tmp;
	} while( cur < tmo);
#endif
}

/*
 * This function is derived from PowerPC code (read timebase as long long).
 * On ARM it just returns the timer value.
 */
unsigned long long get_ticks(void)
{
	return get_timer(0);
}

/*
 * This function is derived from PowerPC code (timebase clock frequency).
 * On ARM it returns the number of timer ticks per second.
 */
ulong get_tbclk (void)
{
	ulong tbclk;

	tbclk = CFG_HZ;
	return tbclk;
}
