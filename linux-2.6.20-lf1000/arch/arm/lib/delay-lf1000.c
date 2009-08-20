/*
 *      Precise Delay Loops for lf1000
 *
 *      Copyright (C) 1993 Linus Torvalds
 *      Copyright (C) 2007 Kosta Demirev <kdemirev@yahoo.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <asm/arch/platform.h>

#define	TIMER		LF1000_FREE_TIMER
#define	SCALE		20

static unsigned int ratio, timer_freq;

void __delay( int loops)
{
int bclock, now;

    bclock = get_timer_cnt( TIMER);
    do {
	now = get_timer_cnt( TIMER);
    } while ((now - bclock) < loops);
}

inline void __const_udelay(unsigned long xloops)
{
// in asm/delay.h is used constant mul factor "((2199023U*HZ)>>11)"
// to get correct delays I(Kosta) must do next conversion
// for LF1000 timer settings
unsigned int freq;

    if( (freq = get_timer_freq( TIMER)) != timer_freq) {
	timer_freq = freq;
	ratio = ((timer_freq/((2199023U*HZ)>>11))<<SCALE)/1000000;
    }

    __delay( (xloops*ratio)>>SCALE);
}

void __udelay(unsigned long usecs)
{
    __const_udelay( usecs*((2199023U*HZ)>>11));	/* usecs */
}
