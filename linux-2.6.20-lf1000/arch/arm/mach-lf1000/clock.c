/*
 *  linux/arch/arm/mach-lf1000/clock.c
 *
 *	  Copyright (C) 2007 Kosta Demirev <kdemirev@yahoo.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <asm/io.h>

#include <asm/semaphore.h>
#include <asm/hardware/lf1000.h>

#include <asm/arch/platform.h>
#include <asm/arch/clkpwr.h>

#include "clock.h"

static LIST_HEAD(clocks);
static DEFINE_MUTEX(clocks_mutex);

static void lf1000_oscvco_set(struct clk *clk)
{
	writel( get_pll_div( clk->number, clk->params->ref, clk->rate),
		(&(clock_p->pllsetreg0) + clk->number));
}

struct clk *clk_get(struct device *dev, const char *id)
{
	struct clk *p, *clk = ERR_PTR(-ENOENT);

	mutex_lock(&clocks_mutex);
	list_for_each_entry(p, &clocks, node) {
		if (strcmp(id, p->name) == 0 && try_module_get(p->owner)) {
			clk = p;
			break;
		}
	}
	mutex_unlock(&clocks_mutex);

	return clk;
}
EXPORT_SYMBOL(clk_get);

void clk_put(struct clk *clk)
{
	module_put(clk->owner);
}
EXPORT_SYMBOL(clk_put);

int clk_enable(struct clk *clk)
{
	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_disable);

/* clk_get_rate() - obtain the current clock rate in KHz */
unsigned long clk_get_rate(struct clk *clk)
{
	return (get_pll_freq(clk->number)/1000);
}
EXPORT_SYMBOL(clk_get_rate);

/* clk_set_rate() - set clock rate provided in KHz */
int clk_set_rate(struct clk *clk, unsigned long rate)
{
	clk->rate = rate * 1000;
	clk->setvco(clk);
	return 0;
}
EXPORT_SYMBOL(clk_set_rate);

static const struct lf1000_params lf1000_pll0_params = {
	.ref		= CRYSTAL_FREQ_HZ,
	.vco_max	= MAX_VCOF_HZ(PLL0),
	.md_min		= MIN_MDIV,
	.md_max		= MAX_MDIV,
	.pd_min		= MIN_PDIV,
	.pd_max		= MAX_PDIV,
};

static struct clk pll0_clk = {
	.name		= "PLL0 CLK",
	.rate		= PLL0_OUT_HZ,
	.number		= PLL0,
	.params		= &lf1000_pll0_params,
	.setvco		= lf1000_oscvco_set,
};

static const struct lf1000_params lf1000_pll1_params = {
	.ref		= CRYSTAL_FREQ_HZ,
	.vco_max	= MAX_VCOF_HZ(PLL1),
	.md_min		= MIN_MDIV,
	.md_max		= MAX_MDIV,
	.pd_min		= MIN_PDIV,
	.pd_max		= MAX_PDIV,
};

static struct clk pll1_clk = {
	.name		= "PLL1 CLK",
	.rate		= PLL1_OUT_HZ,
	.number		= PLL1,
	.params		= &lf1000_pll1_params,
	.setvco 	= lf1000_oscvco_set,
};

#if	defined	CPU_MF2530F
static const struct lf1000_params lf1000_pll2_params = {
	.ref		= CRYSTAL_FREQ_HZ,
	.vco_max	= MAX_VCOF_HZ(PLL2),
	.md_min		= MIN_MDIV,
	.md_max		= MAX_MDIV,
	.pd_min		= MIN_PDIV,
	.pd_max		= MAX_PDIV,
};

static struct clk pll2_clk = {
	.name		= "PLL2 CLK",
	.rate		= PLL2_OUT_HZ,
	.number		= PLL2,
	.params		= &lf1000_pll2_params,
	.setvco 	= lf1000_oscvco_set,
};
#endif

int clk_register(struct clk *clk)
{
	mutex_lock(&clocks_mutex);
	list_add(&clk->node, &clocks);
	mutex_unlock(&clocks_mutex);
	clk->setvco( clk);

	printk("PLL%d=%3d.%03d MHz   ", (int)clk->number,
					(int)get_pll_freq(clk->number)/1000000,
					(int)get_pll_freq(clk->number)%1000000);

	return 0;
}
EXPORT_SYMBOL(clk_register);

void clk_unregister(struct clk *clk)
{
	mutex_lock(&clocks_mutex);
	list_del(&clk->node);
	mutex_unlock(&clocks_mutex);
}
EXPORT_SYMBOL(clk_unregister);

/* generate a suggested divider value from a PLL rate */
int lf1000_CalcDivider(unsigned int pll_hz, unsigned int desired_hz)
{
	int div, rem;

	if(pll_hz == 0 || desired_hz == 0 || desired_hz > pll_hz)
		return -1;

	div = pll_hz/desired_hz;
	rem = pll_hz%desired_hz;

	/*
	 * if 'rem' is greater than (desired_hz / 2) round up
	 * as 'div+1' is a more accurate divisor
	 */
	if ((desired_hz / 2) < rem)
		div++;
	
	return(div);
}
EXPORT_SYMBOL(lf1000_CalcDivider);

static struct clk * plla[] __initdata = {
	&pll0_clk,
	&pll1_clk,
#ifdef	CPU_MF2530F
	&pll2_clk,
#endif
};

// must be initialized very early in boot
//static int __init clk_init(void)
int __init clk_init(void)
{
	u32 tmp, tmp_original;
	int pll;

	/* register clock sources */
	for(pll = 0; pll < ARRAY_SIZE(plla); pll++)
		clk_register(plla[pll]);

	tmp = readl(&clock_p->clkmodereg);
	tmp_original = tmp;	// hang onto, check for changes

#ifdef CPU_MF2530F
	/* SDRAM selects SDRAM_PLL = PLL1 (Syncronous Dynamic RAM clock),
	 * SDRAM clk div = full throttle */
	tmp &= ~((0x3<<CLKSELSDRAM)|(0xF<<CLKDIVSDRAM));
	tmp |= ((SDRAM_PLL<<CLKSELSDRAM)|(0<<CLKDIVSDRAM));
	
	/* set CPU_PLL */
	tmp &= ~(0x3<<CLKSELCPU0);
	tmp |= (CPU_PLL<<CLKSELCPU0);

	/* BCLK & PCLK selects BCLK_PLL ( bus & peripherals clock) */
	tmp &= ~(0x3<<CLKSELBCLK);
	tmp |= (BCLK_PLL<<CLKSELBCLK);

	/* BCLK divider (if needed) */
#ifdef BCLK_DIV
	tmp &= ~(0xF<<CLKDIV1BCLK);
	tmp |= ((BCLK_DIV)<<CLKDIV1BCLK);
#endif

#elif defined CPU_LF1000
	//FIXME: check if the statement below is true
	/* (on LF1000, the SDRAM clock is fully synchronized so there are no
	 *  settings to be done) */
	tmp &= ~((0x3<<CLKSELCPU0) |	/*strip out bits */
		 (0xF<<CLKDIVCPU0) |
		 (0xF<<CLKDIV2CPU0)|
		 (0x3<<CLKSELBCLK) |
		 (0xF<<CLKDIV1BCLK));
	tmp |= ( (CPU_PLL    <<CLKSELCPU0)  |
		((CPU_DIV-1) <<CLKDIVCPU0)  |
		((HCLK_DIV-1)<<CLKDIV2CPU0) |
		( BCLK_PLL   <<CLKSELBCLK)  |
		((BCLK_DIV-1)<<CLKDIV1BCLK));
#endif
	/* apply, if needed */
	if (tmp != tmp_original) {
		writel(tmp, &clock_p->clkmodereg);

		/* set CHGPLL to tell hardware to apply settings, restabalize PLLs */
		writel(readl(&clock_p->pwrmode)|(1<<CHGPLL), &clock_p->pwrmode);
	
		/* wait for PLLs to stabalize */
		while(readl(&clock_p->pwrmode) & (1<<CHGPLL)) mb();
	}

	return 0;
}
