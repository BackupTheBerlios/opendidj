/* arch/arm/mach-lf1000/lf1000_core_func.c
 *
 * Kosta Demirev <kdemirev@yahoo.com> 
 * Andrey Yurovsky <andrey@cozybit.com>
 * Scott Esters <sesters@leapfrog.com>
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <asm/io.h>
#include <asm/arch/platform.h>
#include <asm/arch/common.h>
#include <asm/arch/uart.h>
#include <asm/arch/timer.h>
#include <asm/arch/clkpwr.h>

//#define MAKE_PLL_LIST	// define when building a list of popular PLL freqs

#define	PRECISION	4096	// scale partial results to preserve precision
/*
 * macro calculating frequency out
 *    PRECISION       -- simulate fixed-point math
 *    MP()            -- frequency out
 *    m               -- main divider
 *    p               -- pre divider
 *    s               -- post scaler
 *    mpll            -- pll multiplier
 *    				=1 for LF1000 PLLs & MP2530 PLL2
 *				=2 for MP2530 PLL0 and PLL1
 */

#define	MP( m, p, s, mpll) ((PRECISION*mpll*m)/(p*(1<<s)))

/* macros to build PLL register */
#define PLL0_REG( m, p, s) \
	(((unsigned int)(m) << MDIV_0) | \
	 ((unsigned int)(p) << PDIV_0) | \
	 ((unsigned int)(s) << SDIV_0))

#define PLL1_REG( m, p, s) \
	(((unsigned int)(m) << MDIV_1) | \
	 ((unsigned int)(p) << PDIV_1) | \
	 ((unsigned int)(s) << SDIV_1))

#define PLL2_REG( m, p, s) \
	(((unsigned int)(m) << MDIV_2) | \
	 ((unsigned int)(p) << PDIV_2) | \
	 ((unsigned int)(s) << SDIV_2))

/*
 * get_pll_div()
 *
 * Determine PLL divisor register setting given the pll, frequency in hz,
 * and frequency out in hz.  The PLL is set by adjusting 3 variables, a
 * pre-divider 'p', a master dividor 'm', and a scaler 's'.
 * 
 * This routine first checks to see if the best PLL setting has already been
 * calculated previously, otherwise it loops through all possible settings to
 * find the best match.
 
 * There are two frequency calculations.  The first describes the LF1000 PLL0
 * and PLL1 operation, plus the MP2530 PLL2:
 *     PLL Frequency = ( m * frequency_in ) / ( p * 2^s )
 *
 * This second equation describes the MP2530 PLL0 and PLL1:
 *     PLL Frequency = ( 2 * m * frequency_in ) / ( p * 2^s )
 */

unsigned int __init get_pll_div( int pll, int fin_hz, int fout_hz)
{
	int fin_khz = fin_hz / 1000;
	int fout_khz = fout_hz / 1000;
	
	/*
	 * rewrite the frequency out equation as a ratio:
	 *   (frequency_out)/(frequency_in) = (mpll * m) / (p*(2^s))
	 */

	int fratio = (PRECISION * fout_khz / fin_khz);	// actual ratio

	// initialize, assuming first choice is the best
	int mdiv = MIN_MDIV;
	int pdiv = MIN_PDIV;
	int sdiv = SDIV_HZ(pll, fout_hz);		// select post-scaler
	int mpll = PLL_MUL(pll);			// pll multiplier (1/2)
	int mdiff = MP( pdiv, mdiv, sdiv, mpll );

	int m = 0;
	int p = 0;
	int diff = 0;

#ifndef MAKE_PLL_LIST
#ifdef CPU_LF1000
	/*
	 * Check popular PLL frequency settings before looping through all
	 * possible combinations of 'p' and 'm'. Return if found.
	 */

	if (fout_hz == 393216000 && fin_hz == 27000000)
		return(PLL0_REG(801, 55, 0));
	if (fout_hz == 147000000 && fin_hz == 27000000)
	       	return(PLL0_REG(196,  9, 2));
#endif
#endif

	/* Loop through all settings for 'p' and 'm' saving the best match */
	for(p=MIN_PDIV; p<(MAX_PDIV+1); p++) {
		for(m=MIN_MDIV; m<(MAX_MDIV+1); m++) {
			// see if this is a better setting
			if((diff = abs(fratio - MP(m,p,sdiv,mpll))) < mdiff) {
				mdiff = diff;
				mdiv = m;
				pdiv = p;
			}
		}
	}


#ifdef MAKE_PLL_LIST
	// uncomment when building up list of popular frequencies
	printk(KERN_INFO "%s   pll=%d, fin_hz=%d, fout_hz=%d\n", __FUNCTION__,
		pll, fin_hz, fout_hz);
	printk(KERN_INFO "%s   mdiv=%d, pdiv=%d, sdiv=%d\n", __FUNCTION__,
		mdiv, pdiv, sdiv);
#endif

	/* construct the PLL register setting */
#ifdef CPU_MF2530F
	if(pll == PLL2)
		return(PLL2_REG(mdiv,pdiv,sdiv));
#endif
	return(PLL0_REG(mdiv,pdiv,sdiv));
}

/* 
 * Returns PLL frequency in Hz, or -1 if an invalid PLL number is specified. 
 * On the LF1000, there are two PLLs, 0 and 1.  The formula for PLL Frequency
 * for them is:
 *
 *  F = (MDIV*CRYSTAL_FREQ_HZ)/(PDIV*(1<<SDIV))
 *
 *  Rewritten, to avoid overflowing 32 bits it is:
 *
 *  F = (((CRYSTAL_FREQ_HZ)/PDIV)/(1<<SDIV))*MDIV
 * 
 * On the MF2530F, there are three PLLs: 0, 1, and 2.  PLL 0 or 1 Frequency
 * is (2 * (MDIV + 8) * CRYSTAL_FREQ_HZ) / ((PDIV + 2) * (1<<SDIV))
 *
 * PLL2 Frequency is ((MDIV + 8) * CRYSTAL_FREQ_HZ) / ((PDIV + 2) * (1<<SDIV))
 */

int get_pll_freq(unsigned int pll)
{
	u32 tmp, p, m, s;
	
	if(pll >= NUM_PLLS)
		return -1;

	tmp = REG32(IO_ADDRESS(LF1000_CLKPWR_BASE+PLLSETREG0+pll*sizeof(u32)));

#ifdef CPU_MF2530F
	p = ((tmp>>PDIV_0) & BIT_MASK((24-PDIV_0))) + 2;
	m = ((tmp>>MDIV_0) & BIT_MASK((PDIV_0-MDIV_0))) + 8;
#else
	p = ((tmp>>PDIV_0) & BIT_MASK((24-PDIV_0)));
	m = ((tmp>>MDIV_0) & BIT_MASK((PDIV_0-MDIV_0)));
#endif
	s = ((tmp>>SDIV_0) & BIT_MASK((MDIV_0-SDIV_0)));

#ifdef CPU_MF2530F
	if(pll != 2)
		return ((((CRYSTAL_FREQ_HZ)/p)/(1<<s))*m*2);
#endif

	return ((((CRYSTAL_FREQ_HZ)/p)/(1<<s))*m);
}
EXPORT_SYMBOL(get_pll_freq);

/* returns freq in Hz */
unsigned int get_cpu_freq( void)
{
	return ((get_pll_freq((clock_p->clkmodereg >> 4) & 0x03))/
		((ioread32(&clock_p->clkmodereg) & 0x0f)+1));
}

void calc_freq_ratio(unsigned long * loops)
{
#define	SCALE	(10)

	*loops = ((*loops)*(get_cpu_freq()/(get_timer_freq(LF1000_FREE_TIMER)>>SCALE)))>>SCALE;
}
///////////////////////// Timer init ///////////////////////////////
static struct lf1000_timer_init init_lf1000_timers[] = {
	{
		(struct lf1000_timer*)IO_ADDRESS(LF1000_TIMER0_BASE), 
		LF1000_TIMER0_IRQ
	},
	{
		(struct lf1000_timer*)IO_ADDRESS(LF1000_TIMER1_BASE), 
		LF1000_TIMER1_IRQ
	},
	{
		(struct lf1000_timer*)IO_ADDRESS(LF1000_TIMER2_BASE), 
		LF1000_TIMER2_IRQ
	},
	{
		(struct lf1000_timer*)IO_ADDRESS(LF1000_TIMER3_BASE),
		LF1000_TIMER3_IRQ
	},
#if defined CPU_LF1000
	{
		(struct lf1000_timer*)IO_ADDRESS(LF1000_TIMER4_BASE), 
		LF1000_TIMER4_IRQ
	},
#endif
};

////////////////////// NON INIT functions ////////////////////////////
struct lf1000_timer* get_timer_pnt( int timer)
{
	if(timer >= ARRAY_SIZE(init_lf1000_timers)) 
		return NULL;
	return init_lf1000_timers[timer].timer_pnt;
}

/* returns timer freq in Hz */
int get_timer_freq( int timer)
{
	struct lf1000_timer* timer_p = get_timer_pnt( timer);
	unsigned int divider;

	if( timer >= ARRAY_SIZE( init_lf1000_timers)) return 0;

	// transform TC=div 0; TC/2=div 1; TC/4 = div 2; TC/8 = div 3
	
	/* get SELTCLK and transform it */
	divider = (1<<((ioread32(&timer_p->tmrcontrol)+1) & 0x03));
	/* get CLKDIV */
	divider *= ((ioread32(&timer_p->tmrclkgen)>>4) & 0xff)+1;

	return ((get_pll_freq((ioread32(&timer_p->tmrclkgen) > 1) & 0x03))/divider);
}

int get_timer_irq( int timer)
{
	if(timer >= ARRAY_SIZE( init_lf1000_timers)) 
		return 0;
	return init_lf1000_timers[timer].irq_number;
}

void clear_timer_irq( int irq)
{
int i;

	for( i=0; i<ARRAY_SIZE( init_lf1000_timers); i++)
	if( irq == init_lf1000_timers[i].irq_number) {
		iowrite32( ioread32(&init_lf1000_timers[i].timer_pnt->tmrcontrol) | (1<<INTPEND),
			  &init_lf1000_timers[i].timer_pnt->tmrcontrol);
	}
}

/*
 * get_timer_cnt() -- return number of ticks since last interrupt
 * The LF1000 timer is an up counter, but zeros automaticly when
 * the match value is reached.
 */

unsigned long get_timer_cnt( int timer)
{
struct lf1000_timer* timer_p = get_timer_pnt( timer);
unsigned int tmrcontrol, tmrcount;

	if (!timer_p) return -1;	// timer does not exist

	tmrcontrol = ioread32(&timer_p->tmrcontrol);
	tmrcontrol &= ~(1<<INTPEND);	// do not clear pend
	tmrcontrol |=  (1<<LDCNT);	// latch count
	iowrite32( tmrcontrol, &timer_p->tmrcontrol);
	tmrcount = ioread32(&timer_p->tmrcount);
	return(tmrcount);
}

int read_current_timer(unsigned long *timer_value)
{
	*timer_value = get_timer_cnt( LF1000_FREE_TIMER);
	return 0;
}

unsigned long get_timer_match( int timer)
{
struct lf1000_timer* timer_p = get_timer_pnt( timer);

	return ioread32(&timer_p->tmrmatch);
}

///////////////////////// UART init ///////////////////////////////

static struct lf1000_uart_init __initdata init_lf1000_uarts[] = {
	{
		(struct lf1000_uart*)IO_ADDRESS(LF1000_UART0_BASE), 
		LF1000_UART0_IRQ
	},
	{
		(struct lf1000_uart*)IO_ADDRESS(LF1000_UART1_BASE), 
		LF1000_UART1_IRQ
	},
	{ 
		(struct lf1000_uart*)IO_ADDRESS(LF1000_UART2_BASE), 
		LF1000_UART2_IRQ
	},
	{ 
		(struct lf1000_uart*)IO_ADDRESS(LF1000_UART3_BASE), 
		LF1000_UART3_IRQ
	},
#if defined CPU_MF2530F
	{
		(struct lf1000_uart*)IO_ADDRESS(LF1000_UART4_BASE), 
		LF1000_UART4_IRQ
	},
	{ 
		(struct lf1000_uart*)IO_ADDRESS(LF1000_UART5_BASE), 
		LF1000_UART5_IRQ
	},
#endif
};

static struct lf1000_uart* __init get_uart_pnt( int uart)
{
	if(uart >= ARRAY_SIZE(init_lf1000_uarts)) return NULL;
	return init_lf1000_uarts[uart].uart_pnt;
}

void __init set_uart_baud( int uart, int baud)
{ 

	struct lf1000_uart* uart_p = get_uart_pnt( uart);
	iowrite32((UART_BRD( UART_PLL, baud)<<4)+(UART_PLL<<1), 
			&uart_p->uartclkgen);
	iowrite16(1, &uart_p->brd);
}
