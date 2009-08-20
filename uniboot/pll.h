#ifndef PLL_H
#define PLL_H

#define clock_phy_p		((struct lf1000_clock*)( LF1000_CLOCK_BASE))

#if	1
// for 340 MHz 
  #define	PDIV_UNI	 25
  #define	MDIV_UNI	162
  #define	SDIV_UNI	  0
#else
// for default 199 MHz
  #define	PDIV_UNI	  7
  #define	MDIV_UNI	 56
  #define	SDIV_UNI	  1
#endif

#endif
