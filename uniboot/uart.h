#ifndef UART_H
#define UART_H

/* UART settings */

#define PLL2FREQ(FIN, PDIV, MDIV, SDIV) \
	        ((long long)((long long)(MDIV+8) * (long long)FIN)/((PDIV+2) * (1UL<<SDIV)))
#define UARTSRCCLK		\
		(PLL2FREQ(CRYSTAL_FREQ*1000, PDIV_UNI, MDIV_UNI, SDIV_UNI)/UARTDIV)

void uart_init(void);
#ifdef DEBUG
void uart_putchar(char c);
void uart_puts(char *s);
void uart_hex( unsigned int val);
void uart_int( unsigned int val);
#else
#define uart_putchar(...)
#define uart_puts(...)
#define uart_hex(...)
#define uart_int(...)
#endif

#endif
