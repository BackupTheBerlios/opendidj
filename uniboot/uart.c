#include "autoconf.h"
#include <platform.h>
#include <common.h>
#include <uart.h>
#include <gpio.h>
#include <gpio_hal.h>

#include "pll.h"
#include "uart.h"

void uart_init(void)
{
	u32 tmp;

	/* GPIO setup, as needed */

	#ifdef LF1000_SYS_UART_TX_PORT
	tmp = REG32(LF1000_GPIO_BASE+LF1000_SYS_UART_TX_PORT);
	tmp &= ~(0x3<<(LF1000_SYS_UART_TX_PIN));
	tmp |= (LF1000_SYS_UART_TX_MODE<<(LF1000_SYS_UART_TX_PIN));
	REG32(LF1000_GPIO_BASE+LF1000_SYS_UART_TX_PORT) = tmp;
	#endif

	#ifdef LF1000_SYS_UART_RX_PORT
	tmp = REG32(LF1000_GPIO_BASE+LF1000_SYS_UART_RX_PORT);
	tmp &= ~(0x3<<(LF1000_SYS_UART_RX_PIN));
	tmp |= (LF1000_SYS_UART_RX_MODE<<(LF1000_SYS_UART_RX_PIN));
	REG32(LF1000_GPIO_BASE+LF1000_SYS_UART_RX_PORT) = tmp;
	#endif

	/* clear IRQ pending, set 8 bit word length */
	REG16(LF1000_SYS_UART_BASE+LCON) = (1<<SYNC_PENDCLR)|(3<<WORD_LEN);

	/* enable polling/IRQ transmit, leave receive disabled */
	REG16(LF1000_SYS_UART_BASE+UCON) = (1<<TRANS_MODE)|(1<<RECEIVE_MODE);

	/* set the baud rate */
	//REG16(LF1000_SYS_UART_BASE+BRD) = (UARTSRCCLK/(LF1000_SYS_UART_BR*16))-1;
	REG16(LF1000_SYS_UART_BASE+BRD) = 1; /* FIXME */

	REG16(LF1000_SYS_UART_BASE+UARTCLKGEN) = 
		((UARTDIV-1)<<UARTCLKDIV)|(UART_PLL<<UARTCLKSRCSEL);
}

#ifdef DEBUG
void uart_putchar(char c)
{
	while(!(REG16(LF1000_SYS_UART_BASE+TRSTATUS) & (1<<TRANSMIT_BUFFER_EMPTY))); /* wait for TX empty*/
	REG8(LF1000_SYS_UART_BASE+THB) = c;
}

void uart_puts(char *s)
{
	/* Kosta: saves 4 bytes compared to while() construction */
	do {
		uart_putchar(*s++);
	} while(*s != 0);
}

void uart_hex( unsigned int val)
{
	val= val & 0xf;
	if (val <10) 	uart_putchar('0'+val);
	else		uart_putchar('a'+(val-10));
}

void uart_int( unsigned int val)
{
	uart_puts("\n0x");
	uart_hex(val>>(4*7));
	uart_hex(val>>(4*6));
	uart_hex(val>>(4*5));
	uart_hex(val>>(4*4));
	uart_hex(val>>(4*3));
	uart_hex(val>>(4*2));
	uart_hex(val>>(4*1));
	uart_hex(val>>(4*0));
}
#endif
