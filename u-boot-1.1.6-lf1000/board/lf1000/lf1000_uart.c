#include "autoconf.h"
#include <asm/io.h>
#include <arch-lf1000/platform.h>
#include <arch-lf1000/common.h>
#include <arch-lf1000/gpio_hal.h>
#include <arch-lf1000/gpio.h>
#include <arch-lf1000/uart.h>

/* adjust for port number */
#define GPIOALTFN0      GPIOAALTFN0+0x40*LF1000_SYS_UART_PORT
#define GPIOALTFN1      GPIOAALTFN1+0x40*LF1000_SYS_UART_PORT

/* set pin function */
#define GPIO_SET_FUNC(pin, func)                                        \
        REG32(LF1000_GPIO_BASE+(pin>15?GPIOALTFN1:GPIOALTFN0)) =        \
                                        (func<<((pin>15?pin-16:pin)*2))

int serial_init(void)
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

	/* disable UART clock */
	REG32(LF1000_SYS_UART_BASE+UARTCLKENB) &= ~(1<<UARTCLKGENENB);

	/* clear IRQ pending, set 8 bit word length */
	REG16(LF1000_SYS_UART_BASE+LCON) = (1<<SYNC_PENDCLR)|(3<<WORD_LEN);

	/* enable polling/IRQ transmit and receive */
	REG16(LF1000_SYS_UART_BASE+UCON) = (1<<TRANS_MODE)|(1<<RECEIVE_MODE);

	/* reset the FIFOs */
	REG16(LF1000_SYS_UART_BASE+FCON)=(1<<TX_FIFO_RESET)|(1<<RX_FIFO_RESET);

	/* TODO: do we need this? */
	REG16(LF1000_SYS_UART_BASE+MCON) = (1<<SCRXENB);

	/* set the baud rate */
	REG16(LF1000_SYS_UART_BASE+BRD) = 1; /*XXX*/
	/*FIXME: what we want: UART_BRD(UART_PLL,LF1000_SYS_UART_BR);*/

	/* configure clock source */
	REG32(LF1000_SYS_UART_BASE+UARTCLKGEN) =
	       	((UART_PLL<<UARTCLKSRCSEL)|((UARTDIV-1)<<UARTCLKDIV));

	/* enable UART clock */
	REG32(LF1000_SYS_UART_BASE+UARTCLKENB) |= (1<<UARTCLKGENENB);

	return 0;
}

/*
 * Check if reciever has data
 */
int serial_tstc (void)
{
	return (REG16(LF1000_SYS_UART_BASE+FSTATUS) & 0xF);
}

int serial_getc (void)
{
	while(!serial_tstc());
	return REG16(LF1000_SYS_UART_BASE+RHB);
}


void serial_putc(const char ch)
{
	u16 status;

	if(ch == '\n')
		serial_putc('\r');

	while(1) { /* wait for transmitter to be ready */
		status = REG16(LF1000_SYS_UART_BASE+TRSTATUS);
		if(status & ((1<<TRANSMITTER_EMPTY)|(1<<TRANSMIT_BUFFER_EMPTY)))
			break;
	}
	/* transmit */
	REG8(LF1000_SYS_UART_BASE+THB) = ch;
}

void serial_puts ( const char *s)
{
	while(*s) {
		serial_putc(*s);
		s++;
	}
}

void serial_setbrg (void)
{
}
