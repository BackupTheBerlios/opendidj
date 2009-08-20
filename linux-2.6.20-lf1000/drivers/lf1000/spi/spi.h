/* LF1000 SPI Driver
 *
 * spi.h -- SPI control.
 *
 * Scott Esters
 * LeapFrog Enterprises
 *
 * Andrey Yurovsky <andrey@cozybit.com>
 */

#ifndef _LF1000_SPI_H
#define _LF1000_SPI_H

#include <linux/module.h>
#include <linux/types.h>
#include <linux/ioport.h>
#include <linux/proc_fs.h>
#include <asm/io.h>

/* clock settings */
#define SPI_SRC_HZ      10000000
#define SPI_CLK_SRC     PLL1

/* IO pin control 
 * FIXME: set based on channel, using CONFIG_LF1000_SPI_CHANNEL */
#ifdef CPU_MF2530F
#define SPI_GPIO_PORT   GPIO_PORT_C
#elif defined CPU_LF1000
#define SPI_GPIO_PORT	GPIO_PORT_B
#endif
#define SPI_MOSI_PIN	15
#define SPI_MOSI_FN	GPIO_ALT1
#define SPI_MISO_PIN	14
#define SPI_MISO_FN	GPIO_ALT1
#define SPI_SCLK_PIN	13
#define SPI_SCLK_FN	GPIO_ALT1
#define SPI_SSPFRM_PIN  12
#define SPI_SSPFRM_FN	GPIO_GPIOFN

struct spi_device {
        void *mem;
	int irq;				// assigned IRQ number
	spinlock_t spi_spinlock;		// serialize spi bus access
	int isBusy;				// 1 = spi bus is busy
	wait_queue_head_t wqueue;		// waiting for spi bus lock
	wait_queue_head_t intqueue;		// waiting for an interrupt
#ifdef CONFIG_PROC_FS
        struct proc_dir_entry *proc_port;
#endif
};

#endif
