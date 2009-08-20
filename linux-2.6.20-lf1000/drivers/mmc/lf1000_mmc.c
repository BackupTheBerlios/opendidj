/* LF1000 SD/MMC (SDIO) Driver
 *
 * lf1000_mmc.c -- Main driver functionality.
 *
 * Note: this device supports MMC 4.2 (not HSMMC), SD 2.0, and SDIO 1.10.  
 *
 * Andrey Yurovsky <andrey@cozybit.com>
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/sysfs.h>
#include <linux/mmc/protocol.h>
#include <linux/mmc/host.h>
#include <asm/io.h>

#include <asm/arch/platform.h>
#include <asm/arch/common.h>
#include <asm/arch/gpio.h>

#include "lf1000_mmc.h"

#define SDIO_READ(r)		ioread32(sdio.mem+r)
#define SDIO_WRITE(x, r)	iowrite32(x, sdio.mem+r)
#define PCLK_HZ			(get_pll_freq(PCLK_PLL)/2)

/*
 * Driver Configuration:
 * - The LF1000 has channels 0 and 1, the MP2530F has only channel 0.
 * - The controller's maximum clock rate is 50MHz.
 * - A 400KHz clock is needed for initial scanning of a card.
 */
#define SDIO_CHANNEL		CONFIG_MMC_LF1000_CHANNEL
#define DESIRED_CLOCK_HZ	25000000
#define DESIRED_SLOW_CLOCK_HZ	400000
#define CD_INTERRUPT		GPIO_PORT_A, 19

/*
 * Device private data
 */
struct lf1000_host {
	struct mmc_host *mmc;
	struct mmc_command *cmd;
	struct mmc_request *request;

	void __iomem *mem;
	int irq;
	int div;
};

static struct lf1000_host sdio = {
	.mmc		= NULL,
	.cmd		= NULL,
	.request	= NULL,
	.mem		= NULL,
	.irq		= -1,
	.div		= -1,
};

/*
 * sysfs interface
 */

#ifdef CONFIG_MMC_LF1000_DEBUG
#define PRINT_REG(reg)	len += sprintf(buf+len, #reg "		= 0x%08X\n", \
				SDIO_READ(SDI_##reg));
static ssize_t show_registers(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t len = 0;

	PRINT_REG(CTRL);
	PRINT_REG(PWREN);
	PRINT_REG(CLKDIV);
	PRINT_REG(CLKENA);
	PRINT_REG(TMOUT);
	PRINT_REG(CTYPE);
	PRINT_REG(BLKSIZ);
	PRINT_REG(BYTCNT);
	PRINT_REG(INTMASK);
	PRINT_REG(CMD);
	PRINT_REG(RESP0);
	PRINT_REG(RESP1);
	PRINT_REG(RESP2);
	PRINT_REG(RESP3);
	PRINT_REG(MINTSTS);
	PRINT_REG(RINTSTS);
	PRINT_REG(STATUS);
	PRINT_REG(FIFOTH);
	PRINT_REG(TCBCNT);
	PRINT_REG(TBBCNT);
	PRINT_REG(DEBNCE);
	PRINT_REG(DAT);
	PRINT_REG(CICTRL);
	PRINT_REG(SYSCLKENB);
	PRINT_REG(CLKGEN);

	return len;
}
static DEVICE_ATTR(registers, S_IRUSR|S_IRGRP, show_registers, NULL);
#endif /* CONFIG_MMC_LF1000_DEBUG */

static ssize_t show_sdiostate(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t len = 0;
	u32 status = SDIO_READ(SDI_STATUS);

	len += sprintf(buf+len, "card:		%spresent\n", 
			gpio_get_val(GPIO_PORT_A, 19) ? "not " : "");
	len += sprintf(buf+len, "FSM state:	%d\n",
			(status>>CMDFSM) & 0xF);

	return len;
}
static DEVICE_ATTR(sdiostate, S_IRUSR|S_IRGRP, show_sdiostate, NULL);

static struct attribute *sdio_attributes[] = {
	&dev_attr_sdiostate.attr,
#ifdef CONFIG_MMC_LF1000_DEBUG
	&dev_attr_registers.attr,
#endif
	NULL
};

static struct attribute_group sdio_attr_group = {
	.attrs = sdio_attributes
};

/*
 * Interrupt Handling
 */

static irqreturn_t sdio_irq(int irq, void *dev_id)
{
	return IRQ_NONE;
}

static irqreturn_t handle_card_detect(enum gpio_port port, enum gpio_pin pin,
						void *priv)
{
	gpio_toggle_int_mode(port, pin);
	gpio_clear_pend(port, pin);
	gpio_set_int(port, pin, 0);

	return IRQ_HANDLED;
}

/*
 * Platform Device
 */

static int lf1000_sdio_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	int i;
	u32 tmp;

	printk(KERN_INFO "lf1000-sdio driver\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, SDIO_CHANNEL);
	if(!res) {
		printk(KERN_ERR "sdio: failed to get resource\n");
		return -ENXIO;
	}

	if(!request_mem_region(res->start, (res->end - res->start)+1,
				"lf1000-sdio")) {
		printk(KERN_ERR "sdio: failed to map region\n");
		return -EBUSY;
	}

	sdio.div = lf1000_CalcDivider(PCLK_HZ, DESIRED_CLOCK_HZ);
	if(sdio.div < 0) {
		printk(KERN_ERR "sdio: failed to get a clock divider\n");
		return -EFAULT;
	}
	if(sdio.div >= 64) {
		printk(KERN_INFO "sdio: divider too high, using 64\n");
		sdio.div = 63;
	}

	sdio.mem = ioremap_nocache(res->start, (res->end - res->start)+1);
	if(sdio.mem == NULL) {
		printk(KERN_ERR "sdio: failed to ioremap\n");
		ret = -ENOMEM;
		goto fail_remap;
	}

	i = platform_get_irq(pdev, SDIO_CHANNEL);
	if(i < 0) {
		ret = i;
		printk(KERN_ERR "sdio: failed to get IRQ number\n");
		goto fail_irq;
	}
	ret = request_irq(i, sdio_irq, SA_INTERRUPT|SA_SAMPLE_RANDOM, 
			"sdio", NULL);
	if(ret) {
		printk(KERN_ERR "sdio: requesting IRQ failed\n");
		goto fail_irq;
	}
	sdio.irq = i;

	/*
	 * Configure Pins
	 */

	/* nCD */
	gpio_configure_pin(CD_INTERRUPT, GPIO_GPIOFN, 0, 0, 0);
	/* nWP */
	gpio_configure_pin(GPIO_PORT_A, 20, GPIO_GPIOFN, 1, 0, 0);
#if SDIO_CHANNEL == 0
	for(i = 2; i <= 5; i++) /* SSDAT0 */
		gpio_configure_pin(GPIO_PORT_B, i, GPIO_ALT1, 1, 0, 0);
	/* SDCLK0 */
	gpio_configure_pin(GPIO_PORT_B, 0, GPIO_ALT1, 1, 0, 0);
	/* SDCMD0 */
	gpio_configure_pin(GPIO_PORT_B, 1, GPIO_ALT1, 1, 0, 0);
#else
	for(i = 8; i <= 11; i++) /* SSDAT1 */
		gpio_configure_pin(GPIO_PORT_B, i, GPIO_ALT1, 1, 0, 0);
	/* SDCLK1 */
	gpio_configure_pin(GPIO_PORT_B, 6, GPIO_ALT1, 1, 0, 0);
	/* SDCMD1 */
	gpio_configure_pin(GPIO_PORT_B, 7, GPIO_ALT1, 1, 0, 0);
#endif

	/*
	 * Set up the clock
	 */

	SDIO_WRITE((sdio.div<<CLKDIV0), SDI_CLKGEN);
	SDIO_WRITE((1<<PCLKMODE)|(1<<CLKGENENB), SDI_SYSCLKENB);

	/*
	 * Reset the hardware
	 */

	/* DMA */
	tmp = SDIO_READ(SDI_CTRL);
	tmp &= ~((1<<FIFORST)|(1<<CTRLRST));
	tmp |= (1<<DMARST);
	SDIO_WRITE(tmp, SDI_CTRL);
	while(IS_SET(SDIO_READ(SDI_CTRL), DMARST)); /*XXX*/

	/* FIFO */
	tmp = SDIO_READ(SDI_CTRL);
	tmp &= ~((1<<DMARST)|(1<<CTRLRST));
	tmp |= (1<<FIFORST);
	SDIO_WRITE(tmp, SDI_CTRL);
	while(IS_SET(SDIO_READ(SDI_CTRL), FIFORST)); /*XXX*/

	/* Controller */
	tmp = SDIO_READ(SDI_CTRL);
	tmp &= ~((1<<DMARST)|(1<<FIFORST));
	tmp |= (1<<CTRLRST);
	SDIO_WRITE(tmp, SDI_CTRL);
	while(IS_SET(SDIO_READ(SDI_CTRL), CTRLRST)); /*XXX*/

	/* turn off DMA */
	tmp = SDIO_READ(SDI_CTRL);
	BIT_CLR(tmp, DMA_ENA);
	SDIO_WRITE(tmp, SDI_CTRL);

	/* enable clock low power mode */
	tmp = SDIO_READ(SDI_SYSCLKENB);
	BIT_SET(tmp, LOWPWR);
	SDIO_WRITE(tmp, SDI_SYSCLKENB);

	/* set host data and response timeouts */
	SDIO_WRITE((0xFFFFFF<<DTMOUT)|(0x64<<RSPTMOUT), SDI_TMOUT);

	/* use 4-bit bus */
	SDIO_WRITE((1<<WIDTH), SDI_CTYPE);

	/* use 512 byte blocks */
	SDIO_WRITE(512, SDI_BLKSIZ);

	/* configure FIFOs */
	tmp = SDIO_READ(SDI_FIFOTH);
	tmp &= ~((0xFFF<<RXTH)|(0xFFF<<TXTH));
	tmp |= ((7<<RXTH)|(8<<TXTH));
	SDIO_WRITE(tmp, SDI_FIFOTH);

	/* disable interrupts, clear any pending */
	SDIO_WRITE(0, SDI_INTMASK);
	SDIO_WRITE(0xFFFFFFFF, SDI_RINTSTS);

	/* TODO: set up DMA */

	/* set up IRQ for card detection */
	ret = gpio_request_irq(CD_INTERRUPT, handle_card_detect, NULL);
	if(ret < 0)
		printk(KERN_ERR "sdio: failed to register CD interrupt\n");
	gpio_set_int_mode(CD_INTERRUPT, GPIO_IMODE_LOW_LEVEL);
	gpio_set_int(CD_INTERRUPT, 1);

	sysfs_create_group(&pdev->dev.kobj, &sdio_attr_group);

	return 0;

fail_irq:
	iounmap(sdio.mem);
fail_remap:
	release_mem_region(res->start, (res->end - res->start) + 1);
	return ret;
}

static int lf1000_sdio_remove(struct platform_device *pdev)
{
	struct resource *res  = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	gpio_set_int(CD_INTERRUPT, 0);
	gpio_free_irq(CD_INTERRUPT, handle_card_detect);

	sysfs_remove_group(&pdev->dev.kobj, &sdio_attr_group);

	/* turn off the clock */
	SDIO_WRITE((0<<PCLKMODE)|(0<<CLKGENENB), SDI_SYSCLKENB);

	/* disable interrupts, clear any pending */
	SDIO_WRITE(0, SDI_INTMASK);
	SDIO_WRITE(0xFFFFFFFF, SDI_RINTSTS);

	if(sdio.irq != -1)
		free_irq(sdio.irq, NULL);
	if(sdio.mem != NULL)
		iounmap(sdio.mem);
	release_mem_region(res->start, (res->end - res->start) + 1);
	return 0;
}

static struct platform_driver lf1000_sdio_driver = {
	.probe		= lf1000_sdio_probe,
	.remove		= lf1000_sdio_remove,
	.driver		= {
		.name	= "lf1000-sdio",
		.owner	= THIS_MODULE,
	},
};

/*
 * Module stuff
 */

static int __init sdio_init(void)
{
	return platform_driver_register(&lf1000_sdio_driver);
}

static void __exit sdio_cleanup(void)
{
	platform_driver_unregister(&lf1000_sdio_driver);
}

module_init(sdio_init);
module_exit(sdio_cleanup);
MODULE_AUTHOR("Andrey Yurovsky");
MODULE_VERSION("1:1.0");
MODULE_LICENSE("GPL");
