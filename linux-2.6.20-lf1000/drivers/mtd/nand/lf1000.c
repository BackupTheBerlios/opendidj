/*
 *  drivers/mtd/nand/lf1000.c
 *
 * Copyright 2007 LeapFrog Enterprises Inc.
 *
 * Kosta Demirev <kdemirev@yahoo.com>
 * Andrey Yurovsky <andrey@cozybit.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <asm/arch/platform.h>
#include <asm/arch/common.h>
#include <asm/arch/nand.h>
#include <asm/io.h>
#include <asm/sizes.h>

#if defined CPU_LF1000 && defined CONFIG_MTD_NAND_LF1000_HWECC
void lf1000_enable_hwecc(struct mtd_info *mtd, int mode);
int lf1000_calculate_ecc(struct mtd_info *mtd, const uint8_t *dat,
				uint8_t *ecc_code);
int lf1000_correct_ecc(struct mtd_info *mtd, uint8_t *dat, uint8_t *read_ecc,
				uint8_t *calc_ecc);
#endif

/* control registers */
#define NAND_BASE	IO_ADDRESS(LF1000_MCU_S_BASE)

/*
 * Private device structure
 */
struct lf1000_nand_device {
	void *mem;	/* NAND controller IO memory */
	struct mtd_info *mtd_onboard;
	struct mtd_info *mtd_cart;

	struct nand_hw_control controller;
};

static struct lf1000_nand_device nand = {
	.mem = NULL,
	.mtd_onboard = NULL,
	.mtd_cart = NULL,
};

/*
 * Define partitions for flash devices
 */

#if defined (CONFIG_MACH_ME_MP2530F)
#define LF_ERASE_BLK 0x4000
#else
/* All other boards have big erase blocks */
#define LF_ERASE_BLK 0x20000
#endif

/* Just shortening the names for clearer code */
#define LF_P0 (CONFIG_NAND_LF1000_P0_SIZE)
#define LF_P1 (CONFIG_NAND_LF1000_P1_SIZE)
#define LF_P2 (CONFIG_NAND_LF1000_P2_SIZE)
#define LF_P3 (CONFIG_NAND_LF1000_P3_SIZE)
#define LF_P4 (CONFIG_NAND_LF1000_P4_SIZE)
#define LF_P5 (CONFIG_NAND_LF1000_P5_SIZE)
#define LF_P6 (CONFIG_NAND_LF1000_P6_SIZE)
#define LF_P7 (CONFIG_NAND_LF1000_P7_SIZE)

#if ((LF_P0 % LF_ERASE_BLK) || (LF_P1 % LF_ERASE_BLK) || \
	(LF_P2 % LF_ERASE_BLK) || (LF_P3 % LF_ERASE_BLK) || \
	(LF_P4 % LF_ERASE_BLK) || (LF_P5 % LF_ERASE_BLK) || \
	(LF_P6 % LF_ERASE_BLK) || (LF_P7 % LF_ERASE_BLK))
#error "NAND partitions must be multiple of erase block."
#endif

static struct mtd_partition partition_info[] = {
  	{ .name		= "LF1000_uniboot",
  	  .offset	= 0,
 	  .size		= LF_P0},
  	{ .name		= "Atomic_Boot_Flags",
 	  .offset	= LF_P0,
 	  .size		= LF_P1 },
  	{ .name		= "Manufacturing_Data",
 	  .offset	= LF_P0 + LF_P1,
 	  .size		= LF_P2},
  	{ .name		= "Kernel0",
 	  .offset	= LF_P0 + LF_P1 + LF_P2,
 	  .size		= LF_P3 },
  	{ .name		= "Linux_RFS0",
 	  .offset	= LF_P0 + LF_P1 + LF_P2 + LF_P3,
 	  .size		= LF_P4 },
  	{ .name		= "Kernel1",
 	  .offset	= LF_P0 + LF_P1 + LF_P2 + LF_P3 + LF_P4,
 	  .size		= LF_P5 },
  	{ .name		= "Linux_RFS1",
 	  .offset	= LF_P0 + LF_P1 + LF_P2 + LF_P3 + LF_P4 + LF_P5,
 	  .size		= LF_P6 },
  	{ .name		= "Brio",
 	  .offset	= LF_P0 + LF_P1 + LF_P2 + LF_P3 + LF_P4 + LF_P5 + LF_P6,
 	  .size		= LF_P7 },
  	{ .name		= "EXT",
 	  .offset	= LF_P0 + LF_P1 + LF_P2 + LF_P3 + LF_P4 + LF_P5 + LF_P6 + LF_P7,
 	  .size		= MTDPART_SIZ_FULL },
};

static struct mtd_partition partition_info_cart[] = {
	{ .name		= "Cartridge",
	  .offset	= 0,
 	  .size		= MTDPART_SIZ_FULL },
};

#undef LF_P0
#undef LF_P1
#undef LF_P2
#undef LF_P3
#undef LF_P4
#undef LF_P5
#undef LF_P6
#undef LF_P7
#undef LF_ERASE_BLK

/* Ask gpio driver, which sampled this at boot time. */
static u8 get_cart_type(void)
{
	return gpio_get_cart_config ();
}

/*******************
 * sysfs Interface *
 *******************/

static ssize_t show_cart(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	if(nand.mtd_cart == NULL)
		return sprintf(buf, "%s", "none\n");

	switch(get_cart_type()) {
		case NAND_CART_PRODUCTION:
			return sprintf(buf, "%s", "production\n");
		case NAND_CART_DEVELOPMENT:
			return sprintf(buf, "%s", "development\n");
		case NAND_CART_MANUFACTURING:
			return sprintf(buf, "%s", "manufacturing\n");
	}
	return sprintf(buf, "%s", "unknown\n");
}
static DEVICE_ATTR(cartridge, S_IRUSR|S_IRGRP, show_cart, NULL);

#if defined CONFIG_MTD_NAND_LF1000_DEBUG && defined CPU_LF1000
static ssize_t show_nand_timing(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	u32 tmp;

	tmp = readl(NAND_BASE+MEMTIMEACS);
	len += sprintf(buf+len, "0: ACS = %d\n", ((tmp>>22) & 0x3));
	tmp = readl(NAND_BASE+MEMTIMECOS);
	len += sprintf(buf+len, "1: COS = %d\n", ((tmp>>22) & 0x3));
	tmp = readl(NAND_BASE+MEMTIMEACCH);
	len += sprintf(buf+len, "2: ACC = %d\n", ((tmp>>12) & 0xF));
	tmp = readl(NAND_BASE+MEMTIMECOH);
	len += sprintf(buf+len, "3: COH = %d\n", ((tmp>>21) & 0x3));	
	tmp = readl(NAND_BASE+MEMTIMECAH);
	len += sprintf(buf+len, "4: CAH = %d\n", ((tmp>>22) & 0x3));	

	return len;
}

static ssize_t set_nand_timing(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int index, value;
	u32 tmp;

	if(sscanf(buf, "%u,%u", &index, &value) != 2)
		return -EINVAL;
	
	switch(index) {
		case 0: /* ACS */
		tmp = readl(NAND_BASE+MEMTIMEACS) & ~(0x3<<22);
		tmp |= ((0x3 & value)<<22);
		writel(tmp, NAND_BASE+MEMTIMEACS);
		break;
		case 1: /* COS */
		tmp =  readl(NAND_BASE+MEMTIMECOS) & ~(0x3<<22);
		tmp |= ((0x3 & value)<<22);
		writel(tmp, NAND_BASE+MEMTIMECOS);
		break;
		case 2: /* ACC */
		tmp = readl(NAND_BASE+MEMTIMEACCH) & ~(0xF<<12);
		tmp |= ((0xF & value)<<12);
		writel(tmp, NAND_BASE+MEMTIMEACCH);
		break;
		case 3: /* COH */
		tmp = readl(NAND_BASE+MEMTIMECOH) & ~(0x3<<21);
		tmp |= ((0x3 & value)<<21);
		writel(tmp, NAND_BASE+MEMTIMECOH);
		break;
		case 4: /* CAH */
		tmp = readl(NAND_BASE+MEMTIMECAH) & ~(0x3<<22);
		tmp |= ((0x3 & value)<<22);
		writel(tmp, NAND_BASE+MEMTIMECAH);
		break;
		default:
		return -EINVAL;
	}

	return count;
}

static DEVICE_ATTR(timing, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP, 
		show_nand_timing, set_nand_timing);
#endif


#ifdef CONFIG_MTD_NAND_LF1000_PROF
#include "prof.h"
static unsigned long ws_n[NS_MAX], ws_sum[NS_MAX], ws_min[NS_MAX], ws_max[NS_MAX];
static long ws_start[NS_MAX];
static int ws_any = 0;

/* From the timer module; 15 minutes worth of ticks at 4.593MHz fits in 32 bits */
extern int read_current_timer(unsigned long *timer_value);

void nand_stats_erase (void)
{
	int i;
	for (i=0; i<NS_MAX; i++)
		ws_n[i] = ws_sum[i] = 0;
}

/* Accumulate a start (in=1) or stop (in=0) time for a given type of access */
void nand_stats_accum (enum prof_type type, int in)
{
	long stop, delta;
	if (type >= NS_MAX)
	{
		printk (KERN_ALERT "nand_stats_accum: type=%d > NS_MAX", type);
		return;
	}
	if (!ws_any)
	{
		/* First time through, erase stats  */
		nand_stats_erase ();
		ws_any = 1;
	}
	read_current_timer ((unsigned long *)&stop);
	if (in)
	{
		ws_start[type] = stop;
	}
	else
	{
		delta = stop - ws_start[type];
		ws_sum[type] += delta;
		if (!ws_n[type])
		{
			/* First data point, set min and max */
			ws_min[type] = ws_max[type] = delta;
		}
		else
		{
			if (ws_min[type] > delta)
				ws_min[type] = delta;
			if (ws_max[type] < delta)
				ws_max[type] = delta;
		}
		ws_n[type]++;
	}
}

static ssize_t show_write_stats(struct device *dev, struct device_attribute *attr, char *buf)
{
	int x=0, i;
	static char *title[] = {"Read ", "Write", "Erase", "Lock "};
	for (i=0; i<NS_MAX; i++)
	{
		if (ws_n[i])
		{
			x += sprintf (buf+x, "%s N=%ld %ld/%ld/%ld\n", 
				      title[i], ws_n[i], ws_min[i], ws_sum[i]/ws_n[i], ws_max[i]);
		}
		else
		{
			x += sprintf (buf+x, "%s N=%ld %ld/%ld/%ld\n", title[i], 0L,0L,0L,0L);
		}
	}
	return x;
}

static ssize_t clear_write_stats(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	nand_stats_erase ();
	return count;
}

/* Write to this sysfs device: clear stats.  Read: dump out stats.  See profnand.c */
static DEVICE_ATTR(write, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP, 
		   show_write_stats, clear_write_stats);

#endif
/* End #ifdef CONFIG_MTD_NAND_LF1000_PROF */

static struct attribute *nand_attributes[] = {
	&dev_attr_cartridge.attr,
#if defined CONFIG_MTD_NAND_LF1000_DEBUG && defined CPU_LF1000
	&dev_attr_timing.attr,
#endif
#ifdef CONFIG_MTD_NAND_LF1000_PROF
	&dev_attr_write.attr,
#endif
	NULL
};

static struct attribute_group nand_attr_group = {
	.attrs = nand_attributes
};

/*
 * There are 3 ways to test ready/busy bit:
 * 1) test the RnB bit in NFCONTROL (used here)
 * 2) test the IRQPEND bit in NFCONTROL and then set it to clear the interrupt
 * 3) send a NAND_CMD_STATUS to then NAND chip, test the response against
 *    the mask 0x40
 */
static int lf1000_nand_ready(struct mtd_info *mtd)
{
	u32 ctl = readl(NAND_BASE+NFCONTROL);

	if(IS_SET(ctl,RnB))
		return 1;	/* ready */
	return 0;		/* busy */
}

/*
 * hardware-specific access to control and address lines:
 * The LF1000's NAND controller handles the CLE and ALE signals automatically,
 * data must simply be written to the appropriate register: NFCMD or NFADDR
 * respectively.
 */
static void lf1000_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *chip = mtd->priv;

	if(cmd == NAND_CMD_NONE)
		return;

	if(ctrl & NAND_CLE) /* command */
		writeb(cmd, chip->IO_ADDR_W + NFCMD);
	else if(ctrl & NAND_ALE) /* address */
		writeb(cmd, chip->IO_ADDR_W + NFADDR);
}

void lf1000_select_chip(struct mtd_info *mtd, int chipnr)
{
	struct nand_chip *chip = mtd->priv;
	u32 tmp;

	switch(chipnr) {
	case -1:
		chip->cmd_ctrl(mtd, NAND_CMD_NONE, 0 | NAND_CTRL_CHANGE);
		break;
	case 0:
		tmp = readl(NAND_BASE+NFCONTROL);
		BIT_CLR(tmp, NFBANK);
		writel(tmp, NAND_BASE+NFCONTROL);
		break;
	default:
		BUG();
	}
}

void lf1000_select_cart(struct mtd_info *mtd, int chipnr)
{
	struct nand_chip *chip = mtd->priv;
	u32 tmp;

	switch(chipnr) {
	case -1:
		chip->cmd_ctrl(mtd, NAND_CMD_NONE, 0 | NAND_CTRL_CHANGE);
		break;
	case 0:
		tmp = readl(NAND_BASE+NFCONTROL);
		BIT_SET(tmp, NFBANK);
		writel(tmp, NAND_BASE+NFCONTROL);
		break;
	default:
		BUG();
	}
}

#ifdef CONFIG_MTD_PARTITIONS
const char *part_probes[] = { "cmdlinepart", NULL };
#endif

/* Our OTP parts are made by NAND_MFR_TOSHIBA */
#define	OTP_DEVID_128MBIT	0x73	/* 16MByte */
#define	OTP_DEVID_256MBIT	0x75	/* 32MByte */
#define	OTP_DEVID_512MBIT	0x76	/* 64MByte */
#define	OTP_DEVID_1GBIT		0x79	/* 128MByte */

static int lf1000_init_cart(void)
{
	struct nand_chip *cart;

	nand.mtd_cart = kmalloc(sizeof(struct mtd_info) +
			sizeof(struct nand_chip), GFP_KERNEL);
	if(!nand.mtd_cart) {
		printk(KERN_ERR "Unable to allocate LF1000 NAND cart device\n");
		return -ENOMEM;
	}

	nand.mtd_cart->owner = THIS_MODULE;

	/* Get pointer to private data */
	cart = (struct nand_chip *) (&nand.mtd_cart[1]);

	memset(nand.mtd_cart, 0, sizeof(struct mtd_info)+
			sizeof(struct nand_chip));

	cart->controller = &nand.controller;

	/* Link the private data with the MTD structure */
	nand.mtd_cart->priv = cart;

	/* Set address of NAND IO lines */
	cart->IO_ADDR_R = (void __iomem *)(IO_ADDRESS(LF1000_NAND_BASE)+NFDATA);
	cart->IO_ADDR_W = (void __iomem *)(IO_ADDRESS(LF1000_NAND_BASE)+NFDATA);

	cart->dev_ready = lf1000_nand_ready;
	cart->options = 0; /* 8 bit bus width */
	cart->cmd_ctrl = lf1000_hwcontrol; /* hardware access for cmd, addr */
	cart->select_chip = lf1000_select_cart;

	/* 25 us command delay time */
	cart->chip_delay = 25;

#if defined CPU_LF1000 && defined CONFIG_MTD_NAND_LF1000_HWECC
	cart->ecc.mode = NAND_ECC_SOFT; /* TODO: enable hardware ECC for
						 cart as well, once it's fully
						 working on onboard chip... */
#else /* !CPU_LF1000 || !CONFIG_MTD_NAND_LF1000_HWECC */
	cart->ecc.mode = NAND_ECC_SOFT; /* NONE; */
#endif

	/* OTP: detect with cart_id gpios looking for "production" cart */
	if (get_cart_type() == NAND_CART_PRODUCTION) {
		cart->ecc.mode = NAND_ECC_NONE;
	}

	/* report the ECC mode that we wound up with for the cartridge */
	printk(KERN_INFO "lf1000-nand: cartridge ECC mode: ");
	switch(cart->ecc.mode) {
		case NAND_ECC_NONE:
		printk(KERN_INFO "none\n");
		break;
		case NAND_ECC_SOFT:
		printk(KERN_INFO "software\n");
		break;
		case NAND_ECC_HW_SYNDROME:
		printk(KERN_INFO "hardware\n");
		break;
		default:
		printk(KERN_INFO "unknown\n");
		break;
	}

	return 0;
}

/*
 * Platform Device
 */

static int lf1000_nand_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;
	struct nand_chip *this;
	const char *part_type = 0;
	int base_parts_nb = 0;
	int cart_parts_nb = 0;
	struct mtd_partition *base_parts = 0;
	struct mtd_partition *cart_parts = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res) {
		printk(KERN_ERR "nand: failed to get resource!\n");
		return -ENXIO;
	}

	if(!request_mem_region(res->start, (res->end - res->start)+1,
				"lf1000-nand")) {
		printk(KERN_ERR "nand: failed to map memory region.");
		return -EBUSY;
	}

	nand.mem = ioremap(res->start, (res->end - res->start)+1);
	if(nand.mem == NULL) {
		printk(KERN_ERR "nand: failed to ioremap\n");
		ret = -ENOMEM;
		goto fail_remap;
	}

	/* initialize the controller */
	spin_lock_init(&nand.controller.lock);
	init_waitqueue_head(&nand.controller.wq);

	/* Allocate memory for MTD device structure and private data */
	nand.mtd_onboard = kmalloc(sizeof(struct mtd_info) +
			sizeof(struct nand_chip), GFP_KERNEL);
	if(!nand.mtd_onboard) {
		printk(KERN_ERR "Unable to allocate LF1000 NAND MTD device\n");
		ret = -ENOMEM;
		goto fail_mem_onboard;
	}

	nand.mtd_onboard->owner = THIS_MODULE;

	/* Get pointer to private data */
	this = (struct nand_chip *)(&nand.mtd_onboard[1]);

	/* Initialize structures */
	memset(nand.mtd_onboard, 0, sizeof(struct mtd_info)+
			sizeof(struct nand_chip));

	this->controller = &nand.controller;

	/* Link the private data with the MTD structure */
	nand.mtd_onboard->priv = this;

	/* Set address of NAND IO lines */
	this->IO_ADDR_R = (void __iomem *)(IO_ADDRESS(LF1000_NAND_BASE)+NFDATA);
	this->IO_ADDR_W = (void __iomem *)(IO_ADDRESS(LF1000_NAND_BASE)+NFDATA);

	this->dev_ready = lf1000_nand_ready;
	this->options = 0; /* 8 bit bus width */
	this->cmd_ctrl = lf1000_hwcontrol; /* hardware access for cmd, addr */
	this->select_chip = lf1000_select_chip;

	/* 25 us command delay time */
	this->chip_delay = 25;

#if defined CPU_LF1000 && defined CONFIG_MTD_NAND_LF1000_HWECC
	this->ecc.mode = NAND_ECC_HW_SYNDROME;
	this->ecc.size = 512;
	this->ecc.bytes = 6;
	this->ecc.hwctl = lf1000_enable_hwecc;
	this->ecc.calculate = lf1000_calculate_ecc;
	this->ecc.correct = lf1000_correct_ecc;
#else /* !CPU_LF1000 || !CONFIG_MTD_NAND_LF1000_HWECC */
	this->ecc.mode = NAND_ECC_SOFT;
#endif

	/* find the onboard NAND Flash (must exist) */
	if(nand_scan(nand.mtd_onboard, 1)) {
		ret = -ENXIO;
		goto fail_find_onboard;
	}

	/* check if a cartridge is inserted */
	gpio_configure_pin(NAND_CART_DETECT_PORT, NAND_CART_DETECT_PIN,
			GPIO_GPIOFN, 0, 1, 0);
	if(gpio_get_val(NAND_CART_DETECT_PORT, NAND_CART_DETECT_PIN) != 
			NAND_CART_DETECT_LEVEL) {
		printk(KERN_INFO "lf1000-mtd: cartridge not inserted\n");
	} /* if inserted, try to detect it */
	else {
		ret = lf1000_init_cart();
		if(ret)
			goto fail_init_cart;
		if(nand_scan(nand.mtd_cart, 1)) {
			printk(KERN_INFO "lf1000-mtd: cartridge not found\n");
			nand_release(nand.mtd_cart);
			kfree(nand.mtd_cart);
			nand.mtd_cart = NULL;
		}
	}

	/* Add the base partitions */
#ifdef CONFIG_MTD_PARTITIONS
	nand.mtd_onboard->name = "lf1000-base";
	base_parts_nb = parse_mtd_partitions(nand.mtd_onboard, 
			part_probes, &base_parts, 0);
	if (base_parts_nb > 0)
		part_type = "command line";
	else
		base_parts_nb = 0;

	if(nand.mtd_cart != NULL) {
		nand.mtd_cart->name = "lf1000-cart";
		cart_parts_nb = parse_mtd_partitions(nand.mtd_cart,
						     part_probes,
						     &cart_parts, 0);
		if (cart_parts_nb > 0)
			part_type = "command line";
		else
			cart_parts_nb = 0;
	}
#endif
	if (base_parts_nb == 0) {
		base_parts = partition_info;
		base_parts_nb = ARRAY_SIZE(partition_info);
		part_type = "static";
	}

	if (cart_parts_nb == 0) {
		cart_parts = partition_info_cart;
		cart_parts_nb = ARRAY_SIZE(partition_info_cart);
		part_type = "static";
	}

	/* Register the onboard partitions */
	add_mtd_partitions(nand.mtd_onboard, base_parts, base_parts_nb);

	/* Register the cartridge partitions, if it exists */
	if (nand.mtd_cart != NULL) {
		add_mtd_partitions(nand.mtd_cart, cart_parts, cart_parts_nb);
	}

	/* enable NAND_WP pin as an output, enable write & erase */
	gpio_configure_pin(NAND_WP_PORT, NAND_WP_PIN, GPIO_GPIOFN, 1, 0, 1);

	sysfs_create_group(&pdev->dev.kobj, &nand_attr_group);
	return 0;

fail_init_cart:
fail_find_onboard:
	kfree(nand.mtd_onboard);
	if(nand.mtd_cart)
		kfree(nand.mtd_onboard);
fail_mem_onboard:
	iounmap(nand.mem);
fail_remap:
	release_mem_region(res->start, (res->end - res->start) + 1);
	return ret;
}

static int lf1000_nand_remove(struct platform_device *pdev)
{
	struct resource *res  = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	sysfs_remove_group(&pdev->dev.kobj, &nand_attr_group);

	/* Release resources, unregister device */
	nand_release(nand.mtd_onboard);
	if(nand.mtd_cart)
		nand_release(nand.mtd_cart);

	/* Free the MTD device structure */
	kfree(nand.mtd_onboard);
	if(nand.mtd_cart)
		kfree(nand.mtd_cart);

	if(nand.mem != NULL)
		iounmap(nand.mem);

	release_mem_region(res->start, (res->end - res->start) + 1);
	return 0;
}

static struct platform_driver lf1000_nand_driver = {
	.probe		= lf1000_nand_probe,
	.remove		= lf1000_nand_remove,
	.driver		= {
		.name	= "lf1000-nand",
		.owner	= THIS_MODULE,
	},
};

/*
 * Module stuff
 */

static int __init lf1000_init(void)
{
	return platform_driver_register(&lf1000_nand_driver);
}

module_init(lf1000_init);

static void __exit lf1000_cleanup(void)
{
	platform_driver_unregister(&lf1000_nand_driver);
}
module_exit(lf1000_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kosta Demirev <kdemirev@yahoo.com>, Andrey Yurovsky <andrey@cozybit.com>");
MODULE_DESCRIPTION("Glue layer for NAND flash on Leap Frog LF1000");
