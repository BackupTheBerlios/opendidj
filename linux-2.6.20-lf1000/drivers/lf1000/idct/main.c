/* LF1000 IDCT Macro Block Decoder driver 
 *
 * main.c -- Main driver functionality.
 *
 * Brian Cavagnolo <brian@cozybit.com>
 * Andrey Yurovsky <andrey@cozybit.com>
 * Dave Milici <dmilici@leapfrog.com>
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/arch/platform.h>

#include "idct.h"
#include "idct_proc.h"

/* device private data */
struct idct_device idct = {
	.mem = NULL,
	.cdev = NULL,
	.major = IDCT_MAJOR,
};

/*******************************
 * character device operations *
 *******************************/

static int idct_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static void idct_vma_open(struct vm_area_struct *vma)
{
	printk(KERN_DEBUG "idct: vma_open virt:%lX, phs %lX\n",
	       vma->vm_start, vma->vm_pgoff<<PAGE_SHIFT);
}

static void idct_vma_close(struct vm_area_struct *vma)
{
	printk(KERN_DEBUG "idct: vma_close\n");
}

struct vm_operations_struct idct_vm_ops = {
	.open  = idct_vma_open,
	.close = idct_vma_close,
};

static int idct_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;
	
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_ops = &idct_vm_ops;
	vma->vm_flags |= VM_IO;
	
	ret = io_remap_pfn_range(vma,
				 vma->vm_start, 
				 0xC000F800>>PAGE_SHIFT,
				 vma->vm_end - vma->vm_start, 
				 vma->vm_page_prot);
	if(ret < 0) {
		printk(KERN_ALERT "idct: failed to mmap\n");
		return -EAGAIN;
	}
	
	idct_vma_open(vma);
	return 0;
}

struct file_operations idct_fops = {
	.owner = THIS_MODULE,
	.open  = idct_open,
	.mmap = idct_mmap,
};

/*********************
 *  module functions *
 *********************/

static int lf1000_idct_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res) {
		printk(KERN_ERR "idct: failed to get resource\n");
		return -ENXIO;
	}

	if(!request_mem_region(res->start, (res->end - res->start)+1,
				"lf1000-idct")) {
		printk(KERN_ERR "idct: failed to map region.");
		return -EBUSY;
	}

	idct.mem = ioremap_nocache(res->start, (res->end - res->start)+1);
	if(idct.mem == NULL) {
		printk(KERN_ERR "idct: failed to ioremap\n");
		ret = -ENOMEM;
		goto fail_remap;
	}

	ret = register_chrdev(idct.major, "idct", &idct_fops);
	if(ret < 0) {
		printk(KERN_ERR "idct: failed to get a device\n");
		goto fail_dev;
	}
	if(idct.major == 0) idct.major = ret;

	idct.cdev = cdev_alloc();
	idct.cdev->owner = THIS_MODULE;
	idct.cdev->ops = &idct_fops;
	ret = cdev_add(idct.cdev, 0, 1);
	if(ret < 0) {
		printk(KERN_ALERT "idct: failed to create character device\n");
		goto fail_add;
	}

#ifdef CONFIG_PROC_FS
	idct_make_proc();
#endif
	return 0;

fail_add:
	unregister_chrdev(idct.major, "idct");
fail_dev:
fail_remap:
	release_mem_region(res->start, (res->end - res->start) + 1);

	return ret;
}

static int lf1000_idct_remove(struct platform_device *pdev)
{
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

#ifdef CONFIG_PROC_FS
	idct_remove_proc();
#endif

	unregister_chrdev(idct.major, "idct");
	if(idct.cdev != NULL)
		cdev_del(idct.cdev);
	
	if(idct.mem != NULL)
		iounmap(idct.mem);

	release_mem_region(res->start, (res->end - res->start) + 1);

	return 0;
}

static struct platform_driver lf1000_idct_driver = {
	.probe		= lf1000_idct_probe,
	.remove		= lf1000_idct_remove,
	.driver		= {
		.name	= "lf1000-idct",
		.owner	= THIS_MODULE,
	},
};

static int __init idct_init(void)
{
	return platform_driver_register(&lf1000_idct_driver);
}

static void __exit idct_cleanup(void)
{
	platform_driver_unregister(&lf1000_idct_driver);
}

module_init(idct_init);
module_exit(idct_cleanup);
MODULE_AUTHOR("Brian Cavagnolo, Andrey Yurovsky");
MODULE_VERSION("1:2.0");
