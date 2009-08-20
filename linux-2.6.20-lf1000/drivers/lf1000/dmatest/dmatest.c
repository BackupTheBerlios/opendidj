/* LF1000 Dummy DMA Test Driver
 *
 * Brian Cavagnolo <brian@cozybit.com>
 *
 * Welcome to a terribly inefficient demonstration of how to use the LF1000 dma
 * API.  You should read asm/arch/dma.h for details on how the API works.  This
 * sample driver takes write data from userspace and copies it into a dma write
 * buffer.  Next it initiates a dma transfer to copy the dma write buffer to
 * the dma read buffer.  When the DMA transfer completes, readers are
 * awakened.  This driver is terribly inefficient because writers will have to
 * sleep until both the read and write buffers are available.  This is
 * necessary prior to scheduling the DMA.  A ping-pong architecture would be
 * superior, but this is just meant to be a simple demo.
 *
 * To test the demo, enable CONFIG_LF1000_DMATEST in your .config and create
 * the device node /dev/dmatest like so:
 *
 * $ mknod /dev/dmatest c 246 0
 *
 * Now you can use cat to test the dma driver:
 *
 * $ cat /path/to/random/file  > /dev/dmatest & cat /dev/dmatest > dump
 *
 * You will have to ctrl-c after you suspect the transfer is complete.  The
 * reason is that after the writer stops writing, the reader sleeps waiting for
 * more data; yet another reason why this is such a crummy driver worthy only
 * of demonstration.  If everything worked, /path/to/random/file and dump
 * should be the same:
 *
 * $ diff /path/to/random/file dump
 *
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/spinlock.h>
#include <asm/arch/dma.h>
#include <linux/dma-mapping.h>

#define DMATEST_MAJOR 246
#define BUF_SIZE 4096
#define MIN(x, y) (((x) < (y))?(x):(y))

struct dmatest_dev {
	int devnum;
	struct cdev cdev;

	/* lock to protect readers and writers from eachother */
	struct semaphore lock;
	int dmachan;
	struct lf1000_dma_desc desc;

	/* write buffering */
	wait_queue_head_t wqueue;
	char *wbuf;
	dma_addr_t wbuf_dma;
	int wlen;

	/* read buffering */
	wait_queue_head_t rqueue;
	char *rbuf;
	dma_addr_t rbuf_dma;
	int rlen;

	int buf_reader;
	int buf_writer;
};

static struct dmatest_dev dmatest_data;

int dmatest_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	/* Support one reader and one writer */
	if (down_interruptible(&dmatest_data.lock))
		return -ERESTARTSYS;

	if((filp->f_flags & O_ACCMODE) == O_WRONLY) {
		if(dmatest_data.buf_writer != 0) {
			up(&dmatest_data.lock);
			return -EBUSY;
		}
		dmatest_data.buf_writer = 1;
	}

	if((filp->f_flags & O_ACCMODE) == O_RDONLY) {
		if(dmatest_data.buf_reader != 0) {
			up(&dmatest_data.lock);
			return -EBUSY;
		}
		dmatest_data.buf_reader = 1;
	}

	up(&dmatest_data.lock);

	return ret;
}

int dmatest_close(struct inode *inode, struct file *filp)
{
	int ret = 0;

	/* Support one reader and one writer */
	if (down_interruptible(&dmatest_data.lock))
		return -ERESTARTSYS;

	if((filp->f_flags & O_ACCMODE) == O_WRONLY) {
		dmatest_data.buf_writer = 0;
	}

	if((filp->f_flags & O_ACCMODE) == O_RDONLY) {
		dmatest_data.buf_reader = 0;
	}

	up(&dmatest_data.lock);

	return ret;
}

ssize_t dmatest_write(struct file *filp, const char __user *buff, size_t count,
		      loff_t *offp)
{
	if (down_interruptible(&dmatest_data.lock))
		return -ERESTARTSYS;

	/* sleep if write buffer or read buffer is in use */
	while((dmatest_data.wlen != 0) || (dmatest_data.rlen != 0)) {
		up(&dmatest_data.lock);
		if(filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		printk("Write sleeping until buffers available.\n");
		if(wait_event_interruptible(dmatest_data.wqueue,
					    (dmatest_data.wlen == 0) &&
					    (dmatest_data.rlen == 0)))
			return -ERESTARTSYS;
		if(down_interruptible(&dmatest_data.lock))
			return -ERESTARTSYS;
	}
	
	/* okay.  The write buffer and read buffers are both empty, and we have
	 * the semaphore.  wlen is zero so we know that no dma is active right
	 * now.
	 */
	count = MIN(count, BUF_SIZE);
	if(copy_from_user(&dmatest_data.wbuf[0], buff, count)) {
		up(&dmatest_data.lock);
		return -EFAULT;
	}
	dmatest_data.wlen = count;

	/* Launch the DMA */
	lf1000_dma_initdesc(&dmatest_data.desc);
	dmatest_data.desc.src = dmatest_data.wbuf_dma;
	dmatest_data.desc.dst = dmatest_data.rbuf_dma;
	dmatest_data.desc.len = count;
	dmatest_data.desc.flags = (DMA_DST_MEM | DMA_DST_8BIT);
	dmatest_data.desc.flags |= (DMA_SRC_MEM | DMA_SRC_8BIT);
	lf1000_dma_launch(dmatest_data.dmachan, &dmatest_data.desc);

	printk("Successfully wrote %d bytes.\n", count);
	up(&dmatest_data.lock);
	return count;
}

ssize_t dmatest_read(struct file *filp, char __user *buff, size_t count,
		     loff_t *offp)
{
	int real_count;

	if (down_interruptible(&dmatest_data.lock))
		return -ERESTARTSYS;

	/* sleep if read buffer has no data */
	while(dmatest_data.rlen == 0) {
		up(&dmatest_data.lock);
		if(filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		printk("Read sleeping until data is ready.\n");
		if(wait_event_interruptible(dmatest_data.rqueue,
					    (dmatest_data.rlen != 0)))
			return -ERESTARTSYS;
		if(down_interruptible(&dmatest_data.lock))
			return -ERESTARTSYS;
	}
	
	/* okay.  The read buffer has data and we have the semaphore. */
	real_count = MIN(count, dmatest_data.rlen);
	if(copy_to_user(buff, &dmatest_data.rbuf[0], real_count)) {
		up(&dmatest_data.lock);
		return -EFAULT;
	}

	if(real_count < dmatest_data.rlen) {
		/* copy the residue down to 0 for the next reader */
		memcpy(&dmatest_data.wbuf[0], &dmatest_data.wbuf[real_count],
		       dmatest_data.rlen - real_count);
		dmatest_data.rlen = dmatest_data.rlen - real_count;
	} else {
		/* we consumed all of the data */
		dmatest_data.rlen = 0;
		wake_up_interruptible(&dmatest_data.wqueue);
	}
	printk("Successfully read %d bytes.\n", real_count);
	up(&dmatest_data.lock);
	return real_count;

}

struct file_operations dmatest_fops = {
	.owner = THIS_MODULE,
	.open = dmatest_open,
	.open = dmatest_close,
	.read = dmatest_read,
	.write = dmatest_write,
};

/* DMA handers are called in IRQ context.  No sleeping! */
static void dmatest_handler(int channel, void *priv)
{
	
	if(channel == dmatest_data.dmachan) {
		printk("DMA transfer complete\n");
		dmatest_data.wlen = 0;
		dmatest_data.rlen = dmatest_data.desc.len;
		/* wake up readers then writers */
		wake_up_interruptible(&dmatest_data.rqueue);
		wake_up_interruptible(&dmatest_data.wqueue);
		lf1000_dma_int_dis(channel);
	} else {
		/* should never get here! */
		printk(KERN_CRIT "dmatest_handler called with bad channel!\n");
	}
}

void dmatest_cleanup(void)
{
	if(dmatest_data.dmachan >= 0) {
		lf1000_dma_free(dmatest_data.dmachan);
	}

	if(dmatest_data.rbuf) {
		lf1000_dma_buf_free(BUF_SIZE, (void *)dmatest_data.rbuf,
				    dmatest_data.rbuf_dma);
	}
	if(dmatest_data.wbuf) {
		lf1000_dma_buf_free(BUF_SIZE, (void *)dmatest_data.wbuf,
				    dmatest_data.wbuf_dma);
	}

	cdev_del(&dmatest_data.cdev);
}

int __init dmatest_init(void)
{
	int ret = 0;

	memset(&dmatest_data, 0, sizeof(struct dmatest_dev));

	/* Set up the char device */
	dmatest_data.devnum = MKDEV(DMATEST_MAJOR, 0);
	cdev_init(&dmatest_data.cdev, &dmatest_fops);
	dmatest_data.cdev.owner = THIS_MODULE;
	dmatest_data.cdev.ops = &dmatest_fops;
	ret = cdev_add(&dmatest_data.cdev, dmatest_data.devnum, 1);

	if(ret) {
		printk(KERN_ALERT "dmatest: failed to get a device\n");
		goto fail;
	}

	/* Set up the read and write buffers */
	init_MUTEX(&dmatest_data.lock);
	init_waitqueue_head(&dmatest_data.rqueue);
	init_waitqueue_head(&dmatest_data.wqueue);
	dmatest_data.wbuf = (char *)lf1000_dma_buf_alloc(BUF_SIZE,
						&dmatest_data.wbuf_dma);
	if(!dmatest_data.wbuf) {
		ret = -ENOMEM;
		goto fail;
	}
	dmatest_data.rbuf = (char *)lf1000_dma_buf_alloc(BUF_SIZE,
						&dmatest_data.rbuf_dma);
	if(!dmatest_data.rbuf) {
		ret = -ENOMEM;
		goto fail;
	}

	dmatest_data.dmachan = lf1000_dma_request("dmatest", DMA_PRIO_MEDIUM,
						  dmatest_handler, NULL);
	if(dmatest_data.dmachan < 0) {
		ret = dmatest_data.dmachan;
		goto fail;
	}

	return 0;
 fail:
	dmatest_cleanup();
	return ret;
}

module_init(dmatest_init);
module_exit(dmatest_cleanup);
MODULE_AUTHOR("Brian Cavagnolo");
MODULE_VERSION("1:1.0");
