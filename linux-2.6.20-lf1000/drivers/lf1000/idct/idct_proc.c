#include "idct_proc.h"

int idct_read_proc(char *buf, char **start, off_t offset, int count,
		   int *eof, void *data)
{
	int len = 0;
	void *base = idct.mem;

	/* Warning: do not write more than one page of data. */
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "IDCT_BUF_DATA", ioread32(base + IDCT_BUF_DATA));
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "IDCT_BUF_DATA", ioread32(base + IDCT_BUF_DATA + 0x04));
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "IDCT_BUF_DATA", ioread32(base + IDCT_BUF_DATA + 0x08));
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "IDCT_BUF_DATA", ioread32(base + IDCT_BUF_DATA + 0x0C));
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "IDCT_BUF_DATA", ioread32(base + IDCT_BUF_DATA + 0x10));
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "IDCT_BUF_DATA", ioread32(base + IDCT_BUF_DATA + 0x14));
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "IDCT_BUF_DATA", ioread32(base + IDCT_BUF_DATA + 0x18));
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "IDCT_BUF_DATA", ioread32(base + IDCT_BUF_DATA + 0x1C));
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "IDCT_CON", ioread32(base + IDCT_CON));
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "IDCT_INT_ENB", ioread32(base + IDCT_INT_ENB));
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "IDCT_INT_PEND", ioread32(base + IDCT_INT_PEND));

	*eof = 1; 
	return len; 
}

void idct_make_proc(void)
{
	idct.proc = create_proc_read_entry("driver/idct",
					   0,
					   NULL,
					   idct_read_proc,
					   NULL );
}

void idct_remove_proc(void)
{
	remove_proc_entry("driver/idct", NULL);
}

