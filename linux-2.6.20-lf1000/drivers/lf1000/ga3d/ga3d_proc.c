#include "ga3d_proc.h"

int ga3d_read_proc(char *buf, char **start, off_t offset, int count,
		   int *eof, void *data)
{
	int len = 0;
	void *base = ga3d.mem;

	/* Warning: do not write more than one page of data. */
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "GRP3D_CPCONTROL", ioread32(base + GRP3D_CPCONTROL));
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "GRP3D_CMDQSTART", ioread32(base + GRP3D_CMDQSTART));
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "GRP3D_CMDQEND", ioread32(base + GRP3D_CMDQEND));
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "GRP3D_CMDQFRONT", ioread32(base + GRP3D_CMDQFRONT));
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "GRP3D_CMDQREAR", ioread32(base + GRP3D_CMDQREAR));
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "GRP3D_STATUS", ioread32(base + GRP3D_STATUS));
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "GRP3D_INT", ioread32(base + GRP3D_INT));
	len += sprintf(buf + len,"%s = 0x%08X\n",
		       "GRP3D_CONTROL", ioread32(base + GRP3D_CONTROL));

	*eof = 1; 
	return len; 
}

void ga3d_make_proc(void)
{
	ga3d.proc = create_proc_read_entry("driver/ga3d",
					   0,
					   NULL,
					   ga3d_read_proc,
					   NULL );
}

void ga3d_remove_proc(void)
{
	remove_proc_entry("driver/ga3d", NULL);
}

