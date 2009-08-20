#include <linux/proc_fs.h>
#include <asm/io.h>

#include "ga3d.h"
#include "ga3d_hal.h"

extern struct ga3d_device ga3d;

int ga3d_read_proc(char *buf, char **start, off_t offset, int count,
		   int *eof, void *data);
void ga3d_make_proc(void);
void ga3d_remove_proc(void);
