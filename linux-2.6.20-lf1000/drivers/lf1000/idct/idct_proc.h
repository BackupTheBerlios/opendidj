#include <linux/proc_fs.h>
#include <asm/io.h>

#include "idct.h"
#include "idct_hal.h"

extern struct idct_device idct;

int idct_read_proc(char *buf, char **start, off_t offset, int count,
		   int *eof, void *data);
void idct_make_proc(void);
void idct_remove_proc(void);
