/*
 * arch/sh/kernel/cpu/sh2a/probe.c
 *
 * CPU Subtype Probing for SH-2A.
 *
 * Copyright (C) 2004, 2005 Paul Mundt
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/init.h>
#include <asm/processor.h>
#include <asm/cache.h>

int __init detect_cpu_and_cache_system(void)
{
	/* Just SH7206 for now .. */
	cpu_data->type			= CPU_SH7206;

	cpu_data->dcache.ways		= 4;
	cpu_data->dcache.way_incr	= (1 << 11);
	cpu_data->dcache.sets		= 128;
	cpu_data->dcache.entry_shift	= 4;
	cpu_data->dcache.linesz		= L1_CACHE_BYTES;
	cpu_data->dcache.flags		= 0;

	/*
	 * The icache is the same as the dcache as far as this setup is
	 * concerned. The only real difference in hardware is that the icache
	 * lacks the U bit that the dcache has, none of this has any bearing
	 * on the cache info.
	 */
	cpu_data->icache		= cpu_data->dcache;

	return 0;
}

