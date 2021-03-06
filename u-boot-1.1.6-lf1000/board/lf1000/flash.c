/*
 * (C) Copyright 2007
 * Leap Frog Inc.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/* The LF1000 does not actually have any NOR flash.  But for some reason, the
 * cramfs stuff that is pulled in along with JFFS2 read support requires it.
 * Because we're not actually using cramfs, these functions are not really used.
 */

#include <common.h>
#include <flash.h>

#if !defined(CFG_NO_FLASH)

flash_info_t flash_info[CFG_MAX_FLASH_BANKS]; /* info for FLASH chips	*/

/*-----------------------------------------------------------------------
 */
void flash_print_info(flash_info_t *info)
{

}

int flash_erase(flash_info_t *info, int s_first, int s_last)
{
	return 1;
}

/*-----------------------------------------------------------------------
 * Copy memory to flash, returns:
 * 0 - OK
 * 1 - write timeout
 * 2 - Flash not erased
 */

int write_buff(flash_info_t *info, uchar *src, ulong addr, ulong cnt)
{
	return 2;
}

unsigned long flash_init(void)
{
	return 0;
}

#endif /* !CFG_NO_FLASH */
