/*
 * (C) Copyright 2011 Logic Product Development, Inc.
 *
 * Peter Barada <peter.barada@logicpd.com>
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
#include <common.h>
#include <command.h>
#include <part.h>
#include <fat.h>
#include <asm/arch/cpu.h>
#include <asm/arch/bits.h>
#include <asm/arch/mux.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/sys_info.h>
#include <asm/arch/clocks.h>
#include <asm/arch/mem.h>
#include "dm3730logic-printk_debug.h"

#ifdef CFG_PRINTK_DEBUG
void printk_debug_dump(void)
{
	struct printk_debug *printk_debug;
	unsigned log_start, log_end, log_size;
	unsigned i;
	int found;
	char *log_buf;
	/* The printk debug structure is at the start of DRAM */
	printk_debug = (struct printk_debug *)OMAP34XX_SDRC_CS0 + PRINTK_DEBUG_OFFSET;

	if (printk_debug->tag == PRINTK_DEBUG_COOKIE) {
		log_buf = printk_debug->log_buf_phys;
		log_size = printk_debug->log_size;
		log_start = printk_debug->log_start;
		log_end = printk_debug->log_end;


		printf("%s: found tag, buf %p size %u start %u end %u\n", __FUNCTION__, log_buf, log_size, log_start, log_end);

		/* back up log_end by ndump_char and wrap if need be */
		if (log_end < printk_debug->ndump_chars)
			log_end += log_size;
		log_end -= printk_debug->ndump_chars;

		/* Search forward up to end for next '\n' */
		for (found=0, i=0; !found && (i<printk_debug->ndump_chars); ++i) {
			if (log_buf[log_end] == '\n')
				found=1;
			if (++log_end > log_size)
				log_end = 0;
		}


		/* Now dump until we hit the old log_end */
		printf("%s log_end %u printk_debug->log_end %u\n",
			__FUNCTION__, log_end, printk_debug->log_end);
		while (log_end != printk_debug->log_end) {
			printf("%c", log_buf[log_end]);
			if (++log_end > log_size)
				log_end = 0;
		}

		/* clear out the tag so we don't do it again */
		printk_debug->tag = 0;
	}
}
#else
void printk_debug_dump(void)
{
}
#endif /* CFG_PRINTK_DEBUG */
