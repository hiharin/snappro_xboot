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

#define PRINTK_DEBUG_COOKIE 0xfeedf001
struct printk_debug {
	unsigned int tag;	/* tag, valid if PRINTK_DEBUG_COOKIE */
	char *log_buf_phys;	/* physical address of kernel log_buf */
	unsigned log_size;	/* size of log_buf */
	unsigned log_start;	/* kernel starting offset */
	unsigned log_end;	/* kernel ending offset */
	unsigned ndump_chars;	/* number of characters to print */
};

/* Offset from start of SDRAM to find the printk_debug record */
#define PRINTK_DEBUG_OFFSET 0x0

extern void printk_debug_dump(void);
