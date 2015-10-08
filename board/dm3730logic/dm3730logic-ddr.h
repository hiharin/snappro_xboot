/*
 * (C) Copyright 2011
 * Logic Product Development, <www.logicpd.com>
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

extern void config_dm3730logic_ddr(void);
extern char *config_dm3730logic_ddr_type(void);

struct sdram_timings {
	char	*name;
	u32	offset;
	u32	sysconfig;
	u32	sharing;
	u32	mcfg[2];
	u32	rfr[2];
	u32	actima[2];
	u32	actimb[2];
	u32	mr[2];
	u32	emr[2];
	u32	dlla, dllb;
	u32	power, cfg;
};
