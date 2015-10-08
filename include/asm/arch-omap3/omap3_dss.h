/*
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 * Syed Mohammed Khasim <khasim <at> ti.com>
 *
 * Referred to Linux Kernel DSS driver files for OMAP3 by
 * Tomi Valkeinen from drivers/video/omap2/dss/
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation's version 2 and any
 * later version the License.
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

#ifndef DSS_H
#define DSS_H

/*
 * DSS Base Registers
 */
#define OMAP3_DSS_BASE  	0x48050040
#define OMAP3_DISPC_BASE	0x48050440

/* Enabling Display controller */
#define LCD_ENABLE		1
#define DIG_ENABLE		(1<<1)
#define GO_LCD			(1<<5)
#define GO_DIG			(1<<6)
#define GP_OUT0			(1<<15)
#define GP_OUT1			(1<<16)

#define DISPC_ENABLE		(LCD_ENABLE | \
				DIG_ENABLE | \
				GO_LCD | \
				GO_DIG | \
				GP_OUT0| \
				GP_OUT1)

/* Few Register Offsets */
#define FRAME_MODE_SHIFT       1
#define TFTSTN_SHIFT           3
#define DATALINES_SHIFT        8

#define LCD_DL_16BIT		1
#define LCD_DL_24BIT		3

#define dss_write_reg(r, v) 	__raw_writel(v, r)

#define dss_read_reg(r) 	__raw_readl(r)

/* DSS Registers */
struct dss_regs {
    	u32 control;                /* 0x40 */
    	u32 sdi_control;            /* 0x44 */
    	u32 pll_control;            /* 0x48 */
};

/* DISPC Registers */
struct dispc_regs {
    	u32 control;                /* 0x40 */
    	u32 config;                 /* 0x44 */
    	u32 reserve_2;              /* 0x48 */
    	u32 default_color0;         /* 0x4C */
    	u32 default_color1;         /* 0x50 */
    	u32 trans_color0;           /* 0x54 */
    	u32 trans_color1;           /* 0x58 */
    	u32 line_status;            /* 0x5C */
    	u32 line_number;            /* 0x60 */
    	u32 timing_h;               /* 0x64 */
    	u32 timing_v;               /* 0x68 */
    	u32 pol_freq;               /* 0x6C */
    	u32 divisor;                /* 0x70 */
    	u32 global_alpha;           /* 0x74 */
    	u32 size_dig;               /* 0x78 */
    	u32 size_lcd;               /* 0x7C */
    	u32 gfx_ba0;                /* 0x80 */
    	u32 gfx_ba1;                /* 0x84 */
    	u32 gfx_position;           /* 0x88 */
    	u32 gfx_size;               /* 0x8C */
    	u32 padding1;               /* 0x90 */
    	u32 padding2;               /* 0x94 */
    	u32 padding3;               /* 0x98 */
    	u32 padding4;               /* 0x9C */
    	u32 gfx_attributes;         /* 0xA0 */
    	u32 gfx_fifo_threashold;    /* 0xA4 */
    	u32 gfx_fifo_size_status;   /* 0xA8 */
    	u32 gfx_row_inc;            /* 0xAC */
    	u32 gfx_pixel_inc;          /* 0xB0 */
    	u32 gfx_window_skip;        /* 0xB4 */
    	u32 gfx_table_ba;           /* 0xB8 */
};

enum omap_panel_config {
	OMAP_DSS_LCD_IVS	= 1<<0,
	OMAP_DSS_LCD_IHS	= 1<<1,
	OMAP_DSS_LCD_IPC	= 1<<2,
	OMAP_DSS_LCD_IEO	= 1<<3,
	OMAP_DSS_LCD_RF		= 1<<4,
	OMAP_DSS_LCD_ONOFF	= 1<<5,
	OMAP_DSS_LCD_TFT	= 1<<20,
};

/*
 * Panel Configuration
 */
struct panel_config {
    	u32 timing_h;
    	u32 timing_v;
    	u32 pol_freq;
    	u32 pixel_clock;
    	u32 lcd_size;
    	u32 panel_color;
    	u8 panel_type;
    	u8 data_lines;
    	u8 load_mode;
};

struct dss_clock_info {
	/* rates that we get with dividers below */
	ulong fck;
	/* dividers */
	u16 fck_div;
};

struct dispc_clock_info {
	/* rates that we get with dividers below */
	ulong lck;
	ulong pck;
	/* dividers */
	u16 lck_div;
	u16 pck_div;
};

/*
 * Generic DSS Functions
 */
void omap3_dss_panel_config(const struct panel_config *panel_cfg);
void omap3_dss_set_fb(const ulong fb);
void omap3_dss_enable(void);

#endif /* DSS_H */
