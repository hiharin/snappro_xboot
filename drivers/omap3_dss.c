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

#include <common.h>
#include <malloc.h>
#include <asm/arch/io.h>
#include <asm/arch/omap3_dss.h>

/*
 * Compute the absolute value of an integer
 */
static inline int abs(int x)
{
	if (x < 0)
		return -x;
	return x;
}

/* 
 * Find dispc dividers that produce req_pck with fck as input clock rate
 */
void dispc_find_clk_divs(s32 is_tft, 
			unsigned long req_pck, 
			unsigned long fck,
			struct dispc_clock_info *cinfo)
{
	u16 pcd_min = is_tft ? 2 : 3;
	u16 best_ld, cur_ld;
	u16 best_pd, cur_pd;
	unsigned long best_pck;

	best_pck = 0;
	best_ld = 0;
	best_pd = 0;

	for (cur_ld = 1; cur_ld <= 255; ++cur_ld) {
		unsigned long lck = fck / cur_ld;

		for (cur_pd = pcd_min; cur_pd <= 255; ++cur_pd) {
			unsigned long pck = lck / cur_pd;
			long old_delta, new_delta;

			old_delta = abs(best_pck - req_pck);
			new_delta = abs(pck - req_pck);

			if (best_pck == 0 || new_delta < old_delta) {
				best_pck = pck;
				best_ld = cur_ld;
				best_pd = cur_pd;

				if (pck == req_pck)
					goto found;
			}

			if (pck < req_pck)
				break;
		}

		if (lck / pcd_min < req_pck)
			break;
	}

found:
	cinfo->lck_div = best_ld;
	cinfo->pck_div = best_pd;
	cinfo->lck = fck / cinfo->lck_div;
	cinfo->pck = cinfo->lck / cinfo->pck_div;
}

/*
 * Calculate dss divisor
 */
s32 omap3_dss_calc_divisor(s32 is_tft, 
			u32 req_pck,
			u32 *dispc_divisor,
			u32 *result_fck_div)
{
	unsigned long prate;
	u16 fck_div, fck_div_max, fck_min_div = 1, fck_div_factor;
	s32 min_fck_per_pck;
	unsigned long fck, max_dss_fck = 173000000; /* max DSS VP_CLK */
	struct dispc_clock_info cur_dispc;
	struct dss_clock_info best_dss;
	struct dispc_clock_info best_dispc;

	prate = 864000000; /* Fclk of DSS (864Mhz) */

	memset(&best_dss, 0, sizeof(best_dss));
	memset(&best_dispc, 0, sizeof(best_dispc));

	min_fck_per_pck = 1;

	fck_div_max = 16;
	fck_div_factor = 1;

	for (fck_div = fck_div_max; fck_div >= fck_min_div; --fck_div) {
		fck = prate / fck_div * fck_div_factor;

		if (fck > max_dss_fck) {
			continue;
		}

		if (min_fck_per_pck &&
			fck < req_pck * min_fck_per_pck) {
			continue;
		}

		dispc_find_clk_divs(is_tft, req_pck, fck, &cur_dispc);

		if (abs(cur_dispc.pck - req_pck) <
			abs(best_dispc.pck - req_pck)) {

			best_dss.fck = fck;
			best_dss.fck_div = fck_div;

			best_dispc = cur_dispc;

			if (cur_dispc.pck == req_pck)
				break;
		}
	}

	/* Setup divisor */
	*dispc_divisor = (cur_dispc.lck_div << 16) | cur_dispc.pck_div;
	*result_fck_div = best_dss.fck_div;
	return 0;
}

/*
 * Configure Panel Specific Parameters
 */
void omap3_dss_panel_config(const struct panel_config *panel_cfg)
{
	struct prcm *prcm_base = (struct prcm *) PRCM_BASE;
    	struct dispc_regs *dispc = (struct dispc_regs *) OMAP3_DISPC_BASE;
    	int ret;
    	u32 divisor, cm_clksel_dss;
    	u32 fck_div;

    	/* Calculate timing of DISPC_DIVISOR; LCD in 16:23, PCD in 0:7 */
    	ret = omap3_dss_calc_divisor(panel_cfg->panel_type == 1,
		    panel_cfg->pixel_clock * 1000, &divisor, &fck_div);
    	cm_clksel_dss = dss_read_reg(&prcm_base->clksel_dss);
    	cm_clksel_dss &= ~0x3f;  /* clear CLKSELDSS1 */
    	cm_clksel_dss |= fck_div; /* or in new clksel_dss1 */
    	dss_write_reg(&prcm_base->clksel_dss, cm_clksel_dss);

    	dss_write_reg(&dispc->timing_h, panel_cfg->timing_h);
    	dss_write_reg(&dispc->timing_v, panel_cfg->timing_v);
    	dss_write_reg(&dispc->pol_freq, panel_cfg->pol_freq);
    	dss_write_reg(&dispc->divisor, divisor);
    	dss_write_reg(&dispc->size_lcd, panel_cfg->lcd_size);
    	dss_write_reg(&dispc->config, 
			(panel_cfg->load_mode << FRAME_MODE_SHIFT));
    	dss_write_reg(&dispc->control, 
			((panel_cfg->panel_type << TFTSTN_SHIFT) |
       			 (panel_cfg->data_lines << DATALINES_SHIFT)));
    	dss_write_reg(&dispc->default_color0, panel_cfg->panel_color);
	dss_write_reg(&dispc->gfx_attributes, (1<<0) | (8<<1));
	dss_write_reg(&dispc->gfx_size, panel_cfg->lcd_size);
}

/*
 * Configure framebuffer address
 */
void omap3_dss_set_fb(const ulong fb)
{
    	struct dispc_regs *dispc = (struct dispc_regs *) OMAP3_DISPC_BASE;

	dss_write_reg(&dispc->gfx_ba0, fb);
	dss_write_reg(&dispc->gfx_ba1, fb);
}

/*
 * Enable LCD and DIGITAL OUT in DSS
 */
void omap3_dss_enable(void)
{
    	struct dispc_regs *dispc = (struct dispc_regs *) OMAP3_DISPC_BASE;
    	u32 l = 0;

    	l = dss_read_reg(&dispc->control);
    	l |= DISPC_ENABLE;
    	dss_write_reg(&dispc->control, l);
}
