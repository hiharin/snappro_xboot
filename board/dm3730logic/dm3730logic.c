/*
 * (C) Copyright 2011
 * Logic Product Development <www.logicpd.com>
 * Peter Barada <peter.barada@logicpd.com>
 *
 * Based on omap3evm.c
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
#include <malloc.h>
#include <asm/arch/cpu.h>
#include <asm/arch/bits.h>
#include <asm/arch/mux.h>
#include <asm/arch/io.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/sys_info.h>
#include <asm/arch/clocks.h>
#include <asm/arch/mem.h>
#include "dm3730logic-ddr.h"
#include "dm3730logic-printk_debug.h"

/* Used to index into DPLL parameter tables */
struct dpll_param {
        unsigned int m;
        unsigned int n;
        unsigned int fsel;
        unsigned int m2;
};

struct dpll_per_36x_param {
	unsigned int sys_clk;
	unsigned int m;
	unsigned int n;
	unsigned int m2;
	unsigned int m3;
	unsigned int m4;
	unsigned int m5;
	unsigned int m6;
	unsigned int m2div;
};

typedef struct dpll_param dpll_param;

extern unsigned int is_ddr_166M;

#define MAX_SIL_INDEX	3

/* Following functions are exported from lowlevel_init.S */
extern dpll_param *get_mpu_dpll_param(void);
extern dpll_param *get_iva_dpll_param(void);
extern dpll_param *get_core_dpll_param(void);
extern dpll_param *get_per_dpll_param(void);

extern dpll_param *get_36x_mpu_dpll_param(void);
extern dpll_param *get_36x_iva_dpll_param(void);
extern dpll_param *get_36x_core_dpll_param(void);
extern dpll_param *get_36x_per_dpll_param(void);

extern int mmc_init(int verbose);
extern block_dev_desc_t *mmc_get_dev(int dev);

omap_boot_device_t omap_boot_device(void)
{
	/* The trace vector area is copied from 0x4020FFBO on startup.
	 * _trace_vector_area_save[1] is the hi work of the trace vector
	 * that holds the memory boot bits */
	extern u32 _trace_vector_area_save[5];
	u32 trace_vect_hi;

	trace_vect_hi = _trace_vector_area_save[1];
#if 0
	printf("%s: %08x %08x %08x %08x %08x\n", __FUNCTION__,
		_trace_vector_area_save[0],
		_trace_vector_area_save[1],
		_trace_vector_area_save[2],
		_trace_vector_area_save[3],
		_trace_vector_area_save[4]);

	printf("%s: trace_vect_hi %08x\n", __FUNCTION__, trace_vect_hi);
	printf("%s: PRCM.CONTROL_STATUS %08x\n", __FUNCTION__, *((unsigned int*)0x480022f0));
#endif


	if (trace_vect_hi & 0x00000002)
		return boot_device_nor;
	if (trace_vect_hi & 0x00000004)
		return boot_device_nand;
	if (trace_vect_hi & 0x00000008)
		return boot_device_onenand;
	if (trace_vect_hi & 0x00000010)
		return boot_device_doc;
	if (trace_vect_hi & 0x00000060)
		return boot_device_sdmmc;
	return boot_device_unknown;
}

/*******************************************************
 * Routine: delay
 * Description: spinning delay to use before udelay works
 ******************************************************/
static inline void delay(unsigned long loops)
{
	__asm__ volatile ("1:\n" "subs %0, %1, #1\n"
			  "bne 1b":"=r" (loops):"0"(loops));
}

void udelay (unsigned long usecs) {
	delay(usecs);
}

/*****************************************
 * Routine: board_init
 * Description: Early hardware init.
 *****************************************/
int board_init (void)
{
	return 0;
}

/*************************************************************
 *  get_device_type(): tell if GP/HS/EMU/TST
 *************************************************************/
u32 get_device_type(void)
{
        int mode;
        mode = __raw_readl(CONTROL_STATUS) & (DEVICE_MASK);
        return(mode >>= 8);
}

/************************************************
 * get_sysboot_value(void) - return SYS_BOOT[4:0]
 ************************************************/
u32 get_sysboot_value(void)
{
        int mode;
        mode = __raw_readl(CONTROL_STATUS) & (SYSBOOT_MASK);
        return mode;
}
/*************************************************************
 * Routine: get_mem_type(void) - returns the kind of memory connected
 * to GPMC that we are trying to boot form. Uses SYS BOOT settings.
 *************************************************************/
u32 get_mem_type(void)
{
        u32   mem_type = get_sysboot_value();
        switch (mem_type){
            case 0:
            case 2:
            case 4:
            case 16:
            case 22:    return GPMC_ONENAND;

            case 1:
            case 12:
            case 15:
            case 21:
            case 27:    return GPMC_NAND;

            case 3:
            case 6:     return MMC_ONENAND;

            case 8:
            case 11:
            case 14:
            case 20:
            case 26:    return GPMC_MDOC;

            case 17:
            case 18:
            case 24:	return MMC_NAND;

            case 7:
            case 10:
            case 13:
            case 19:
            case 25:
            default:    return GPMC_NOR;
        }
}

/******************************************
 * get_cpu_rev(void) - extract version info
 ******************************************/
u32 get_cpu_rev(void)
{
	u32 cpuid=0;
	/* On ES1.0 the IDCODE register is not exposed on L4
	 * so using CPU ID to differentiate
	 * between ES2.0 and ES1.0.
	 */
	__asm__ __volatile__("mrc p15, 0, %0, c0, c0, 0":"=r" (cpuid));
	if((cpuid  & 0xf) == 0x0)
		return CPU_3430_ES1;
	else
		return CPU_3430_ES2;

}

u32 is_cpu_family(void)
{
	u32 cpuid = 0, cpu_family = 0;
	u16 hawkeye;

	__asm__ __volatile__("mrc p15, 0, %0, c0, c0, 0":"=r"(cpuid));
	if ((cpuid & 0xf) == 0x0) {
		cpu_family = CPU_OMAP34XX;
	} else {
		cpuid = __raw_readl(OMAP34XX_CONTROL_ID);
		hawkeye  = (cpuid >> HAWKEYE_SHIFT) & 0xffff;

		switch (hawkeye) {
			case HAWKEYE_OMAP34XX:
				cpu_family = CPU_OMAP34XX;
				break;
			case HAWKEYE_AM35XX:
				cpu_family = CPU_AM35XX;
				break;
			case HAWKEYE_OMAP36XX:
				cpu_family = CPU_OMAP36XX;
				break;
			default:
				cpu_family = CPU_OMAP34XX;
				break;
		}
	}
	return cpu_family;
}
/******************************************
 * cpu_is_3410(void) - returns true for 3410
 ******************************************/
u32 cpu_is_3410(void)
{
	int status;
	if(get_cpu_rev() < CPU_3430_ES2) {
		return 0;
	} else {
		/* read scalability status and return 1 for 3410*/
		status = __raw_readl(CONTROL_SCALABLE_OMAP_STATUS);
		/* Check whether MPU frequency is set to 266 MHz which
		 * is nominal for 3410. If yes return true else false
		 */
		if (((status >> 8) & 0x3) == 0x2)
			return 1;
		else
			return 0;
	}
}

/*****************************************************************
 * sr32 - clear & set a value in a bit range for a 32 bit address
 *****************************************************************/
void sr32(u32 addr, u32 start_bit, u32 num_bits, u32 value)
{
	u32 tmp, msk = 0;
	msk = 1 << num_bits;
	--msk;
	tmp = __raw_readl(addr) & ~(msk << start_bit);
	tmp |=  value << start_bit;
	__raw_writel(tmp, addr);
}

/*********************************************************************
 * wait_on_value() - common routine to allow waiting for changes in
 *   volatile regs.
 *********************************************************************/
u32 wait_on_value(u32 read_bit_mask, u32 match_value, u32 read_addr, u32 bound)
{
	u32 i = 0, val;
	do {
		++i;
		val = __raw_readl(read_addr) & read_bit_mask;
		if (val == match_value)
			return (1);
		if (i == bound)
			return (0);
	} while (1);
}

/*************************************************************
 * get_sys_clk_speed - determine reference oscillator speed
 *  based on known 32kHz clock and gptimer.
 *************************************************************/
u32 get_osc_clk_speed(void)
{
	u32 start, cstart, cend, cdiff, cdiv, val;

	val = __raw_readl(PRM_CLKSRC_CTRL);

	if (val & BIT7)
		cdiv = 2;
	else if (val & BIT6)
		cdiv = 1;
	else
		/*
		 * Should never reach here!
		 * TBD: Add a WARN()/BUG()
		 *      For now, assume divider as 1.
		 */
		cdiv = 1;

	/* enable timer2 */
	val = __raw_readl(CM_CLKSEL_WKUP) | BIT0;
	__raw_writel(val, CM_CLKSEL_WKUP);	/* select sys_clk for GPT1 */

	/* Enable I and F Clocks for GPT1 */
	val = __raw_readl(CM_ICLKEN_WKUP) | BIT0 | BIT2;
	__raw_writel(val, CM_ICLKEN_WKUP);
	val = __raw_readl(CM_FCLKEN_WKUP) | BIT0;
	__raw_writel(val, CM_FCLKEN_WKUP);

	__raw_writel(0, OMAP34XX_GPT1 + TLDR);	/* start counting at 0 */
	__raw_writel(GPT_EN, OMAP34XX_GPT1 + TCLR);     /* enable clock */
	/* enable 32kHz source *//* enabled out of reset */
	/* determine sys_clk via gauging */

	start = 20 + __raw_readl(S32K_CR);	/* start time in 20 cycles */
	while (__raw_readl(S32K_CR) < start);	/* dead loop till start time */
	cstart = __raw_readl(OMAP34XX_GPT1 + TCRR);	/* get start sys_clk count */
	while (__raw_readl(S32K_CR) < (start + 20));	/* wait for 40 cycles */
	cend = __raw_readl(OMAP34XX_GPT1 + TCRR);	/* get end sys_clk count */
	cdiff = cend - cstart;				/* get elapsed ticks */

	if (cdiv == 2)
	{
		cdiff *= 2;
	}

	/* based on number of ticks assign speed */
	if (cdiff > 19000)
		return (S38_4M);
	else if (cdiff > 15200)
		return (S26M);
	else if (cdiff > 13000)
		return (S24M);
	else if (cdiff > 9000)
		return (S19_2M);
	else if (cdiff > 7600)
		return (S13M);
	else
		return (S12M);
}

/******************************************************************************
 * get_sys_clkin_sel() - returns the sys_clkin_sel field value based on
 *   -- input oscillator clock frequency.
 *
 *****************************************************************************/
void get_sys_clkin_sel(u32 osc_clk, u32 *sys_clkin_sel)
{
	if(osc_clk == S38_4M)
		*sys_clkin_sel=  4;
	else if(osc_clk == S26M)
		*sys_clkin_sel = 3;
	else if(osc_clk == S19_2M)
		*sys_clkin_sel = 2;
	else if(osc_clk == S13M)
		*sys_clkin_sel = 1;
	else if(osc_clk == S12M)
		*sys_clkin_sel = 0;
}

/*
 * OMAP34x/35x specific functions
 */
static void dpll3_init_34xx(u32 sil_index, u32 clk_index)
{
	dpll_param *ptr;

	/* Getting the base address of Core DPLL param table*/
	ptr = (dpll_param *)get_core_dpll_param();

	/* Moving it to the right sysclk and ES rev base */
	ptr = ptr + 2*clk_index + sil_index;

	/* CORE DPLL */
	/* Select relock bypass: CM_CLKEN_PLL[0:2] */
	sr32(CM_CLKEN_PLL, 0, 3, PLL_FAST_RELOCK_BYPASS);
	wait_on_value(BIT0, 0, CM_IDLEST_CKGEN, LDELAY);

	/* CM_CLKSEL1_EMU[DIV_DPLL3] */
	sr32(CM_CLKSEL1_EMU, 16, 5, CORE_M3X2);

	/* M2 (CORE_DPLL_CLKOUT_DIV): CM_CLKSEL1_PLL[27:31] */
	sr32(CM_CLKSEL1_PLL, 27, 5, ptr->m2);

	/* M (CORE_DPLL_MULT): CM_CLKSEL1_PLL[16:26] */
	sr32(CM_CLKSEL1_PLL, 16, 11, ptr->m);

	/* N (CORE_DPLL_DIV): CM_CLKSEL1_PLL[8:14] */
	sr32(CM_CLKSEL1_PLL, 8, 7, ptr->n);

	/* Source is the CM_96M_FCLK: CM_CLKSEL1_PLL[6] */
	sr32(CM_CLKSEL1_PLL, 6, 1, 0);

	sr32(CM_CLKSEL_CORE, 8, 4, CORE_SSI_DIV);	/* ssi */
	sr32(CM_CLKSEL_CORE, 4, 2, CORE_FUSB_DIV);	/* fsusb */
	sr32(CM_CLKSEL_CORE, 2, 2, CORE_L4_DIV);	/* l4 */
	sr32(CM_CLKSEL_CORE, 0, 2, CORE_L3_DIV);	/* l3 */

	sr32(CM_CLKSEL_GFX,  0, 3, GFX_DIV_34X);	/* gfx */
	sr32(CM_CLKSEL_WKUP, 1, 2, WKUP_RSM);		/* reset mgr */

	/* FREQSEL (CORE_DPLL_FREQSEL): CM_CLKEN_PLL[4:7] */
	sr32(CM_CLKEN_PLL,   4, 4, ptr->fsel);
	sr32(CM_CLKEN_PLL,   0, 3, PLL_LOCK);		/* lock mode */

	wait_on_value(BIT0, 1, CM_IDLEST_CKGEN, LDELAY);
}

static void dpll4_init_34xx(u32 sil_index, u32 clk_index)
{
	dpll_param *ptr;

	ptr = (dpll_param *)get_per_dpll_param();

	/* Moving it to the right sysclk base */
	ptr = ptr + clk_index;

	/* EN_PERIPH_DPLL: CM_CLKEN_PLL[16:18] */
	sr32(CM_CLKEN_PLL, 16, 3, PLL_STOP);
	wait_on_value(BIT1, 0, CM_IDLEST_CKGEN, LDELAY);

	sr32(CM_CLKSEL1_EMU, 24, 5, PER_M6X2);		/* set M6 */
	sr32(CM_CLKSEL_CAM, 0, 5, PER_M5X2);		/* set M5 */
	sr32(CM_CLKSEL_DSS, 0, 5, PER_M4X2);		/* set M4 */
	sr32(CM_CLKSEL_DSS, 8, 5, PER_M3X2);		/* set M3 */

	/* M2 (DIV_96M): CM_CLKSEL3_PLL[0:4] */
	sr32(CM_CLKSEL3_PLL, 0, 5, ptr->m2);

	/* M (PERIPH_DPLL_MULT): CM_CLKSEL2_PLL[8:18] */
	sr32(CM_CLKSEL2_PLL, 8, 11, ptr->m);

	/* N (PERIPH_DPLL_DIV): CM_CLKSEL2_PLL[0:6] */
	sr32(CM_CLKSEL2_PLL, 0, 7, ptr->n);

	/* FREQSEL (PERIPH_DPLL_FREQSEL): CM_CLKEN_PLL[20:23] */
	sr32(CM_CLKEN_PLL, 20, 4, ptr->fsel);

	/* LOCK MODE (EN_PERIPH_DPLL) : CM_CLKEN_PLL[16:18] */
	sr32(CM_CLKEN_PLL, 16, 3, PLL_LOCK);
	wait_on_value(BIT1, 2, CM_IDLEST_CKGEN, LDELAY);
}

static void mpu_init_34xx(u32 sil_index, u32 clk_index)
{
	dpll_param *ptr;

	/* Getting the base address to MPU DPLL param table*/
	ptr = (dpll_param *)get_mpu_dpll_param();

	/* Moving it to the right sysclk and ES rev base */
	ptr = ptr + 2*clk_index + sil_index;

	/* MPU DPLL (unlocked already) */
	/* M2 (MPU_DPLL_CLKOUT_DIV) : CM_CLKSEL2_PLL_MPU[0:4] */
	sr32(CM_CLKSEL2_PLL_MPU, 0, 5, ptr->m2);

	/* M (MPU_DPLL_MULT) : CM_CLKSEL2_PLL_MPU[8:18] */
	sr32(CM_CLKSEL1_PLL_MPU, 8, 11, ptr->m);

	/* N (MPU_DPLL_DIV) : CM_CLKSEL2_PLL_MPU[0:6] */
	sr32(CM_CLKSEL1_PLL_MPU, 0, 7, ptr->n);

	/* FREQSEL (MPU_DPLL_FREQSEL) : CM_CLKEN_PLL_MPU[4:7] */
	sr32(CM_CLKEN_PLL_MPU, 4, 4, ptr->fsel);
}

static void iva_init_34xx(u32 sil_index, u32 clk_index)
{
	dpll_param *ptr;

	/* Getting the base address to IVA DPLL param table*/
	ptr = (dpll_param *)get_iva_dpll_param();

	/* Moving it to the right sysclk and ES rev base */
	ptr = ptr + 2*clk_index + sil_index;

	/* IVA DPLL */
	/* EN_IVA2_DPLL : CM_CLKEN_PLL_IVA2[0:2] */
	sr32(CM_CLKEN_PLL_IVA2, 0, 3, PLL_STOP);
	wait_on_value(BIT0, 0, CM_IDLEST_PLL_IVA2, LDELAY);

	/* M2 (IVA2_DPLL_CLKOUT_DIV) : CM_CLKSEL2_PLL_IVA2[0:4] */
	sr32(CM_CLKSEL2_PLL_IVA2, 0, 5, ptr->m2);

	/* M (IVA2_DPLL_MULT) : CM_CLKSEL1_PLL_IVA2[8:18] */
	sr32(CM_CLKSEL1_PLL_IVA2, 8, 11, ptr->m);

	/* N (IVA2_DPLL_DIV) : CM_CLKSEL1_PLL_IVA2[0:6] */
	sr32(CM_CLKSEL1_PLL_IVA2, 0, 7, ptr->n);

	/* FREQSEL (IVA2_DPLL_FREQSEL) : CM_CLKEN_PLL_IVA2[4:7] */
	sr32(CM_CLKEN_PLL_IVA2, 4, 4, ptr->fsel);

	/* LOCK MODE (EN_IVA2_DPLL) : CM_CLKEN_PLL_IVA2[0:2] */
	sr32(CM_CLKEN_PLL_IVA2, 0, 3, PLL_LOCK);

	wait_on_value(BIT0, 1, CM_IDLEST_PLL_IVA2, LDELAY);
}

/*
 * OMAP3630 specific functions
 */
static void dpll3_init_36xx(u32 sil_index, u32 clk_index)
{
	dpll_param *ptr;

	/* Getting the base address of Core DPLL param table*/
	ptr = (dpll_param *)get_36x_core_dpll_param();

	/* Moving it to the right sysclk and ES rev base */
	ptr += clk_index;

	/* CORE DPLL */
	/* Select relock bypass: CM_CLKEN_PLL[0:2] */
	sr32(CM_CLKEN_PLL, 0, 3, PLL_FAST_RELOCK_BYPASS);
	wait_on_value(BIT0, 0, CM_IDLEST_CKGEN, LDELAY);

	/* CM_CLKSEL1_EMU[DIV_DPLL3] */
	sr32(CM_CLKSEL1_EMU, 16, 5, CORE_M3X2);

	/* M2 (CORE_DPLL_CLKOUT_DIV): CM_CLKSEL1_PLL[27:31] */
	sr32(CM_CLKSEL1_PLL, 27, 5, ptr->m2);

	/* M (CORE_DPLL_MULT): CM_CLKSEL1_PLL[16:26] */
	sr32(CM_CLKSEL1_PLL, 16, 11, ptr->m);

	/* N (CORE_DPLL_DIV): CM_CLKSEL1_PLL[8:14] */
	sr32(CM_CLKSEL1_PLL, 8, 7, ptr->n);

	/* Source is the CM_96M_FCLK: CM_CLKSEL1_PLL[6] */
	sr32(CM_CLKSEL1_PLL, 6, 1, 0);

	sr32(CM_CLKSEL_CORE, 8, 4, CORE_SSI_DIV);	/* ssi */
	sr32(CM_CLKSEL_CORE, 4, 2, CORE_FUSB_DIV);	/* fsusb */
	sr32(CM_CLKSEL_CORE, 2, 2, CORE_L4_DIV);	/* l4 */
	sr32(CM_CLKSEL_CORE, 0, 2, CORE_L3_DIV);	/* l3 */

	sr32(CM_CLKSEL_GFX,  0, 3, GFX_DIV_36X);		/* gfx */
	sr32(CM_CLKSEL_WKUP, 1, 2, WKUP_RSM);		/* reset mgr */

	/* FREQSEL (CORE_DPLL_FREQSEL): CM_CLKEN_PLL[4:7] */
	sr32(CM_CLKEN_PLL,   4, 4, ptr->fsel);
	sr32(CM_CLKEN_PLL,   0, 3, PLL_LOCK);		/* lock mode */

	wait_on_value(BIT0, 1, CM_IDLEST_CKGEN, LDELAY);
}

static void dpll4_init_36xx(u32 sil_index, u32 clk_index)
{
	struct dpll_per_36x_param *ptr;

	ptr = (struct dpll_per_36x_param *)get_36x_per_dpll_param();

	/* Moving it to the right sysclk base */
	ptr += clk_index;

	/* EN_PERIPH_DPLL: CM_CLKEN_PLL[16:18] */
	sr32(CM_CLKEN_PLL, 16, 3, PLL_STOP);
	wait_on_value(BIT1, 0, CM_IDLEST_CKGEN, LDELAY);

	/* M6 (DIV_DPLL4): CM_CLKSEL1_EMU[24:29] */
	sr32(CM_CLKSEL1_EMU, 24, 6, ptr->m6);

	/* M5 (CLKSEL_CAM): CM_CLKSEL1_EMU[0:5] */
	sr32(CM_CLKSEL_CAM, 0, 6, ptr->m5);

	/* M4 (CLKSEL_DSS1): CM_CLKSEL_DSS[0:5] */
	sr32(CM_CLKSEL_DSS, 0, 6, ptr->m4);

	/* M3 (CLKSEL_DSS1): CM_CLKSEL_DSS[8:13] */
	sr32(CM_CLKSEL_DSS, 8, 6, ptr->m3);

	/* M2 (DIV_96M): CM_CLKSEL3_PLL[0:4] */
	sr32(CM_CLKSEL3_PLL, 0, 5, ptr->m2);

	/* M (PERIPH_DPLL_MULT): CM_CLKSEL2_PLL[8:19] */
	sr32(CM_CLKSEL2_PLL, 8, 12, ptr->m);

	/* N (PERIPH_DPLL_DIV): CM_CLKSEL2_PLL[0:6] */
	sr32(CM_CLKSEL2_PLL, 0, 7, ptr->n);

	/* M2DIV (CLKSEL_96M): CM_CLKSEL_CORE[12:13] */
	sr32(CM_CLKSEL_CORE, 12, 2, ptr->m2div);

	/* LOCK MODE (EN_PERIPH_DPLL): CM_CLKEN_PLL[16:18] */
	sr32(CM_CLKEN_PLL, 16, 3, PLL_LOCK);
	wait_on_value(BIT1, 2, CM_IDLEST_CKGEN, LDELAY);
}

static void mpu_init_36xx(u32 sil_index, u32 clk_index)
{
	dpll_param *ptr;

	/* Getting the base address to MPU DPLL param table*/
	ptr = (dpll_param *)get_36x_mpu_dpll_param();

	/* Moving it to the right sysclk and ES rev base */
	ptr = ptr + (2*clk_index) + sil_index;

	/* MPU DPLL (unlocked already) */
	/* M2 (MPU_DPLL_CLKOUT_DIV) : CM_CLKSEL2_PLL_MPU[0:4] */
	sr32(CM_CLKSEL2_PLL_MPU, 0, 5, ptr->m2);

	/* M (MPU_DPLL_MULT) : CM_CLKSEL2_PLL_MPU[8:18] */
	sr32(CM_CLKSEL1_PLL_MPU, 8, 11, ptr->m);

	/* N (MPU_DPLL_DIV) : CM_CLKSEL2_PLL_MPU[0:6] */
	sr32(CM_CLKSEL1_PLL_MPU, 0, 7, ptr->n);

	/* LOCK MODE (EN_MPU_DPLL) : CM_CLKEN_PLL_IVA2[0:2] */
	sr32(CM_CLKEN_PLL_MPU, 0, 3, PLL_LOCK);
	wait_on_value(BIT0, 1, CM_IDLEST_PLL_MPU, LDELAY);
}

static void iva_init_36xx(u32 sil_index, u32 clk_index)
{
	dpll_param *ptr;

	/* Getting the base address to IVA DPLL param table*/
	ptr = (dpll_param *)get_36x_iva_dpll_param();

	/* Moving it to the right sysclk and ES rev base */
	ptr = ptr + (2*clk_index) + sil_index;

	/* IVA DPLL */
	/* EN_IVA2_DPLL : CM_CLKEN_PLL_IVA2[0:2] */
	sr32(CM_CLKEN_PLL_IVA2, 0, 3, PLL_STOP);
	wait_on_value(BIT0, 0, CM_IDLEST_PLL_IVA2, LDELAY);

	/* M2 (IVA2_DPLL_CLKOUT_DIV) : CM_CLKSEL2_PLL_IVA2[0:4] */
	sr32(CM_CLKSEL2_PLL_IVA2, 0, 5, ptr->m2);

	/* M (IVA2_DPLL_MULT) : CM_CLKSEL1_PLL_IVA2[8:18] */
	sr32(CM_CLKSEL1_PLL_IVA2, 8, 11, ptr->m);

	/* N (IVA2_DPLL_DIV) : CM_CLKSEL1_PLL_IVA2[0:6] */
	sr32(CM_CLKSEL1_PLL_IVA2, 0, 7, ptr->n);

	/* LOCK MODE (EN_IVA2_DPLL) : CM_CLKEN_PLL_IVA2[0:2] */
	sr32(CM_CLKEN_PLL_IVA2, 0, 3, PLL_LOCK);

	wait_on_value(BIT0, 1, CM_IDLEST_PLL_IVA2, LDELAY);
}


/******************************************************************************
 * prcm_init() - inits clocks for PRCM as defined in clocks.h
 *   -- called from SRAM, or Flash (using temp SRAM stack).
 *****************************************************************************/
void prcm_init(void)
{
	u32 osc_clk=0, sys_clkin_sel;
	u32 clk_index, sil_index;

	/* Gauge the input clock speed and find out the sys_clkin_sel
	 * value corresponding to the input clock.
	 */
	osc_clk = get_osc_clk_speed();
	get_sys_clkin_sel(osc_clk, &sys_clkin_sel);

	sr32(PRM_CLKSEL, 0, 3, sys_clkin_sel); /* set input crystal speed */

	/* If the input clock is greater than 19.2M always divide/2 */
	/*
	 * On OMAP3630, DDR data corruption has been observed on OFF mode
	 * exit if the sys clock was lower than 26M. As a work around,
	 * OMAP3630 is operated at 26M sys clock and this internal division
	 * is not performed.
	 */
	if((is_cpu_family() != CPU_OMAP36XX) && (sys_clkin_sel > 2)) {
		sr32(PRM_CLKSRC_CTRL, 6, 2, 2);/* input clock divider */
		clk_index = sys_clkin_sel/2;
	} else {
		sr32(PRM_CLKSRC_CTRL, 6, 2, 1);/* input clock divider */
		clk_index = sys_clkin_sel;
	}

	if (is_cpu_family() == CPU_OMAP36XX) {
		dpll3_init_36xx(0, clk_index);
		dpll4_init_36xx(0, clk_index);
		mpu_init_36xx(0, clk_index);
		iva_init_36xx(0, clk_index);
	} else {
		sil_index = get_cpu_rev() - 1;

		/* The DPLL tables are defined according to sysclk value and
		 * silicon revision. The clk_index value will be used to get
		 * the values for that input sysclk from the DPLL param table
		 * and sil_index will get the values for that SysClk for the
		 * appropriate silicon rev.
		 */

		/* Unlock MPU DPLL (slows things down, and needed later) */
		sr32(CM_CLKEN_PLL_MPU, 0, 3, PLL_LOW_POWER_BYPASS);
		wait_on_value(BIT0, 0, CM_IDLEST_PLL_MPU, LDELAY);

		dpll3_init_34xx(sil_index, clk_index);
		dpll4_init_34xx(sil_index, clk_index);
		iva_init_34xx(sil_index, clk_index);
		mpu_init_34xx(sil_index, clk_index);

		/* Lock MPU DPLL to set frequency */
		sr32(CM_CLKEN_PLL_MPU, 0, 3, PLL_LOCK);
		wait_on_value(BIT0, 1, CM_IDLEST_PLL_MPU, LDELAY);
	}

	/* Set up GPTimers to sys_clk source only */
 	sr32(CM_CLKSEL_PER, 0, 8, 0xff);
	sr32(CM_CLKSEL_WKUP, 0, 1, 1);

	delay(5000);
}

/*****************************************
 * Routine: secure_unlock
 * Description: Setup security registers for access
 * (GP Device only)
 *****************************************/
void secure_unlock(void)
{
	/* Permission values for registers -Full fledged permissions to all */
	#define UNLOCK_1 0xFFFFFFFF
	#define UNLOCK_2 0x00000000
	#define UNLOCK_3 0x0000FFFF
	/* Protection Module Register Target APE (PM_RT)*/
	__raw_writel(UNLOCK_1, RT_REQ_INFO_PERMISSION_1);
	__raw_writel(UNLOCK_1, RT_READ_PERMISSION_0);
	__raw_writel(UNLOCK_1, RT_WRITE_PERMISSION_0);
	__raw_writel(UNLOCK_2, RT_ADDR_MATCH_1);

	__raw_writel(UNLOCK_3, GPMC_REQ_INFO_PERMISSION_0);
	__raw_writel(UNLOCK_3, GPMC_READ_PERMISSION_0);
	__raw_writel(UNLOCK_3, GPMC_WRITE_PERMISSION_0);

	__raw_writel(UNLOCK_3, OCM_REQ_INFO_PERMISSION_0);
	__raw_writel(UNLOCK_3, OCM_READ_PERMISSION_0);
	__raw_writel(UNLOCK_3, OCM_WRITE_PERMISSION_0);
	__raw_writel(UNLOCK_2, OCM_ADDR_MATCH_2);

	/* IVA Changes */
	__raw_writel(UNLOCK_3, IVA2_REQ_INFO_PERMISSION_0);
	__raw_writel(UNLOCK_3, IVA2_READ_PERMISSION_0);
	__raw_writel(UNLOCK_3, IVA2_WRITE_PERMISSION_0);

	__raw_writel(UNLOCK_1, SMS_RG_ATT0); /* SDRC region 0 public */
}

/**********************************************************
 * Routine: try_unlock_sram()
 * Description: If chip is GP type, unlock the SRAM for
 *  general use.
 ***********************************************************/
void try_unlock_memory(void)
{
	int mode;

	/* if GP device unlock device SRAM for general use */
	/* secure code breaks for Secure/Emulation device - HS/E/T*/
	mode = get_device_type();
	if (mode == GP_DEVICE) {
		secure_unlock();
	}
	return;
}

/**********************************************************
 * Routine: s_init
 * Description: Does early system init of muxing and clocks.
 * - Called at time when only stack is available.
 **********************************************************/
extern void clear_bss(void);

/***********************************************************************
 * get_cs0_size() - get size of chip select 0/1
 ************************************************************************/
u32 get_sdr_cs_size(u32 offset)
{
	u32 size;

	/* get ram size field */
	size = __raw_readl(SDRC_MCFG_0 + offset) >> 8;
	size &= 0x3FF;          /* remove unwanted bits */
	size *= SZ_2M;          /* find size in MB */
	return size;
}

static void init_malloc_pool(void)
{
	unsigned int size;
	void *dram_base;
	unsigned int malloc_size = (2<<20); /* 2MB */

	size = get_sdr_cs_size(SDRC_CS0_OSET);
	size += get_sdr_cs_size(SDRC_CS1_OSET);

	dram_base = (void *)(0x80000000 + size);
	malloc_init(dram_base - malloc_size, malloc_size);
}

void s_init(void)
{
	/* Clear out the BSS */
	clear_bss();

	watchdog_init();
#ifdef CONFIG_3430_AS_3410
	/* setup the scalability control register for
	 * 3430 to work in 3410 mode
	 */
	__raw_writel(0x5ABF,CONTROL_SCALABLE_OMAP_OCP);
#endif
	try_unlock_memory();
	set_muxconf_regs();
	delay(100);
	prcm_init();
	per_clocks_enable();
#if 1
	serial_init();  /* Done to get early debug output; also done later */
#endif

	config_dm3730logic_ddr();

#ifdef CFG_PRINTK_DEBUG
	printk_debug_dump();
#endif

	init_malloc_pool();

	/*
	 * WORKAROUND: To support both Micron and Hynix NAND/DDR parts
	 */
	if ((get_mem_type() == GPMC_NAND) || (get_mem_type() == MMC_NAND))
		nand_init();
}

/*******************************************************
 * Routine: misc_init_r
 * Description: Init ethernet (done here so udelay works)
 ********************************************************/
int misc_init_r (void)
{
#ifdef CFG_PRINTF
	printf("DRAM: %s\n", config_dm3730logic_ddr_type());
#endif
	return(0);
}

/******************************************************
 * Routine: wait_for_command_complete
 * Description: Wait for posting to finish on watchdog
 ******************************************************/
void wait_for_command_complete(unsigned int wd_base)
{
	int pending = 1;
	do {
		pending = __raw_readl(wd_base + WWPS);
	} while (pending);
}

/****************************************
 * Routine: watchdog_init
 * Description: Shut down watch dogs
 *****************************************/
void watchdog_init(void)
{
	/* There are 3 watch dogs WD1=Secure, WD2=MPU, WD3=IVA. WD1 is
	 * either taken care of by ROM (HS/EMU) or not accessible (GP).
	 * We need to take care of WD2-MPU or take a PRCM reset.  WD3
	 * should not be running and does not generate a PRCM reset.
	 */
	sr32(CM_FCLKEN_WKUP, 5, 1, 1);
	sr32(CM_ICLKEN_WKUP, 5, 1, 1);
	wait_on_value(BIT5, 0x20, CM_IDLEST_WKUP, 5); /* some issue here */

	__raw_writel(WD_UNLOCK1, WD2_BASE + WSPR);
	wait_for_command_complete(WD2_BASE);
	__raw_writel(WD_UNLOCK2, WD2_BASE + WSPR);
}

/**********************************************
 * Routine: dram_init
 * Description: sets uboots idea of sdram size
 **********************************************/
int dram_init (void)
{
	return 0;
}

/*****************************************************************
 * Routine: peripheral_enable
 * Description: Enable the clks & power for perifs (GPT2, UART1,...)
 ******************************************************************/
void per_clocks_enable(void)
{
	/* Enable GP2 timer */
	sr32(CM_CLKSEL_PER, 0, 1, 0x1); /* GPT2 = sys clk */
	sr32(CM_ICLKEN_PER, 3, 1, 0x1); /* ICKen GPT2 */
	sr32(CM_FCLKEN_PER, 3, 1, 0x1); /* FCKen GPT2 */

	/* Enable GPIO2-6 clocks */
	sr32(CM_ICLKEN_PER, 13, 5, 0x1F);
	sr32(CM_FCLKEN_PER, 13, 5, 0x1F);

	/* Enable DSS clocks */
	sr32(CM_ICLKEN_DSS, 0, 1, 0x1);
	sr32(CM_FCLKEN_DSS, 0, 2, 0x3);

	/* Enable LCD (McSPI4) clocks */
	sr32(CM_FCLKEN1_CORE, 21, 1, 0x1);
	sr32(CM_ICLKEN1_CORE, 21, 1, 0x1);	

#ifdef CFG_NS16550
	/* Enable UART1 clocks */
	sr32(CM_FCLKEN1_CORE, 13, 1, 0x1);
	sr32(CM_ICLKEN1_CORE, 13, 1, 0x1);
#endif

#ifdef CONFIG_MMC
	/* Enable MMC1 clocks */
	sr32(CM_FCLKEN1_CORE, 24, 1, 0x1);
	sr32(CM_ICLKEN1_CORE, 24, 1, 0x1);
#endif
	delay(1000);
}


/* Setup the polarity of the WAIT0 pin */
void nand_setup_wait(void)
{
	__raw_writel((__raw_readl(GPMC_CONFIG) & ~GPMC_CONFIG_WAIT0POL), GPMC_CONFIG);
}

/* Wait for the NAND chip to become ready by observing WAIT0 (R/nB) go high */
void nand_wait_rdy(void)
{
	int val;
	do {
		val = __raw_readl(GPMC_STATUS);
	} while (!(val & GPMC_STATUS_WAIT0));
}

/**********************************************************
 * Routine: nand+_init
 * Description: Set up nand for nand and jffs2 commands
 *********************************************************/
int nand_init(void)
{
	int ret = 0;

	/* global settings */
	__raw_writel(0x10, GPMC_SYSCONFIG);	/* smart idle */
	__raw_writel(0x0, GPMC_IRQENABLE);	/* isr's sources masked */
	__raw_writel(0, GPMC_TIMEOUT_CONTROL);/* timeout disable */

	/* Set the GPMC Vals . For NAND boot on 3430SDP, NAND is mapped at CS0
         *  , NOR at CS1 and MPDB at CS3. And oneNAND boot, we map oneNAND at CS0.
	 *  We configure only GPMC CS0 with required values. Configiring other devices
	 *  at other CS in done in u-boot anyway. So we don't have to bother doing it here.
         */
	__raw_writel(0 , GPMC_CONFIG7 + GPMC_CONFIG_CS0);
	delay(1000);

#define LOGIC_NAND_GPMC_CONFIG1	0x00001800
#define LOGIC_NAND_GPMC_CONFIG2	0x00090900
#define LOGIC_NAND_GPMC_CONFIG3	0x00090902
#define LOGIC_NAND_GPMC_CONFIG4	0x07020702
#define LOGIC_NAND_GPMC_CONFIG5	0x00080909
#define LOGIC_NAND_GPMC_CONFIG6	0x000002CF
#define LOGIC_NAND_GPMC_CONFIG7	0x00000C70

	if ((get_mem_type() == GPMC_NAND) || (get_mem_type() == MMC_NAND)){
        	__raw_writel( LOGIC_NAND_GPMC_CONFIG1, GPMC_CONFIG1 + GPMC_CONFIG_CS0);
        	__raw_writel( LOGIC_NAND_GPMC_CONFIG2, GPMC_CONFIG2 + GPMC_CONFIG_CS0);
        	__raw_writel( LOGIC_NAND_GPMC_CONFIG3, GPMC_CONFIG3 + GPMC_CONFIG_CS0);
        	__raw_writel( LOGIC_NAND_GPMC_CONFIG4, GPMC_CONFIG4 + GPMC_CONFIG_CS0);
        	__raw_writel( LOGIC_NAND_GPMC_CONFIG5, GPMC_CONFIG5 + GPMC_CONFIG_CS0);
        	__raw_writel( LOGIC_NAND_GPMC_CONFIG6, GPMC_CONFIG6 + GPMC_CONFIG_CS0);

        	/* Enable the GPMC Mapping */
        	__raw_writel(( ((OMAP34XX_GPMC_CS0_SIZE & 0xF)<<8) |
        		     ((NAND_BASE_ADR>>24) & 0x3F) |
        		     (1<<6) ),  (GPMC_CONFIG7 + GPMC_CONFIG_CS0));
        	delay(2000);

		nand_setup_wait();

         	if (nand_chip(NAND_ECC_HW_1BIT|
#ifdef CFG_NAND_4BIT_SW
				NAND_ECC_SW_4BIT
#else
				NAND_ECC_CHIP
#endif
				)){
#ifdef CFG_PRINTF
        		printf("Unsupported Chip!\n");
#endif
        		ret = 1;
			goto out;
        	}

	}

#ifdef ONENAND_START_BLOCK
	if ((get_mem_type() == GPMC_ONENAND) || (get_mem_type() == MMC_ONENAND)){
        	__raw_writel( ONENAND_GPMC_CONFIG1, GPMC_CONFIG1 + GPMC_CONFIG_CS0);
        	__raw_writel( ONENAND_GPMC_CONFIG2, GPMC_CONFIG2 + GPMC_CONFIG_CS0);
        	__raw_writel( ONENAND_GPMC_CONFIG3, GPMC_CONFIG3 + GPMC_CONFIG_CS0);
        	__raw_writel( ONENAND_GPMC_CONFIG4, GPMC_CONFIG4 + GPMC_CONFIG_CS0);
        	__raw_writel( ONENAND_GPMC_CONFIG5, GPMC_CONFIG5 + GPMC_CONFIG_CS0);
        	__raw_writel( ONENAND_GPMC_CONFIG6, GPMC_CONFIG6 + GPMC_CONFIG_CS0);

        	/* Enable the GPMC Mapping */
        	__raw_writel(( ((OMAP34XX_GPMC_CS0_SIZE & 0xF)<<8) |
        		     ((ONENAND_BASE>>24) & 0x3F) |
        		     (1<<6) ),  (GPMC_CONFIG7 + GPMC_CONFIG_CS0));
        	delay(2000);

        	if (onenand_chip()){
#ifdef CFG_PRINTF
        		printf("OneNAND Unsupported !\n");
#endif
        		ret = 1;
			goto out;
        	}
	}
#endif

out:
	serialization_info();
	return ret;
}

typedef int (mmc_boot_addr) (void);
int mmc_boot(unsigned char *buf)
{

       long size = 0;
#ifdef CFG_CMD_FAT
       block_dev_desc_t *dev_desc = NULL;
       unsigned char ret = 0;

       printf("Loading Linux from MMC\n");

       ret = mmc_init(1);
       if(ret == 0){
               printf("\n MMC init failed\n");
               return 0;
       }

       dev_desc = mmc_get_dev(0);
       fat_register_device(dev_desc, 1);
       size = file_fat_read(CFG_LOADFILE, buf, 0);
       if (size == -1) {
               return 0;
       }
       printf("%ld bytes read from MMC to %p\n", size, buf);
#endif
       return size;
}

/* optionally do something like blinking LED */
void board_hang (void)
{ while (0) {};}


/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * M0   - Mode 0
 * The commented string gives the final mux configuration for that pin
 */

/*
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers specific to the
 *		hardware. Many pins need to be moved from protect to primary
 *		mode.
 */

#define USE_UBOOT_MUX

#ifndef USE_UBOOT_MUX

/* Set MUX for UART, GPMC, SDRC, GPIO */

void set_muxconf_regs(void)
{
	MUX_VAL(CP(SDRC_D0),        (IEN  | PTD | DIS | M0)); /*SDRC_D0*/
	MUX_VAL(CP(SDRC_D1),        (IEN  | PTD | DIS | M0)); /*SDRC_D1*/
	MUX_VAL(CP(SDRC_D2),        (IEN  | PTD | DIS | M0)); /*SDRC_D2*/
	MUX_VAL(CP(SDRC_D3),        (IEN  | PTD | DIS | M0)); /*SDRC_D3*/
	MUX_VAL(CP(SDRC_D4),        (IEN  | PTD | DIS | M0)); /*SDRC_D4*/
	MUX_VAL(CP(SDRC_D5),        (IEN  | PTD | DIS | M0)); /*SDRC_D5*/
	MUX_VAL(CP(SDRC_D6),        (IEN  | PTD | DIS | M0)); /*SDRC_D6*/
	MUX_VAL(CP(SDRC_D7),        (IEN  | PTD | DIS | M0)); /*SDRC_D7*/
	MUX_VAL(CP(SDRC_D8),        (IEN  | PTD | DIS | M0)); /*SDRC_D8*/
	MUX_VAL(CP(SDRC_D9),        (IEN  | PTD | DIS | M0)); /*SDRC_D9*/
	MUX_VAL(CP(SDRC_D10),       (IEN  | PTD | DIS | M0)); /*SDRC_D10*/
	MUX_VAL(CP(SDRC_D11),       (IEN  | PTD | DIS | M0)); /*SDRC_D11*/
	MUX_VAL(CP(SDRC_D12),       (IEN  | PTD | DIS | M0)); /*SDRC_D12*/
	MUX_VAL(CP(SDRC_D13),       (IEN  | PTD | DIS | M0)); /*SDRC_D13*/
	MUX_VAL(CP(SDRC_D14),       (IEN  | PTD | DIS | M0)); /*SDRC_D14*/
	MUX_VAL(CP(SDRC_D15),       (IEN  | PTD | DIS | M0)); /*SDRC_D15*/
	MUX_VAL(CP(SDRC_D16),       (IEN  | PTD | DIS | M0)); /*SDRC_D16*/
	MUX_VAL(CP(SDRC_D17),       (IEN  | PTD | DIS | M0)); /*SDRC_D17*/
	MUX_VAL(CP(SDRC_D18),       (IEN  | PTD | DIS | M0)); /*SDRC_D18*/
	MUX_VAL(CP(SDRC_D19),       (IEN  | PTD | DIS | M0)); /*SDRC_D19*/
	MUX_VAL(CP(SDRC_D20),       (IEN  | PTD | DIS | M0)); /*SDRC_D20*/
	MUX_VAL(CP(SDRC_D21),       (IEN  | PTD | DIS | M0)); /*SDRC_D21*/
	MUX_VAL(CP(SDRC_D22),       (IEN  | PTD | DIS | M0)); /*SDRC_D22*/
	MUX_VAL(CP(SDRC_D23),       (IEN  | PTD | DIS | M0)); /*SDRC_D23*/
	MUX_VAL(CP(SDRC_D24),       (IEN  | PTD | DIS | M0)); /*SDRC_D24*/
	MUX_VAL(CP(SDRC_D25),       (IEN  | PTD | DIS | M0)); /*SDRC_D25*/
	MUX_VAL(CP(SDRC_D26),       (IEN  | PTD | DIS | M0)); /*SDRC_D26*/
	MUX_VAL(CP(SDRC_D27),       (IEN  | PTD | DIS | M0)); /*SDRC_D27*/
	MUX_VAL(CP(SDRC_D28),       (IEN  | PTD | DIS | M0)); /*SDRC_D28*/
	MUX_VAL(CP(SDRC_D29),       (IEN  | PTD | DIS | M0)); /*SDRC_D29*/
	MUX_VAL(CP(SDRC_D30),       (IEN  | PTD | DIS | M0)); /*SDRC_D30*/
	MUX_VAL(CP(SDRC_D31),       (IEN  | PTD | DIS | M0)); /*SDRC_D31*/
	MUX_VAL(CP(SDRC_CLK),       (IEN  | PTD | DIS | M0)); /*SDRC_CLK*/
	MUX_VAL(CP(SDRC_DQS0),      (IEN  | PTD | DIS | M0)); /*SDRC_DQS0*/
	MUX_VAL(CP(SDRC_DQS1),      (IEN  | PTD | DIS | M0)); /*SDRC_DQS1*/
	MUX_VAL(CP(SDRC_DQS2),      (IEN  | PTD | DIS | M0)); /*SDRC_DQS2*/
	MUX_VAL(CP(SDRC_DQS3),      (IEN  | PTD | DIS | M0)); /*SDRC_DQS3*/
	MUX_VAL(CP(GPMC_A1),        (IDIS | PTD | DIS | M0)); /*GPMC_A1*/
	MUX_VAL(CP(GPMC_A2),        (IDIS | PTD | DIS | M0)); /*GPMC_A2*/
	MUX_VAL(CP(GPMC_A3),        (IDIS | PTD | DIS | M0)); /*GPMC_A3*/
	MUX_VAL(CP(GPMC_A4),        (IDIS | PTD | DIS | M0)); /*GPMC_A4*/
	MUX_VAL(CP(GPMC_A5),        (IDIS | PTD | DIS | M0)); /*GPMC_A5*/
	MUX_VAL(CP(GPMC_A6),        (IDIS | PTD | DIS | M0)); /*GPMC_A6*/
	MUX_VAL(CP(GPMC_A7),        (IDIS | PTD | DIS | M0)); /*GPMC_A7*/
	MUX_VAL(CP(GPMC_A8),        (IDIS | PTD | DIS | M0)); /*GPMC_A8*/
	MUX_VAL(CP(GPMC_A9),        (IDIS | PTD | DIS | M0)); /*GPMC_A9*/
	MUX_VAL(CP(GPMC_A10),       (IDIS | PTD | DIS | M0)); /*GPMC_A10*/
	MUX_VAL(CP(GPMC_D0),        (IEN  | PTD | DIS | M0)); /*GPMC_D0*/
	MUX_VAL(CP(GPMC_D1),        (IEN  | PTD | DIS | M0)); /*GPMC_D1*/
	MUX_VAL(CP(GPMC_D2),        (IEN  | PTD | DIS | M0)); /*GPMC_D2*/
	MUX_VAL(CP(GPMC_D3),        (IEN  | PTD | DIS | M0)); /*GPMC_D3*/
	MUX_VAL(CP(GPMC_D4),        (IEN  | PTD | DIS | M0)); /*GPMC_D4*/
	MUX_VAL(CP(GPMC_D5),        (IEN  | PTD | DIS | M0)); /*GPMC_D5*/
	MUX_VAL(CP(GPMC_D6),        (IEN  | PTD | DIS | M0)); /*GPMC_D6*/
	MUX_VAL(CP(GPMC_D7),        (IEN  | PTD | DIS | M0)); /*GPMC_D7*/
	MUX_VAL(CP(GPMC_D8),        (IEN  | PTD | DIS | M0)); /*GPMC_D8*/
	MUX_VAL(CP(GPMC_D9),        (IEN  | PTD | DIS | M0)); /*GPMC_D9*/
	MUX_VAL(CP(GPMC_D10),       (IEN  | PTD | DIS | M0)); /*GPMC_D10*/
	MUX_VAL(CP(GPMC_D11),       (IEN  | PTD | DIS | M0)); /*GPMC_D11*/
	MUX_VAL(CP(GPMC_D12),       (IEN  | PTD | DIS | M0)); /*GPMC_D12*/
	MUX_VAL(CP(GPMC_D13),       (IEN  | PTD | DIS | M0)); /*GPMC_D13*/
	MUX_VAL(CP(GPMC_D14),       (IEN  | PTD | DIS | M0)); /*GPMC_D14*/
	MUX_VAL(CP(GPMC_D15),       (IEN  | PTD | DIS | M0)); /*GPMC_D15*/
	MUX_VAL(CP(GPMC_nCS0),      (IDIS | PTU | EN  | M0)); /*GPMC_nCS0*/
	MUX_VAL(CP(GPMC_nCS1),      (IDIS | PTU | EN  | M0)); /*GPMC_nCS1*/
	MUX_VAL(CP(GPMC_nCS2),      (IDIS | PTU | EN  | M0)); /*GPMC_nCS2*/
	MUX_VAL(CP(GPMC_nCS3),      (IDIS | PTU | EN  | M0)); /*GPMC_nCS3*/
	MUX_VAL(CP(GPMC_nCS4),      (IDIS | PTU | EN  | M0)); /*GPMC_nCS4*/
	MUX_VAL(CP(GPMC_nCS5),      (IDIS | PTU | EN  | M0)); /*GPMC_nCS5*/
	MUX_VAL(CP(GPMC_nCS6),      (IDIS | PTU | EN  | M0)); /*GPMC_nCS6*/
	MUX_VAL(CP(GPMC_nCS7),      (IDIS | PTU | EN  | M0)); /*GPMC_nCS7*/
	MUX_VAL(CP(GPMC_CLK),       (IDIS | PTD | DIS | M0)); /*GPMC_CLK*/
	MUX_VAL(CP(GPMC_nADV_ALE),  (IDIS | PTD | DIS | M0)); /*GPMC_nADV_ALE*/
	MUX_VAL(CP(GPMC_nOE),       (IDIS | PTD | DIS | M0)); /*GPMC_nOE*/
	MUX_VAL(CP(GPMC_nWE),       (IDIS | PTD | DIS | M0)); /*GPMC_nWE*/
	MUX_VAL(CP(GPMC_nBE0_CLE),  (IDIS | PTD | DIS | M0)); /*GPMC_nBE0_CLE*/
	MUX_VAL(CP(GPMC_nBE1),      (IDIS | PTD | DIS | M4)); /*GPIO_61*/
	MUX_VAL(CP(GPMC_nWP),       (IEN  | PTD | DIS | M0)); /*GPMC_nWP*/
	MUX_VAL(CP(GPMC_WAIT0),     (IEN  | PTU | EN  | M0)); /*GPMC_WAIT0*/
	MUX_VAL(CP(GPMC_WAIT1),     (IEN  | PTU | EN  | M0)); /*GPMC_WAIT1*/
	MUX_VAL(CP(GPMC_WAIT2),     (IEN  | PTU | EN  | M4)); /*GPIO_64*/
	MUX_VAL(CP(GPMC_WAIT3),     (IEN  | PTU | EN  | M4)); /*GPIO_65*/
	MUX_VAL(CP(DSS_DATA18),     (IEN  | PTD | DIS | M4)); /*GPIO_88*/
	MUX_VAL(CP(DSS_DATA19),     (IEN  | PTD | DIS | M4)); /*GPIO_89*/
	MUX_VAL(CP(DSS_DATA20),     (IEN  | PTD | DIS | M4)); /*GPIO_90*/
	MUX_VAL(CP(DSS_DATA21),     (IEN  | PTD | DIS | M4)); /*GPIO_91*/
	MUX_VAL(CP(CAM_WEN),        (IEN  | PTD | DIS | M4)); /*GPIO_167*/
	MUX_VAL(CP(UART1_TX),       (IDIS | PTD | DIS | M0)); /*UART1_TX*/
	MUX_VAL(CP(UART1_RTS),      (IDIS | PTD | DIS | M0)); /*UART1_RTS*/
	MUX_VAL(CP(UART1_CTS),      (IEN | PTU | DIS | M0)); /*UART1_CTS*/
	MUX_VAL(CP(UART1_RX),       (IEN | PTD | DIS | M0)); /*UART1_RX*/
	MUX_VAL(CP(McBSP1_DX),      (IEN  | PTD | DIS | M4)); /*GPIO_158*/
	MUX_VAL(CP(SYS_32K),        (IEN  | PTD | DIS | M0)); /*SYS_32K*/
	MUX_VAL(CP(SYS_BOOT0),      (IEN  | PTD | DIS | M4)); /*GPIO_2 */
	MUX_VAL(CP(SYS_BOOT1),      (IEN  | PTD | DIS | M4)); /*GPIO_3 */
	MUX_VAL(CP(SYS_BOOT2),      (IEN  | PTD | DIS | M4)); /*GPIO_4 */
	MUX_VAL(CP(SYS_BOOT3),      (IEN  | PTD | DIS | M4)); /*GPIO_5 */
	MUX_VAL(CP(SYS_BOOT4),      (IEN  | PTD | DIS | M4)); /*GPIO_6 */
	MUX_VAL(CP(SYS_BOOT5),      (IEN  | PTD | DIS | M4)); /*GPIO_7 */
	MUX_VAL(CP(SYS_BOOT6),      (IEN  | PTD | DIS | M4)); /*GPIO_8 */
	MUX_VAL(CP(SYS_CLKOUT2),    (IEN  | PTU | EN  | M4)); /*GPIO_186*/
	MUX_VAL(CP(JTAG_nTRST),     (IEN  | PTD | DIS | M0)); /*JTAG_nTRST*/
	MUX_VAL(CP(JTAG_TCK),       (IEN  | PTD | DIS | M0)); /*JTAG_TCK*/
	MUX_VAL(CP(JTAG_TMS),       (IEN  | PTD | DIS | M0)); /*JTAG_TMS*/
	MUX_VAL(CP(JTAG_TDI),       (IEN  | PTD | DIS | M0)); /*JTAG_TDI*/
	MUX_VAL(CP(JTAG_EMU0),      (IEN  | PTD | DIS | M0)); /*JTAG_EMU0*/
	MUX_VAL(CP(JTAG_EMU1),      (IEN  | PTD | DIS | M0)); /*JTAG_EMU1*/
	MUX_VAL(CP(ETK_CLK),        (IEN  | PTD | DIS | M4)); /*GPIO_12*/
	MUX_VAL(CP(ETK_CTL),        (IEN  | PTD | DIS | M4)); /*GPIO_13*/
	MUX_VAL(CP(ETK_D0 ),        (IEN  | PTD | DIS | M4)); /*GPIO_14*/
	MUX_VAL(CP(ETK_D1 ),        (IEN  | PTD | DIS | M4)); /*GPIO_15*/
	MUX_VAL(CP(ETK_D2 ),        (IEN  | PTD | DIS | M4)); /*GPIO_16*/
	MUX_VAL(CP(ETK_D10),        (IEN  | PTD | DIS | M4)); /*GPIO_24*/
	MUX_VAL(CP(ETK_D11),        (IEN  | PTD | DIS | M4)); /*GPIO_25*/
	MUX_VAL(CP(ETK_D12),        (IEN  | PTD | DIS | M4)); /*GPIO_26*/
	MUX_VAL(CP(ETK_D13),        (IEN  | PTD | DIS | M4)); /*GPIO_27*/
	MUX_VAL(CP(ETK_D14),        (IEN  | PTD | DIS | M4)); /*GPIO_28*/
	MUX_VAL(CP(ETK_D15),        (IEN  | PTD | DIS | M4)); /*GPIO_29*/
}

#else  // USE_UBOOT_MUX

#define	SAFE_MODE_PINS_1
#define	SAFE_MODE_PINS_2
#define	SAFE_MODE_PINS_3
#define	SAFE_MODE_PINS_4
#undef	SAFE_MODE_PINS_5
#define	SAFE_MODE_PINS_5A
#define	SAFE_MODE_PINS_6
void set_muxconf_regs(void)
{
 /*SDRC*/
	MUX_VAL(CP(SDRC_D0),		(IEN  | PTD | DIS | M0)); /*SDRC_D0*/
	MUX_VAL(CP(SDRC_D1),		(IEN  | PTD | DIS | M0)); /*SDRC_D1*/
	MUX_VAL(CP(SDRC_D2),		(IEN  | PTD | DIS | M0)); /*SDRC_D2*/
	MUX_VAL(CP(SDRC_D3),		(IEN  | PTD | DIS | M0)); /*SDRC_D3*/
	MUX_VAL(CP(SDRC_D4),		(IEN  | PTD | DIS | M0)); /*SDRC_D4*/
	MUX_VAL(CP(SDRC_D5),		(IEN  | PTD | DIS | M0)); /*SDRC_D5*/
	MUX_VAL(CP(SDRC_D6),		(IEN  | PTD | DIS | M0)); /*SDRC_D6*/
	MUX_VAL(CP(SDRC_D7),		(IEN  | PTD | DIS | M0)); /*SDRC_D7*/
	MUX_VAL(CP(SDRC_D8),		(IEN  | PTD | DIS | M0)); /*SDRC_D8*/
	MUX_VAL(CP(SDRC_D9),		(IEN  | PTD | DIS | M0)); /*SDRC_D9*/
	MUX_VAL(CP(SDRC_D10),		(IEN  | PTD | DIS | M0)); /*SDRC_D10*/
	MUX_VAL(CP(SDRC_D11),		(IEN  | PTD | DIS | M0)); /*SDRC_D11*/
	MUX_VAL(CP(SDRC_D12),		(IEN  | PTD | DIS | M0)); /*SDRC_D12*/
	MUX_VAL(CP(SDRC_D13),		(IEN  | PTD | DIS | M0)); /*SDRC_D13*/
	MUX_VAL(CP(SDRC_D14),		(IEN  | PTD | DIS | M0)); /*SDRC_D14*/
	MUX_VAL(CP(SDRC_D15),		(IEN  | PTD | DIS | M0)); /*SDRC_D15*/
	MUX_VAL(CP(SDRC_D16),		(IEN  | PTD | DIS | M0)); /*SDRC_D16*/
	MUX_VAL(CP(SDRC_D17),		(IEN  | PTD | DIS | M0)); /*SDRC_D17*/
	MUX_VAL(CP(SDRC_D18),		(IEN  | PTD | DIS | M0)); /*SDRC_D18*/
	MUX_VAL(CP(SDRC_D19),		(IEN  | PTD | DIS | M0)); /*SDRC_D19*/
	MUX_VAL(CP(SDRC_D20),		(IEN  | PTD | DIS | M0)); /*SDRC_D20*/
	MUX_VAL(CP(SDRC_D21),		(IEN  | PTD | DIS | M0)); /*SDRC_D21*/
	MUX_VAL(CP(SDRC_D22),		(IEN  | PTD | DIS | M0)); /*SDRC_D22*/
	MUX_VAL(CP(SDRC_D23),		(IEN  | PTD | DIS | M0)); /*SDRC_D23*/
	MUX_VAL(CP(SDRC_D24),		(IEN  | PTD | DIS | M0)); /*SDRC_D24*/
	MUX_VAL(CP(SDRC_D25),		(IEN  | PTD | DIS | M0)); /*SDRC_D25*/
	MUX_VAL(CP(SDRC_D26),		(IEN  | PTD | DIS | M0)); /*SDRC_D26*/
	MUX_VAL(CP(SDRC_D27),		(IEN  | PTD | DIS | M0)); /*SDRC_D27*/
	MUX_VAL(CP(SDRC_D28),		(IEN  | PTD | DIS | M0)); /*SDRC_D28*/
	MUX_VAL(CP(SDRC_D29),		(IEN  | PTD | DIS | M0)); /*SDRC_D29*/
	MUX_VAL(CP(SDRC_D30),		(IEN  | PTD | DIS | M0)); /*SDRC_D30*/
	MUX_VAL(CP(SDRC_D31),		(IEN  | PTD | DIS | M0)); /*SDRC_D31*/
	MUX_VAL(CP(SDRC_CLK),		(IEN  | PTD | DIS | M0)); /*SDRC_CLK*/
	MUX_VAL(CP(SDRC_DQS0),		(IEN  | PTD | DIS | M0)); /*SDRC_DQS0*/
	MUX_VAL(CP(SDRC_DQS1),		(IEN  | PTD | DIS | M0)); /*SDRC_DQS1*/
	MUX_VAL(CP(SDRC_DQS2),		(IEN  | PTD | DIS | M0)); /*SDRC_DQS2*/
	MUX_VAL(CP(SDRC_DQS3),		(IEN  | PTD | DIS | M0)); /*SDRC_DQS3*/
 /*GPMC*/
	MUX_VAL(CP(GPMC_A1),		(IDIS | PTU | EN  | M0)); /*GPMC_A1*/
	MUX_VAL(CP(GPMC_A2),		(IDIS | PTU | EN  | M0)); /*GPMC_A2*/
	MUX_VAL(CP(GPMC_A3),		(IDIS | PTU | EN  | M0)); /*GPMC_A3*/
	MUX_VAL(CP(GPMC_A4),		(IDIS | PTU | EN  | M0)); /*GPMC_A4*/
	MUX_VAL(CP(GPMC_A5),		(IDIS | PTU | EN  | M0)); /*GPMC_A5*/
	MUX_VAL(CP(GPMC_A6),		(IDIS | PTU | EN  | M0)); /*GPMC_A6*/
	MUX_VAL(CP(GPMC_A7),		(IDIS | PTU | EN  | M0)); /*GPMC_A7*/
	MUX_VAL(CP(GPMC_A8),		(IDIS | PTU | EN  | M0)); /*GPMC_A8*/
	MUX_VAL(CP(GPMC_A9),		(IDIS | PTU | EN  | M0)); /*GPMC_A9*/
	MUX_VAL(CP(GPMC_A10),		(IDIS | PTU | EN  | M0)); /*GPMC_A10*/
	MUX_VAL(CP(GPMC_D0),		(IEN  | PTU | EN  | M0)); /*GPMC_D0*/
	MUX_VAL(CP(GPMC_D1),		(IEN  | PTU | EN  | M0)); /*GPMC_D1*/
	MUX_VAL(CP(GPMC_D2),		(IEN  | PTU | EN  | M0)); /*GPMC_D2*/
	MUX_VAL(CP(GPMC_D3),		(IEN  | PTU | EN  | M0)); /*GPMC_D3*/
	MUX_VAL(CP(GPMC_D4),		(IEN  | PTU | EN  | M0)); /*GPMC_D4*/
	MUX_VAL(CP(GPMC_D5),		(IEN  | PTU | EN  | M0)); /*GPMC_D5*/
	MUX_VAL(CP(GPMC_D6),		(IEN  | PTU | EN  | M0)); /*GPMC_D6*/
	MUX_VAL(CP(GPMC_D7),		(IEN  | PTU | EN  | M0)); /*GPMC_D7*/
	MUX_VAL(CP(GPMC_D8),		(IEN  | PTU | EN  | M0)); /*GPMC_D8*/
	MUX_VAL(CP(GPMC_D9),		(IEN  | PTU | EN  | M0)); /*GPMC_D9*/
	MUX_VAL(CP(GPMC_D10),		(IEN  | PTU | EN  | M0)); /*GPMC_D10*/
	MUX_VAL(CP(GPMC_D11),		(IEN  | PTU | EN  | M0)); /*GPMC_D11*/
	MUX_VAL(CP(GPMC_D12),		(IEN  | PTU | EN  | M0)); /*GPMC_D12*/
	MUX_VAL(CP(GPMC_D13),		(IEN  | PTU | EN  | M0)); /*GPMC_D13*/
	MUX_VAL(CP(GPMC_D14),		(IEN  | PTU | EN  | M0)); /*GPMC_D14*/
	MUX_VAL(CP(GPMC_D15),		(IEN  | PTU | EN  | M0)); /*GPMC_D15*/
	MUX_VAL(CP(GPMC_NCS0),		(IDIS | PTU | EN  | M0)); /*GPMC_nCS0*/
	MUX_VAL(CP(GPMC_NCS1),		(IDIS | PTU | EN  | M0)); /*GPMC_nCS1*/
	MUX_VAL(CP(GPMC_NCS2),		(IDIS | PTU | EN  | M0)); /*GPMC_nCS2*/
	MUX_VAL(CP(GPMC_NCS3),		(IDIS | PTD | DIS | M0)); /*GPMC_nCS3*/
	MUX_VAL(CP(GPMC_NCS4),		(IEN  | PTU | EN  | M0)); /*GPMC_nCS4*/
#if 1
	/* Display GPIO */
	MUX_VAL(CP(GPMC_NCS5),          (IDIS | PTU | DIS | M4)); /*GPIO_65 backlight */
#else
	MUX_VAL(CP(GPMC_NCS5),		(IDIS | PTU | EN  | M0)); /*GPMC_nCS5*/
#endif
	MUX_VAL(CP(GPMC_NCS6),		(IEN  | PTD | DIS | M0)); /*GPMC_nCS6*/
	MUX_VAL(CP(GPMC_NCS7),		(IDIS | PTD | DIS | M1)); /*GPMC_IO_DIR*/
	MUX_VAL(CP(GPMC_CLK),		(IEN  | PTD | DIS | M0)); /*GPMC_CLK*/
	MUX_VAL(CP(GPMC_NADV_ALE),	(IDIS | PTD | DIS | M0)); /*GPMC_nADV_ALE*/
	MUX_VAL(CP(GPMC_NOE),		(IDIS | PTD | DIS | M0)); /*GPMC_nOE*/
	MUX_VAL(CP(GPMC_NWE),		(IDIS | PTD | DIS | M0)); /*GPMC_nWE*/
	MUX_VAL(CP(GPMC_NBE0_CLE),	(IDIS | PTU | EN  | M0)); /*GPMC_nBE0_CLE*/
	MUX_VAL(CP(GPMC_NBE1),		(IEN  | PTU | EN  | M0)); /*GPMC_nBE1*/
	MUX_VAL(CP(GPMC_NWP),		(IEN  | PTD | DIS | M0)); /*GPMC_nWP*/
	MUX_VAL(CP(GPMC_WAIT0),		(IEN  | PTU | EN  | M0)); /*GPMC_WAIT0*/
	MUX_VAL(CP(GPMC_WAIT1),		(IEN  | PTU | EN  | M0)); /*GPMC_WAIT1*/
#ifdef SAFE_MODE_PINS_1
	MUX_VAL(CP(GPMC_WAIT2),		(IEN  | PTD | EN  | M7)); /*safe mode */
#else
	MUX_VAL(CP(GPMC_WAIT2),		(IEN  | PTU | EN  | M4)); /*GPIO_64*/
								 /* - ETH_nRESET*/
#endif
	MUX_VAL(CP(GPMC_WAIT3),		(IEN  | PTU | EN  | M0)); /*GPMC_WAIT3*/
 /*DSS*/
#ifdef SAFE_MODE_PINS_2
	MUX_VAL(CP(DSS_PCLK),		(IEN  | PTD | EN  | M7)); /*DSS_PCLK*/
	MUX_VAL(CP(DSS_HSYNC),		(IEN  | PTD | EN  | M7)); /*DSS_HSYNC*/
	MUX_VAL(CP(DSS_VSYNC),		(IEN  | PTD | EN  | M7)); /*DSS_VSYNC*/
	MUX_VAL(CP(DSS_ACBIAS),		(IEN  | PTD | EN  | M7)); /*DSS_ACBIAS*/
	MUX_VAL(CP(DSS_DATA0),		(IEN  | PTD | EN  | M7)); /*DSS_DATA0*/
	MUX_VAL(CP(DSS_DATA1),		(IEN  | PTD | EN  | M7)); /*DSS_DATA1*/
	MUX_VAL(CP(DSS_DATA2),		(IEN  | PTD | EN  | M7)); /*DSS_DATA2*/
	MUX_VAL(CP(DSS_DATA3),		(IEN  | PTD | EN  | M7)); /*DSS_DATA3*/
	MUX_VAL(CP(DSS_DATA4),		(IEN  | PTD | EN  | M7)); /*DSS_DATA4*/
	MUX_VAL(CP(DSS_DATA5),		(IEN  | PTD | EN  | M7)); /*DSS_DATA5*/
	MUX_VAL(CP(DSS_DATA6),		(IEN  | PTD | EN  | M7)); /*DSS_DATA6*/
	MUX_VAL(CP(DSS_DATA7),		(IEN  | PTD | EN  | M7)); /*DSS_DATA7*/
	MUX_VAL(CP(DSS_DATA8),		(IEN  | PTD | EN  | M7)); /*DSS_DATA8*/
	MUX_VAL(CP(DSS_DATA9),		(IEN  | PTD | EN  | M7)); /*DSS_DATA9*/
	MUX_VAL(CP(DSS_DATA10),		(IEN  | PTD | EN  | M7)); /*DSS_DATA10*/
	MUX_VAL(CP(DSS_DATA11),		(IEN  | PTD | EN  | M7)); /*DSS_DATA11*/
	MUX_VAL(CP(DSS_DATA12),		(IEN  | PTD | EN  | M7)); /*DSS_DATA12*/
	MUX_VAL(CP(DSS_DATA13),		(IEN  | PTD | EN  | M7)); /*DSS_DATA13*/
	MUX_VAL(CP(DSS_DATA14),		(IEN  | PTD | EN  | M7)); /*DSS_DATA14*/
	MUX_VAL(CP(DSS_DATA15),		(IEN  | PTD | EN  | M7)); /*DSS_DATA15*/
	MUX_VAL(CP(DSS_DATA16),		(IEN  | PTD | EN  | M7)); /*DSS_DATA16*/
	MUX_VAL(CP(DSS_DATA17),		(IEN  | PTD | EN  | M7)); /*DSS_DATA17*/
	MUX_VAL(CP(DSS_DATA18),		(IEN  | PTD | EN  | M7)); /*DSS_DATA18*/
	MUX_VAL(CP(DSS_DATA19),		(IEN  | PTD | EN  | M7)); /*DSS_DATA19*/
	MUX_VAL(CP(DSS_DATA20),		(IEN  | PTD | EN  | M7)); /*DSS_DATA20*/
	MUX_VAL(CP(DSS_DATA21),		(IEN  | PTD | EN  | M7)); /*DSS_DATA21*/
	MUX_VAL(CP(DSS_DATA22),		(IEN  | PTD | EN  | M7)); /*DSS_DATA22*/
	MUX_VAL(CP(DSS_DATA23),		(IEN  | PTD | EN  | M7)); /*DSS_DATA23*/
 /*CAMERA*/
	MUX_VAL(CP(CAM_HS),		(IEN  | PTD | EN  | M7)); /*CAM_HS */
	MUX_VAL(CP(CAM_VS),		(IEN  | PTD | EN  | M7)); /*CAM_VS */
	MUX_VAL(CP(CAM_XCLKA),		(IEN  | PTD | EN  | M7)); /*CAM_XCLKA*/
	MUX_VAL(CP(CAM_PCLK),		(IEN  | PTD | EN  | M7)); /*CAM_PCLK*/
	MUX_VAL(CP(CAM_FLD),		(IEN  | PTD | EN  | M7)); /*GPIO_98*/
								 /* - CAM_RESET*/
	MUX_VAL(CP(CAM_D0),		(IEN  | PTD | EN  | M7)); /*CAM_D0*/
	MUX_VAL(CP(CAM_D1),		(IEN  | PTD | EN  | M7)); /*CAM_D1*/
	MUX_VAL(CP(CAM_D2),		(IEN  | PTD | EN  | M7)); /*CAM_D2*/
	MUX_VAL(CP(CAM_D3),		(IEN  | PTD | EN  | M7)); /*CAM_D3*/
	MUX_VAL(CP(CAM_D4),		(IEN  | PTD | EN  | M7)); /*CAM_D4*/
	MUX_VAL(CP(CAM_D5),		(IEN  | PTD | EN  | M7)); /*CAM_D5*/
	MUX_VAL(CP(CAM_D6),		(IEN  | PTD | EN  | M7)); /*CAM_D6*/
	MUX_VAL(CP(CAM_D7),		(IEN  | PTD | EN  | M7)); /*CAM_D7*/
	MUX_VAL(CP(CAM_D8),		(IEN  | PTD | EN  | M7)); /*CAM_D8*/
	MUX_VAL(CP(CAM_D9),		(IEN  | PTD | EN  | M7)); /*CAM_D9*/
	MUX_VAL(CP(CAM_D10),		(IEN  | PTD | EN  | M7)); /*CAM_D10*/
	MUX_VAL(CP(CAM_D11),		(IEN  | PTD | EN  | M7)); /*CAM_D11*/
	MUX_VAL(CP(CAM_XCLKB),		(IEN  | PTD | EN  | M7)); /*CAM_XCLKB*/
	MUX_VAL(CP(CAM_WEN),		(IEN  | PTD | EN  | M7)); /*GPIO_167*/
	MUX_VAL(CP(CAM_STROBE),		(IEN  | PTD | EN  | M7)); /*CAM_STROBE*/
	MUX_VAL(CP(CSI2_DX0),		(IEN  | PTD | EN  | M7)); /*CSI2_DX0*/
	MUX_VAL(CP(CSI2_DY0),		(IEN  | PTD | EN  | M7)); /*CSI2_DY0*/
	MUX_VAL(CP(CSI2_DX1),		(IEN  | PTD | EN  | M7)); /*CSI2_DX1*/
	MUX_VAL(CP(CSI2_DY1),		(IEN  | PTD | EN  | M7)); /*CSI2_DY1*/
 /*Audio Interface */
	MUX_VAL(CP(MCBSP2_FSX),		(IEN  | PTD | EN  | M7)); /*McBSP2_FSX*/
	MUX_VAL(CP(MCBSP2_CLKX),	(IEN  | PTD | EN  | M7)); /*McBSP2_CLKX*/
	MUX_VAL(CP(MCBSP2_DR),		(IEN  | PTD | EN  | M7)); /*McBSP2_DR*/
	MUX_VAL(CP(MCBSP2_DX),		(IEN  | PTD | EN  | M7)); /*McBSP2_DX*/
#else
 /*CAMERA*/
	MUX_VAL(CP(CAM_HS),		(IEN  | PTD | EN  | M0)); /*CAM_HS */
	MUX_VAL(CP(CAM_VS),		(IEN  | PTD | EN  | M0)); /*CAM_VS */
	MUX_VAL(CP(CAM_XCLKA),		(IDIS | PTD | EN  | M0)); /*CAM_XCLKA*/
	MUX_VAL(CP(CAM_PCLK),		(IEN  | PTD | EN  | M0)); /*CAM_PCLK*/
	MUX_VAL(CP(CAM_FLD),		(IDIS | PTD | EN  | M4)); /*GPIO_98*/
								 /* - CAM_RESET*/
	MUX_VAL(CP(CAM_D0),		(IEN  | PTD | EN  | M0)); /*CAM_D0*/
	MUX_VAL(CP(CAM_D1),		(IEN  | PTD | EN  | M0)); /*CAM_D1*/
	MUX_VAL(CP(CAM_D2),		(IEN  | PTD | EN  | M0)); /*CAM_D2*/
	MUX_VAL(CP(CAM_D3),		(IEN  | PTD | EN  | M0)); /*CAM_D3*/
	MUX_VAL(CP(CAM_D4),		(IEN  | PTD | EN  | M0)); /*CAM_D4*/
	MUX_VAL(CP(CAM_D5),		(IEN  | PTD | EN  | M0)); /*CAM_D5*/
	MUX_VAL(CP(CAM_D6),		(IEN  | PTD | EN  | M0)); /*CAM_D6*/
	MUX_VAL(CP(CAM_D7),		(IEN  | PTD | EN  | M0)); /*CAM_D7*/
	MUX_VAL(CP(CAM_D8),		(IEN  | PTD | EN  | M0)); /*CAM_D8*/
	MUX_VAL(CP(CAM_D9),		(IEN  | PTD | EN  | M0)); /*CAM_D9*/
	MUX_VAL(CP(CAM_D10),		(IEN  | PTD | EN  | M0)); /*CAM_D10*/
	MUX_VAL(CP(CAM_D11),		(IEN  | PTD | EN  | M0)); /*CAM_D11*/
	MUX_VAL(CP(CAM_XCLKB),		(IDIS | PTD | EN  | M0)); /*CAM_XCLKB*/
	MUX_VAL(CP(CAM_WEN),		(IEN  | PTD | EN  | M4)); /*GPIO_167*/
	MUX_VAL(CP(CAM_STROBE),		(IDIS | PTD | EN  | M0)); /*CAM_STROBE*/
	MUX_VAL(CP(CSI2_DX0),		(IEN  | PTD | EN  | M0)); /*CSI2_DX0*/
	MUX_VAL(CP(CSI2_DY0),		(IEN  | PTD | EN  | M0)); /*CSI2_DY0*/
	MUX_VAL(CP(CSI2_DX1),		(IEN  | PTD | EN  | M0)); /*CSI2_DX1*/
	MUX_VAL(CP(CSI2_DY1),		(IEN  | PTD | EN  | M0)); /*CSI2_DY1*/
 /*Audio Interface */
	MUX_VAL(CP(MCBSP2_FSX),		(IEN  | PTD | EN  | M0)); /*McBSP2_FSX*/
	MUX_VAL(CP(MCBSP2_CLKX),	(IEN  | PTD | EN  | M0)); /*McBSP2_CLKX*/
	MUX_VAL(CP(MCBSP2_DR),		(IEN  | PTD | EN  | M0)); /*McBSP2_DR*/
	MUX_VAL(CP(MCBSP2_DX),		(IDIS | PTD | EN  | M0)); /*McBSP2_DX*/
#endif

#if 0 /* Setup in lcd_setup_pinmux() */
	MUX_VAL(CP(DSS_PCLK),		(IDIS | PTD | EN  | M0)); /*DSS_PCLK*/
	MUX_VAL(CP(DSS_HSYNC),		(IDIS | PTD | EN  | M0)); /*DSS_HSYNC*/
	MUX_VAL(CP(DSS_VSYNC),		(IDIS | PTD | EN  | M0)); /*DSS_VSYNC*/
	MUX_VAL(CP(DSS_ACBIAS),		(IDIS | PTD | EN  | M0)); /*DSS_ACBIAS*/
#if 1
 /*DSS - with DATA18-23 muxed as DATA0-5 */
	MUX_VAL(CP(DSS_PCLK),		(IDIS | PTD | EN  | M0)); /*DSS_PCLK*/
	MUX_VAL(CP(DSS_HSYNC),		(IDIS | PTD | EN  | M0)); /*DSS_HSYNC*/
	MUX_VAL(CP(DSS_VSYNC),		(IDIS | PTD | EN  | M0)); /*DSS_VSYNC*/
	MUX_VAL(CP(DSS_ACBIAS),		(IDIS | PTD | EN  | M0)); /*DSS_ACBIAS*/
#if 1
	/* SOM used DATA0-5 for output */
	MUX_VAL(CP(DSS_DATA0),		(IDIS | PTD | EN  | M0)); /*DSS_DATA0*/
	MUX_VAL(CP(DSS_DATA1),		(IDIS | PTD | EN  | M0)); /*DSS_DATA1*/
	MUX_VAL(CP(DSS_DATA2),		(IDIS | PTD | EN  | M0)); /*DSS_DATA2*/
	MUX_VAL(CP(DSS_DATA3),		(IDIS | PTD | EN  | M0)); /*DSS_DATA3*/
	MUX_VAL(CP(DSS_DATA4),		(IDIS | PTD | EN  | M0)); /*DSS_DATA4*/
	MUX_VAL(CP(DSS_DATA5),		(IDIS | PTD | EN  | M0)); /*DSS_DATA5*/
#else
	/* Torpedo doesn't used DATA0-5 for output */
	MUX_VAL(CP(DSS_DATA0),		(IDIS | PTD | EN  | M7)); /*SAFE*/
	MUX_VAL(CP(DSS_DATA1),		(IDIS | PTD | EN  | M7)); /*SAFE*/
	MUX_VAL(CP(DSS_DATA2),		(IDIS | PTD | EN  | M7)); /*SAFE*/
	MUX_VAL(CP(DSS_DATA3),		(IDIS | PTD | EN  | M7)); /*SAFE*/
	MUX_VAL(CP(DSS_DATA4),		(IDIS | PTD | EN  | M7)); /*SAFE*/
	MUX_VAL(CP(DSS_DATA5),		(IDIS | PTD | EN  | M7)); /*SAFE*/
#endif
	MUX_VAL(CP(DSS_DATA6),		(IDIS | PTD | EN  | M0)); /*DSS_DATA6*/
	MUX_VAL(CP(DSS_DATA7),		(IDIS | PTD | EN  | M0)); /*DSS_DATA7*/
	MUX_VAL(CP(DSS_DATA8),		(IDIS | PTD | EN  | M0)); /*DSS_DATA8*/
	MUX_VAL(CP(DSS_DATA9),		(IDIS | PTD | EN  | M0)); /*DSS_DATA9*/
	MUX_VAL(CP(DSS_DATA10),		(IDIS | PTD | EN  | M0)); /*DSS_DATA10*/
	MUX_VAL(CP(DSS_DATA11),		(IDIS | PTD | EN  | M0)); /*DSS_DATA11*/
	MUX_VAL(CP(DSS_DATA12),		(IDIS | PTD | EN  | M0)); /*DSS_DATA12*/
	MUX_VAL(CP(DSS_DATA13),		(IDIS | PTD | EN  | M0)); /*DSS_DATA13*/
	MUX_VAL(CP(DSS_DATA14),		(IDIS | PTD | EN  | M0)); /*DSS_DATA14*/
	MUX_VAL(CP(DSS_DATA15),		(IDIS | PTD | EN  | M0)); /*DSS_DATA15*/
	MUX_VAL(CP(DSS_DATA16),		(IDIS | PTD | EN  | M0)); /*DSS_DATA16*/
	MUX_VAL(CP(DSS_DATA17),		(IDIS | PTD | EN  | M0)); /*DSS_DATA17*/
#if 1
	/* SOM uses DATA18-23 as they are*/
	MUX_VAL(CP(DSS_DATA18),		(IDIS | PTD | EN  | M0)); /*DSS_DATA18*/
	MUX_VAL(CP(DSS_DATA19),		(IDIS | PTD | EN  | M0)); /*DSS_DATA19*/
	MUX_VAL(CP(DSS_DATA20),		(IDIS | PTD | EN  | M0)); /*DSS_DATA20*/
	MUX_VAL(CP(DSS_DATA21),		(IDIS | PTD | EN  | M0)); /*DSS_DATA21*/
	MUX_VAL(CP(DSS_DATA22),		(IDIS | PTD | EN  | M0)); /*DSS_DATA22*/
	MUX_VAL(CP(DSS_DATA23),		(IDIS | PTD | EN  | M0)); /*DSS_DATA23*/
#else
	/* Torpedo uses DATA18-23 as DATA0-5 */
	MUX_VAL(CP(DSS_DATA18),		(IDIS | PTD | EN  | M3)); /*DSS_DATA0*/
	MUX_VAL(CP(DSS_DATA19),		(IDIS | PTD | EN  | M3)); /*DSS_DATA1*/
	MUX_VAL(CP(DSS_DATA20),		(IDIS | PTD | EN  | M3)); /*DSS_DATA2*/
	MUX_VAL(CP(DSS_DATA21),		(IDIS | PTD | EN  | M3)); /*DSS_DATA3*/
	MUX_VAL(CP(DSS_DATA22),		(IDIS | PTD | EN  | M3)); /*DSS_DATA4*/
	MUX_VAL(CP(DSS_DATA23),		(IDIS | PTD | EN  | M3)); /*DSS_DATA5*/
#endif
#else
	MUX_VAL(CP(DSS_DATA0),		(IDIS | PTD | EN  | M0)); /*DSS_DATA0*/
	MUX_VAL(CP(DSS_DATA1),		(IDIS | PTD | EN  | M0)); /*DSS_DATA1*/
	MUX_VAL(CP(DSS_DATA2),		(IDIS | PTD | EN  | M0)); /*DSS_DATA2*/
	MUX_VAL(CP(DSS_DATA3),		(IDIS | PTD | EN  | M0)); /*DSS_DATA3*/
	MUX_VAL(CP(DSS_DATA4),		(IDIS | PTD | EN  | M0)); /*DSS_DATA4*/
	MUX_VAL(CP(DSS_DATA5),		(IDIS | PTD | EN  | M0)); /*DSS_DATA5*/
	MUX_VAL(CP(DSS_DATA6),		(IDIS | PTD | EN  | M0)); /*DSS_DATA6*/
	MUX_VAL(CP(DSS_DATA7),		(IDIS | PTD | EN  | M0)); /*DSS_DATA7*/
	MUX_VAL(CP(DSS_DATA8),		(IDIS | PTD | EN  | M0)); /*DSS_DATA8*/
	MUX_VAL(CP(DSS_DATA9),		(IDIS | PTD | EN  | M0)); /*DSS_DATA9*/
	MUX_VAL(CP(DSS_DATA10),		(IDIS | PTD | EN  | M0)); /*DSS_DATA10*/
	MUX_VAL(CP(DSS_DATA11),		(IDIS | PTD | EN  | M0)); /*DSS_DATA11*/
	MUX_VAL(CP(DSS_DATA12),		(IDIS | PTD | EN  | M0)); /*DSS_DATA12*/
	MUX_VAL(CP(DSS_DATA13),		(IDIS | PTD | EN  | M0)); /*DSS_DATA13*/
	MUX_VAL(CP(DSS_DATA14),		(IDIS | PTD | EN  | M0)); /*DSS_DATA14*/
	MUX_VAL(CP(DSS_DATA15),		(IDIS | PTD | EN  | M0)); /*DSS_DATA15*/
	MUX_VAL(CP(DSS_DATA16),		(IDIS | PTD | EN  | M0)); /*DSS_DATA16*/
	MUX_VAL(CP(DSS_DATA17),		(IDIS | PTD | EN  | M0)); /*DSS_DATA17*/
	MUX_VAL(CP(DSS_DATA18),		(IDIS | PTD | EN  | M0)); /*DSS_DATA18*/
	MUX_VAL(CP(DSS_DATA19),		(IDIS | PTD | EN  | M0)); /*DSS_DATA19*/
	MUX_VAL(CP(DSS_DATA20),		(IDIS | PTD | EN  | M0)); /*DSS_DATA20*/
	MUX_VAL(CP(DSS_DATA21),		(IDIS | PTD | EN  | M0)); /*DSS_DATA21*/
	MUX_VAL(CP(DSS_DATA22),		(IDIS | PTD | EN  | M0)); /*DSS_DATA22*/
	MUX_VAL(CP(DSS_DATA23),		(IDIS | PTD | EN  | M0)); /*DSS_DATA23*/
#endif
#endif

 /*Expansion card  */
	MUX_VAL(CP(MMC1_CLK),		(IDIS | PTU | EN  | M0)); /*MMC1_CLK*/
	MUX_VAL(CP(MMC1_CMD),		(IEN  | PTU | EN  | M0)); /*MMC1_CMD*/
	MUX_VAL(CP(MMC1_DAT0),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT0*/
	MUX_VAL(CP(MMC1_DAT1),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT1*/
	MUX_VAL(CP(MMC1_DAT2),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT2*/
	MUX_VAL(CP(MMC1_DAT3),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT3*/
#ifdef SAFE_MODE_PINS_3
	MUX_VAL(CP(MMC1_DAT4),		(IEN  | PTD | EN  | M7)); /*MMC1_DAT4*/
	MUX_VAL(CP(MMC1_DAT5),		(IEN  | PTD | EN  | M7)); /*MMC1_DAT5*/
	MUX_VAL(CP(MMC1_DAT6),		(IEN  | PTD | EN  | M7)); /*MMC1_DAT6*/
	MUX_VAL(CP(MMC1_DAT7),		(IEN  | PTD | EN  | M7)); /*MMC1_DAT7*/
 /*Wireless LAN */
	MUX_VAL(CP(MMC2_CLK),		(IDIS | PTD | EN  | M4)); /*GPIO_130*/
	MUX_VAL(CP(MMC2_CMD),		(IEN  | PTD | EN  | M7)); /*MMC2_CMD*/
	MUX_VAL(CP(MMC2_DAT0),		(IEN  | PTD | EN  | M7)); /*MMC2_DAT0*/
	MUX_VAL(CP(MMC2_DAT1),		(IEN  | PTD | EN  | M7)); /*MMC2_DAT1*/
	MUX_VAL(CP(MMC2_DAT2),		(IEN  | PTD | EN  | M7)); /*MMC2_DAT2*/
	MUX_VAL(CP(MMC2_DAT3),		(IEN  | PTD | EN  | M7)); /*MMC2_DAT3*/
	MUX_VAL(CP(MMC2_DAT4),		(IEN  | PTD | EN  | M7)); /*MMC2_DAT4*/
	MUX_VAL(CP(MMC2_DAT5),		(IEN  | PTD | EN  | M7)); /*MMC2_DAT5*/
	MUX_VAL(CP(MMC2_DAT6),		(IEN  | PTD | EN  | M7)); /*MMC2_DAT6 */
	MUX_VAL(CP(MMC2_DAT7),		(IEN  | PTD | EN  | M7)); /*MMC2_DAT7*/
 /*Bluetooth*/
	MUX_VAL(CP(MCBSP3_DX),		(IEN  | PTD | EN  | M7)); /*McBSP3_DX*/
	MUX_VAL(CP(MCBSP3_DR),		(IEN  | PTD | EN  | M7)); /*McBSP3_DR*/
	MUX_VAL(CP(MCBSP3_CLKX),	(IEN  | PTD | EN  | M7)); /*McBSP3_CLKX  */
	MUX_VAL(CP(MCBSP3_FSX),		(IEN  | PTD | EN  | M7)); /*McBSP3_FSX*/
	MUX_VAL(CP(UART2_CTS),		(IEN  | PTD | EN  | M7)); /*UART2_CTS*/
	MUX_VAL(CP(UART2_RTS),		(IEN  | PTD | EN  | M7)); /*UART2_RTS*/
	MUX_VAL(CP(UART2_TX),		(IEN  | PTD | EN  | M7)); /*UART2_TX*/
	MUX_VAL(CP(UART2_RX),		(IEN  | PTD | EN  | M7)); /*UART2_RX*/
#else
	MUX_VAL(CP(MMC1_DAT4),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT4*/
	MUX_VAL(CP(MMC1_DAT5),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT5*/
	MUX_VAL(CP(MMC1_DAT6),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT6*/
	MUX_VAL(CP(MMC1_DAT7),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT7*/
 /*Wireless LAN */
	MUX_VAL(CP(MMC2_CLK),		(IEN  | PTD | DIS | M0)); /*MMC2_CLK*/
	MUX_VAL(CP(MMC2_CMD),		(IEN  | PTU | EN  | M0)); /*MMC2_CMD*/
	MUX_VAL(CP(MMC2_DAT0),		(IEN  | PTU | EN  | M0)); /*MMC2_DAT0*/
	MUX_VAL(CP(MMC2_DAT1),		(IEN  | PTU | EN  | M0)); /*MMC2_DAT1*/
	MUX_VAL(CP(MMC2_DAT2),		(IEN  | PTU | EN  | M0)); /*MMC2_DAT2*/
	MUX_VAL(CP(MMC2_DAT3),		(IEN  | PTU | EN  | M0)); /*MMC2_DAT3*/
	MUX_VAL(CP(MMC2_DAT4),		(IDIS | PTD | DIS | M0)); /*MMC2_DAT4*/
	MUX_VAL(CP(MMC2_DAT5),		(IDIS | PTD | DIS | M0)); /*MMC2_DAT5*/
	MUX_VAL(CP(MMC2_DAT6),		(IDIS | PTD | DIS | M0)); /*MMC2_DAT6 */
	MUX_VAL(CP(MMC2_DAT7),		(IEN  | PTU | EN  | M0)); /*MMC2_DAT7*/
 /*Bluetooth*/
	MUX_VAL(CP(MCBSP3_DX),		(IDIS | PTD | DIS | M0)); /*McBSP3_DX*/
	MUX_VAL(CP(MCBSP3_DR),		(IEN  | PTD | DIS | M0)); /*McBSP3_DR*/
	MUX_VAL(CP(MCBSP3_CLKX),	(IEN  | PTD | DIS | M0)); /*McBSP3_CLKX  */
	MUX_VAL(CP(MCBSP3_FSX),		(IEN  | PTD | DIS | M0)); /*McBSP3_FSX*/
	MUX_VAL(CP(UART2_CTS),		(IEN  | PTU | EN  | M0)); /*UART2_CTS*/
	MUX_VAL(CP(UART2_RTS),		(IDIS | PTD | DIS | M0)); /*UART2_RTS*/
	MUX_VAL(CP(UART2_TX),		(IDIS | PTD | DIS | M0)); /*UART2_TX*/
	MUX_VAL(CP(UART2_RX),		(IEN  | PTD | DIS | M0)); /*UART2_RX*/
#endif
 /*Modem Interface */
	MUX_VAL(CP(UART1_TX),		(IDIS | PTD | DIS | M0)); /*UART1_TX*/
	MUX_VAL(CP(UART1_RTS),		(IDIS | PTD | DIS | M0)); /*UART1_RTS*/
	MUX_VAL(CP(UART1_CTS),		(IEN  | PTU | DIS | M0)); /*UART1_CTS*/
	MUX_VAL(CP(UART1_RX),		(IEN  | PTD | DIS | M0)); /*UART1_RX*/
#ifdef SAFE_MODE_PINS_4
	MUX_VAL(CP(MCBSP4_CLKX),	(IEN  | PTD | EN  | M7)); /*GPIO_152*/
								 /* - LCD_INI*/
	MUX_VAL(CP(MCBSP4_DR),		(IEN  | PTD | EN  | M7)); /*GPIO_153*/
								 /* - LCD_ENVDD */
#if 1
#if 1
	/* SOM doesn't use GPIO_154 for backlight pwr */
	MUX_VAL(CP(MCBSP4_DX),		(IDIS | PTD | EN  | M7)); /*GPIO_154*/
#else
	MUX_VAL(CP(MCBSP4_DX),		(IDIS | PTD | EN  | M4)); /*GPIO_154*/
#endif
	MUX_VAL(CP(MCBSP4_FSX),		(IDIS | PTD | EN  | M4)); /*GPIO_155*/
#else
	MUX_VAL(CP(MCBSP4_DX),		(IEN  | PTD | EN  | M7)); /*GPIO_154*/
								 /* - LCD_QVGA/nVGA */
	MUX_VAL(CP(MCBSP4_FSX),		(IEN  | PTD | EN  | M7)); /*GPIO_155*/
								 /* - LCD_RESB */
#endif

	MUX_VAL(CP(MCBSP1_CLKR),	(IEN  | PTD | EN  | M7)); /*MCBSP1_CLKR  */
	MUX_VAL(CP(MCBSP1_FSR),		(IEN  | PTD | EN  | M7)); /*MCBSP1_FSR*/
	MUX_VAL(CP(MCBSP1_DX),		(IEN  | PTD | EN  | M7)); /*MCBSP1_DX*/
	MUX_VAL(CP(MCBSP1_DR),		(IEN  | PTD | EN  | M7)); /*MCBSP1_DR*/
	MUX_VAL(CP(MCBSP_CLKS),		(IEN  | PTD | EN  | M7)); /*MCBSP_CLKS  */
	MUX_VAL(CP(MCBSP1_FSX),		(IEN  | PTD | EN  | M7)); /*MCBSP1_FSX*/
	MUX_VAL(CP(MCBSP1_CLKX),	(IEN  | PTD | EN  | M7)); /*MCBSP1_CLKX  */
 /*Serial Interface*/
	MUX_VAL(CP(UART3_CTS_RCTX),	(IEN  | PTD | EN  | M7)); /*UART3_CTS_*/
								 /* RCTX*/
	MUX_VAL(CP(UART3_RTS_SD),	(IEN  | PTD | EN  | M7)); /*UART3_RTS_SD */
	MUX_VAL(CP(UART3_RX_IRRX),	(IEN  | PTD | EN  | M7)); /*UART3_RX_IRRX*/
	MUX_VAL(CP(UART3_TX_IRTX),	(IEN  | PTD | EN  | M7)); /*UART3_TX_IRTX*/
	MUX_VAL(CP(HSUSB0_CLK),		(IEN  | PTD | EN  | M7)); /*HSUSB0_CLK*/
	MUX_VAL(CP(HSUSB0_STP),		(IEN  | PTD | EN  | M7)); /*HSUSB0_STP*/
	MUX_VAL(CP(HSUSB0_DIR),		(IEN  | PTD | EN  | M7)); /*HSUSB0_DIR*/
	MUX_VAL(CP(HSUSB0_NXT),		(IEN  | PTD | EN  | M7)); /*HSUSB0_NXT*/
	MUX_VAL(CP(HSUSB0_DATA0),	(IEN  | PTD | EN  | M7)); /*HSUSB0_DATA0*/
	MUX_VAL(CP(HSUSB0_DATA1),	(IEN  | PTD | EN  | M7)); /*HSUSB0_DATA1*/
	MUX_VAL(CP(HSUSB0_DATA2),	(IEN  | PTD | EN  | M7)); /*HSUSB0_DATA2*/
	MUX_VAL(CP(HSUSB0_DATA3),	(IEN  | PTD | EN  | M7)); /*HSUSB0_DATA3*/
	MUX_VAL(CP(HSUSB0_DATA4),	(IEN  | PTD | EN  | M7)); /*HSUSB0_DATA4*/
	MUX_VAL(CP(HSUSB0_DATA5),	(IEN  | PTD | EN  | M7)); /*HSUSB0_DATA5*/
	MUX_VAL(CP(HSUSB0_DATA6),	(IEN  | PTD | EN  | M7)); /*HSUSB0_DATA6*/
	MUX_VAL(CP(HSUSB0_DATA7),	(IEN  | PTD | EN  | M7)); /*HSUSB0_DATA7*/
#else
	MUX_VAL(CP(MCBSP4_CLKX),	(IDIS | PTD | DIS | M4)); /*GPIO_152*/
								 /* - LCD_INI*/
	MUX_VAL(CP(MCBSP4_DR),		(IDIS | PTD | DIS | M4)); /*GPIO_153*/
								 /* - LCD_ENVDD */
	MUX_VAL(CP(MCBSP4_DX),		(IDIS | PTD | DIS | M4)); /*GPIO_154*/
								 /* - LCD_QVGA/nVGA */
	MUX_VAL(CP(MCBSP4_FSX),		(IDIS | PTD | DIS | M4)); /*GPIO_155*/
								 /* - LCD_RESB */
	MUX_VAL(CP(MCBSP1_CLKR),	(IEN  | PTD | DIS | M0)); /*MCBSP1_CLKR  */
	MUX_VAL(CP(MCBSP1_FSR),		(IDIS | PTU | EN  | M0)); /*MCBSP1_FSR*/
	MUX_VAL(CP(MCBSP1_DX),		(IDIS | PTD | DIS | M0)); /*MCBSP1_DX*/
	MUX_VAL(CP(MCBSP1_DR),		(IEN  | PTD | DIS | M0)); /*MCBSP1_DR*/
	MUX_VAL(CP(MCBSP_CLKS),		(IEN  | PTU | DIS | M0)); /*MCBSP_CLKS  */
	MUX_VAL(CP(MCBSP1_FSX),		(IEN  | PTD | DIS | M0)); /*MCBSP1_FSX*/
	MUX_VAL(CP(MCBSP1_CLKX),	(IEN  | PTD | DIS | M0)); /*MCBSP1_CLKX  */
 /*Serial Interface*/
	MUX_VAL(CP(UART3_CTS_RCTX),	(IEN  | PTD | EN  | M0)); /*UART3_CTS_*/
								 /* RCTX*/
	MUX_VAL(CP(UART3_RTS_SD),	(IDIS | PTD | DIS | M0)); /*UART3_RTS_SD */
	MUX_VAL(CP(UART3_RX_IRRX),	(IEN  | PTD | DIS | M0)); /*UART3_RX_IRRX*/
	MUX_VAL(CP(UART3_TX_IRTX),	(IDIS | PTD | DIS | M0)); /*UART3_TX_IRTX*/
	MUX_VAL(CP(HSUSB0_CLK),		(IEN  | PTD | DIS | M0)); /*HSUSB0_CLK*/
	MUX_VAL(CP(HSUSB0_STP),		(IDIS | PTU | EN  | M0)); /*HSUSB0_STP*/
	MUX_VAL(CP(HSUSB0_DIR),		(IEN  | PTD | DIS | M0)); /*HSUSB0_DIR*/
	MUX_VAL(CP(HSUSB0_NXT),		(IEN  | PTD | DIS | M0)); /*HSUSB0_NXT*/
	MUX_VAL(CP(HSUSB0_DATA0),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA0*/
	MUX_VAL(CP(HSUSB0_DATA1),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA1*/
	MUX_VAL(CP(HSUSB0_DATA2),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA2*/
	MUX_VAL(CP(HSUSB0_DATA3),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA3*/
	MUX_VAL(CP(HSUSB0_DATA4),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA4*/
	MUX_VAL(CP(HSUSB0_DATA5),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA5*/
	MUX_VAL(CP(HSUSB0_DATA6),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA6*/
	MUX_VAL(CP(HSUSB0_DATA7),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA7*/
#endif
	MUX_VAL(CP(I2C1_SCL),		(IEN  | PTU | EN  | M0)); /*I2C1_SCL*/
	MUX_VAL(CP(I2C1_SDA),		(IEN  | PTU | EN  | M0)); /*I2C1_SDA*/
#ifdef SAFE_MODE_PINS_5
	MUX_VAL(CP(I2C2_SCL),		(IEN  | PTD | EN  | M7)); /*I2C2_SCL*/
	MUX_VAL(CP(I2C2_SDA),		(IEN  | PTD | EN  | M7)); /*I2C2_SDA*/
	MUX_VAL(CP(I2C3_SCL),		(IEN  | PTD | EN  | M7)); /*I2C3_SCL*/
	MUX_VAL(CP(I2C3_SDA),		(IEN  | PTD | EN  | M7)); /*I2C3_SDA*/
	MUX_VAL(CP(I2C4_SCL),		(IEN  | PTD | EN  | M7)); /*I2C4_SCL*/
	MUX_VAL(CP(I2C4_SDA),		(IEN  | PTD | EN  | M7)); /*I2C4_SDA*/
#else
	MUX_VAL(CP(I2C2_SCL),		(IEN  | PTU | EN  | M0)); /*I2C2_SCL*/
	MUX_VAL(CP(I2C2_SDA),		(IEN  | PTU | EN  | M0)); /*I2C2_SDA*/
	MUX_VAL(CP(I2C3_SCL),		(IEN  | PTU | EN  | M0)); /*I2C3_SCL*/
	MUX_VAL(CP(I2C3_SDA),		(IEN  | PTU | EN  | M0)); /*I2C3_SDA*/
	MUX_VAL(CP(I2C4_SCL),		(IEN  | PTU | EN  | M0)); /*I2C4_SCL*/
	MUX_VAL(CP(I2C4_SDA),		(IEN  | PTU | EN  | M0)); /*I2C4_SDA*/
#endif
	MUX_VAL(CP(HDQ_SIO),		(IEN  | PTU | EN  | M0)); /*HDQ_SIO*/
#ifdef SAFE_MODE_PINS_5A
	MUX_VAL(CP(MCSPI1_CLK),		(IEN  | PTD | EN  | M7)); /*McSPI1_CLK*/
	MUX_VAL(CP(MCSPI1_SIMO),	(IEN  | PTD | EN  | M7)); /*McSPI1_SIMO  */
	MUX_VAL(CP(MCSPI1_SOMI),	(IEN  | PTD | EN  | M7)); /*McSPI1_SOMI  */
	MUX_VAL(CP(MCSPI1_CS0),		(IEN  | PTD | EN  | M7)); /*McSPI1_CS0*/
	MUX_VAL(CP(MCSPI1_CS1),		(IEN  | PTD | EN  | M7)); /*GPIO_175*/
								 /* TS_PEN_IRQ */
	MUX_VAL(CP(MCSPI1_CS2),		(IEN  | PTD | EN  | M7)); /*GPIO_176*/
								 /* - LAN_INTR*/
	MUX_VAL(CP(MCSPI1_CS3),		(IEN  | PTD | EN  | M7)); /*McSPI1_CS3*/
	MUX_VAL(CP(MCSPI2_CLK),		(IEN  | PTD | EN  | M7)); /*McSPI2_CLK*/
	MUX_VAL(CP(MCSPI2_SIMO),	(IEN  | PTD | EN  | M7)); /*McSPI2_SIMO*/
	MUX_VAL(CP(MCSPI2_SOMI),	(IEN  | PTD | EN  | M7)); /*McSPI2_SOMI*/
	MUX_VAL(CP(MCSPI2_CS0),		(IEN  | PTD | EN  | M7)); /*McSPI2_CS0*/
	MUX_VAL(CP(MCSPI2_CS1),		(IEN  | PTD | EN  | M7)); /*McSPI2_CS1*/
#else
	MUX_VAL(CP(MCSPI1_CLK),		(IEN  | PTD | DIS | M0)); /*McSPI1_CLK*/
	MUX_VAL(CP(MCSPI1_SIMO),	(IEN  | PTD | DIS | M0)); /*McSPI1_SIMO  */
	MUX_VAL(CP(MCSPI1_SOMI),	(IEN  | PTD | DIS | M0)); /*McSPI1_SOMI  */
	MUX_VAL(CP(MCSPI1_CS0),		(IEN  | PTD | EN  | M0)); /*McSPI1_CS0*/
	MUX_VAL(CP(MCSPI1_CS1),		(IEN  | PTD | EN  | M4)); /*GPIO_175*/
								 /* TS_PEN_IRQ */
	MUX_VAL(CP(MCSPI1_CS2),		(IEN  | PTU | DIS | M4)); /*GPIO_176*/
								 /* - LAN_INTR*/
	MUX_VAL(CP(MCSPI1_CS3),		(IEN  | PTD | EN  | M0)); /*McSPI1_CS3*/
	MUX_VAL(CP(MCSPI2_CLK),		(IEN  | PTD | DIS | M0)); /*McSPI2_CLK*/
	MUX_VAL(CP(MCSPI2_SIMO),	(IEN  | PTD | DIS | M0)); /*McSPI2_SIMO*/
	MUX_VAL(CP(MCSPI2_SOMI),	(IEN  | PTD | DIS | M0)); /*McSPI2_SOMI*/
	MUX_VAL(CP(MCSPI2_CS0),		(IEN  | PTD | EN  | M0)); /*McSPI2_CS0*/
	MUX_VAL(CP(MCSPI2_CS1),		(IEN  | PTD | EN  | M0)); /*McSPI2_CS1*/
#endif
 /*Control and debug */
	MUX_VAL(CP(SYS_32K),		(IEN  | PTD | DIS | M0)); /*SYS_32K*/
	MUX_VAL(CP(SYS_CLKREQ),		(IEN  | PTD | DIS | M0)); /*SYS_CLKREQ*/
	MUX_VAL(CP(SYS_NIRQ),		(IEN  | PTU | EN  | M0)); /*SYS_nIRQ*/
#ifdef SAFE_MODE_PINS_6
	MUX_VAL(CP(SYS_BOOT0),		(IEN  | PTD | EN  | M7)); /*GPIO_2*/
								 /* - PEN_IRQ */
	MUX_VAL(CP(SYS_BOOT1),		(IEN  | PTD | EN  | M7)); /*GPIO_3 */
	MUX_VAL(CP(SYS_BOOT2),		(IEN  | PTD | EN  | M7)); /*GPIO_4*/
	MUX_VAL(CP(SYS_BOOT3),		(IEN  | PTD | EN  | M7)); /*GPIO_5*/
	MUX_VAL(CP(SYS_BOOT4),		(IEN  | PTD | EN  | M7)); /*GPIO_6*/
	MUX_VAL(CP(SYS_BOOT5),		(IEN  | PTD | EN  | M7)); /*GPIO_7*/
#if 1
	MUX_VAL(CP(SYS_BOOT6),		(IDIS | PTU | DIS | M4)); /*SOM BACKLIGHT PWR*/
#else
	MUX_VAL(CP(SYS_BOOT6),		(IDIS | PTD | EN  | M7)); /*GPIO_8*/
#endif
#else
	MUX_VAL(CP(SYS_BOOT0),		(IEN  | PTD | DIS | M4)); /*GPIO_2*/
								 /* - PEN_IRQ */
	MUX_VAL(CP(SYS_BOOT1),		(IEN  | PTD | DIS | M4)); /*GPIO_3 */
	MUX_VAL(CP(SYS_BOOT2),		(IEN  | PTD | DIS | M4)); /*GPIO_4*/
	MUX_VAL(CP(SYS_BOOT3),		(IEN  | PTD | DIS | M4)); /*GPIO_5*/
	MUX_VAL(CP(SYS_BOOT4),		(IEN  | PTD | DIS | M4)); /*GPIO_6*/
	MUX_VAL(CP(SYS_BOOT5),		(IEN  | PTD | DIS | M4)); /*GPIO_7*/
	MUX_VAL(CP(SYS_BOOT6),		(IDIS | PTD | DIS | M4)); /*GPIO_8*/
#endif								 /* - VIO_1V8*/
	MUX_VAL(CP(SYS_OFF_MODE),	(IEN  | PTD | DIS | M0)); /*SYS_OFF_MODE*/
	MUX_VAL(CP(SYS_CLKOUT1),	(IEN  | PTD | DIS | M0)); /*SYS_CLKOUT1*/
	MUX_VAL(CP(SYS_CLKOUT2),	(IEN  | PTU | EN  | M0)); /*SYS_CLKOUT2*/
	MUX_VAL(CP(JTAG_nTRST),		(IEN  | PTD | DIS | M0)); /*JTAG_nTRST*/
	MUX_VAL(CP(JTAG_TCK),		(IEN  | PTD | DIS | M0)); /*JTAG_TCK*/
	MUX_VAL(CP(JTAG_TMS),		(IEN  | PTD | DIS | M0)); /*JTAG_TMS*/
	MUX_VAL(CP(JTAG_TDI),		(IEN  | PTD | DIS | M0)); /*JTAG_TDI*/
	MUX_VAL(CP(JTAG_EMU0),		(IEN  | PTD | DIS | M0)); /*JTAG_EMU0*/
	MUX_VAL(CP(JTAG_EMU1),		(IEN  | PTD | DIS | M0)); /*JTAG_EMU1*/
	MUX_VAL(CP(ETK_CLK_ES2),	(IDIS | PTU | EN  | M0)); /*ETK_CLK*/
	MUX_VAL(CP(ETK_CTL_ES2),	(IDIS | PTD | DIS | M0)); /*ETK_CTL*/
	MUX_VAL(CP(ETK_D0_ES2 ),	(IEN  | PTD | DIS | M0)); /*ETK_D0*/
	MUX_VAL(CP(ETK_D1_ES2 ),	(IEN  | PTD | DIS | M0)); /*ETK_D1*/
	MUX_VAL(CP(ETK_D2_ES2 ),	(IEN  | PTD | EN  | M0)); /*ETK_D2*/
	MUX_VAL(CP(ETK_D3_ES2 ),	(IEN  | PTD | DIS | M0)); /*ETK_D3*/
	MUX_VAL(CP(ETK_D4_ES2 ),	(IEN  | PTD | DIS | M0)); /*ETK_D4*/
	MUX_VAL(CP(ETK_D5_ES2 ),	(IEN  | PTD | DIS | M0)); /*ETK_D5*/
	MUX_VAL(CP(ETK_D6_ES2 ),	(IEN  | PTD | DIS | M0)); /*ETK_D6*/
	MUX_VAL(CP(ETK_D7_ES2 ),	(IEN  | PTD | DIS | M0)); /*ETK_D7*/
	MUX_VAL(CP(ETK_D8_ES2 ),	(IEN  | PTD | DIS | M0)); /*ETK_D8*/
	MUX_VAL(CP(ETK_D9_ES2 ),	(IEN  | PTD | DIS | M0)); /*ETK_D9*/
	MUX_VAL(CP(ETK_D10_ES2),	(IEN  | PTD | DIS | M0)); /*ETK_D10*/
	MUX_VAL(CP(ETK_D11_ES2),	(IEN  | PTD | DIS | M0)); /*ETK_D11*/
	MUX_VAL(CP(ETK_D12_ES2),	(IEN  | PTD | DIS | M0)); /*ETK_D12*/
	MUX_VAL(CP(ETK_D13_ES2),	(IEN  | PTD | DIS | M0)); /*ETK_D13*/
	MUX_VAL(CP(ETK_D14_ES2),	(IEN  | PTD | DIS | M0)); /*ETK_D14*/
	MUX_VAL(CP(ETK_D15_ES2),	(IEN  | PTD | DIS | M0)); /*ETK_D15*/
 /*Die to Die */
	MUX_VAL(CP(D2D_MCAD1),		(IEN  | PTD | EN  | M0)); /*d2d_mcad1*/
	MUX_VAL(CP(D2D_MCAD2),		(IEN  | PTD | EN  | M0)); /*d2d_mcad2*/
	MUX_VAL(CP(D2D_MCAD3),		(IEN  | PTD | EN  | M0)); /*d2d_mcad3*/
	MUX_VAL(CP(D2D_MCAD4),		(IEN  | PTD | EN  | M0)); /*d2d_mcad4*/
	MUX_VAL(CP(D2D_MCAD5),		(IEN  | PTD | EN  | M0)); /*d2d_mcad5*/
	MUX_VAL(CP(D2D_MCAD6),		(IEN  | PTD | EN  | M0)); /*d2d_mcad6*/
	MUX_VAL(CP(D2D_MCAD7),		(IEN  | PTD | EN  | M0)); /*d2d_mcad7*/
	MUX_VAL(CP(D2D_MCAD8),		(IEN  | PTD | EN  | M0)); /*d2d_mcad8*/
	MUX_VAL(CP(D2D_MCAD9),		(IEN  | PTD | EN  | M0)); /*d2d_mcad9*/
	MUX_VAL(CP(D2D_MCAD10),		(IEN  | PTD | EN  | M0)); /*d2d_mcad10*/
	MUX_VAL(CP(D2D_MCAD11),		(IEN  | PTD | EN  | M0)); /*d2d_mcad11*/
	MUX_VAL(CP(D2D_MCAD12),		(IEN  | PTD | EN  | M0)); /*d2d_mcad12*/
	MUX_VAL(CP(D2D_MCAD13),		(IEN  | PTD | EN  | M0)); /*d2d_mcad13*/
	MUX_VAL(CP(D2D_MCAD14),		(IEN  | PTD | EN  | M0)); /*d2d_mcad14*/
	MUX_VAL(CP(D2D_MCAD15),		(IEN  | PTD | EN  | M0)); /*d2d_mcad15*/
	MUX_VAL(CP(D2D_MCAD16),		(IEN  | PTD | EN  | M0)); /*d2d_mcad16*/
	MUX_VAL(CP(D2D_MCAD17),		(IEN  | PTD | EN  | M0)); /*d2d_mcad17*/
	MUX_VAL(CP(D2D_MCAD18),		(IEN  | PTD | EN  | M0)); /*d2d_mcad18*/
	MUX_VAL(CP(D2D_MCAD19),		(IEN  | PTD | EN  | M0)); /*d2d_mcad19*/
	MUX_VAL(CP(D2D_MCAD20),		(IEN  | PTD | EN  | M0)); /*d2d_mcad20*/
	MUX_VAL(CP(D2D_MCAD21),		(IEN  | PTD | EN  | M0)); /*d2d_mcad21*/
	MUX_VAL(CP(D2D_MCAD22),		(IEN  | PTD | EN  | M0)); /*d2d_mcad22*/
	MUX_VAL(CP(D2D_MCAD23),		(IEN  | PTD | EN  | M0)); /*d2d_mcad23*/
	MUX_VAL(CP(D2D_MCAD24),		(IEN  | PTD | EN  | M0)); /*d2d_mcad24*/
	MUX_VAL(CP(D2D_MCAD25),		(IEN  | PTD | EN  | M0)); /*d2d_mcad25*/
	MUX_VAL(CP(D2D_MCAD26),		(IEN  | PTD | EN  | M0)); /*d2d_mcad26*/
	MUX_VAL(CP(D2D_MCAD27),		(IEN  | PTD | EN  | M0)); /*d2d_mcad27*/
	MUX_VAL(CP(D2D_MCAD28),		(IEN  | PTD | EN  | M0)); /*d2d_mcad28*/
	MUX_VAL(CP(D2D_MCAD29),		(IEN  | PTD | EN  | M0)); /*d2d_mcad29*/
	MUX_VAL(CP(D2D_MCAD30),		(IEN  | PTD | EN  | M0)); /*d2d_mcad30*/
	MUX_VAL(CP(D2D_MCAD31),		(IEN  | PTD | EN  | M0)); /*d2d_mcad31*/
	MUX_VAL(CP(D2D_MCAD32),		(IEN  | PTD | EN  | M0)); /*d2d_mcad32*/
	MUX_VAL(CP(D2D_MCAD33),		(IEN  | PTD | EN  | M0)); /*d2d_mcad33*/
	MUX_VAL(CP(D2D_MCAD34),		(IEN  | PTD | EN  | M0)); /*d2d_mcad34*/
	MUX_VAL(CP(D2D_MCAD35),		(IEN  | PTD | EN  | M0)); /*d2d_mcad35*/
	MUX_VAL(CP(D2D_MCAD36),		(IEN  | PTD | EN  | M0)); /*d2d_mcad36*/
	MUX_VAL(CP(D2D_CLK26MI),	(IEN  | PTD | DIS | M0)); /*d2d_clk26mi*/
	MUX_VAL(CP(D2D_NRESPWRON),	(IEN  | PTD | EN  | M0)); /*d2d_nrespwron*/
	MUX_VAL(CP(D2D_NRESWARM),	(IEN  | PTU | EN  | M0)); /*d2d_nreswarm */
	MUX_VAL(CP(D2D_ARM9NIRQ),	(IEN  | PTD | DIS | M0)); /*d2d_arm9nirq */
	MUX_VAL(CP(D2D_UMA2P6FIQ),	(IEN  | PTD | DIS | M0)); /*d2d_uma2p6fiq*/
	MUX_VAL(CP(D2D_SPINT),		(IEN  | PTD | EN  | M0)); /*d2d_spint*/
	MUX_VAL(CP(D2D_FRINT),		(IEN  | PTD | EN  | M0)); /*d2d_frint*/
	MUX_VAL(CP(D2D_DMAREQ0),	(IEN  | PTD | DIS | M0)); /*d2d_dmareq0*/
	MUX_VAL(CP(D2D_DMAREQ1),	(IEN  | PTD | DIS | M0)); /*d2d_dmareq1*/
	MUX_VAL(CP(D2D_DMAREQ2),	(IEN  | PTD | DIS | M0)); /*d2d_dmareq2*/
	MUX_VAL(CP(D2D_DMAREQ3),	(IEN  | PTD | DIS | M0)); /*d2d_dmareq3*/
	MUX_VAL(CP(D2D_N3GTRST),	(IEN  | PTD | DIS | M0)); /*d2d_n3gtrst*/
	MUX_VAL(CP(D2D_N3GTDI),		(IEN  | PTD | DIS | M0)); /*d2d_n3gtdi*/
	MUX_VAL(CP(D2D_N3GTDO),		(IEN  | PTD | DIS | M0)); /*d2d_n3gtdo*/
	MUX_VAL(CP(D2D_N3GTMS),		(IEN  | PTD | DIS | M0)); /*d2d_n3gtms*/
	MUX_VAL(CP(D2D_N3GTCK),		(IEN  | PTD | DIS | M0)); /*d2d_n3gtck*/
	MUX_VAL(CP(D2D_N3GRTCK),	(IEN  | PTD | DIS | M0)); /*d2d_n3grtck*/
	MUX_VAL(CP(D2D_MSTDBY),		(IEN  | PTU | EN  | M0)); /*d2d_mstdby*/
	MUX_VAL(CP(D2D_SWAKEUP),	(IEN  | PTD | EN  | M0)); /*d2d_swakeup*/
	MUX_VAL(CP(D2D_IDLEREQ),	(IEN  | PTD | DIS | M0)); /*d2d_idlereq*/
	MUX_VAL(CP(D2D_IDLEACK),	(IEN  | PTU | EN  | M0)); /*d2d_idleack*/
	MUX_VAL(CP(D2D_MWRITE),		(IEN  | PTD | DIS | M0)); /*d2d_mwrite*/
	MUX_VAL(CP(D2D_SWRITE),		(IEN  | PTD | DIS | M0)); /*d2d_swrite*/
	MUX_VAL(CP(D2D_MREAD),		(IEN  | PTD | DIS | M0)); /*d2d_mread*/
	MUX_VAL(CP(D2D_SREAD),		(IEN  | PTD | DIS | M0)); /*d2d_sread*/
	MUX_VAL(CP(D2D_MBUSFLAG),	(IEN  | PTD | DIS | M0)); /*d2d_mbusflag*/
	MUX_VAL(CP(D2D_SBUSFLAG),	(IEN  | PTD | DIS | M0)); /*d2d_sbusflag*/
	MUX_VAL(CP(SDRC_CKE0),		(IDIS | PTU | EN  | M0)); /*sdrc_cke0*/
	MUX_VAL(CP(SDRC_CKE1),		(IDIS | PTD | DIS | M7)); /*sdrc_cke1*/
}
#endif // USE_UBOOT_MUX
