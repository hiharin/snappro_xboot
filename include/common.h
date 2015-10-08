/*
 * (C) Copyright 2004
 * Texas Instruments
 *
 * (C) Copyright 2000-2004
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __COMMON_H_
#define __COMMON_H_	1

#undef	_LINUX_CONFIG_H
#define _LINUX_CONFIG_H 1	/* avoid reading Linux autoconf.h file	*/

#define DIV_ROUND_UP(n,d)	(((n) + (d) - 1) / (d))
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

typedef unsigned char		uchar;
typedef volatile unsigned long	vu_long;
typedef volatile unsigned short vu_short;
typedef volatile unsigned char	vu_char;

#include <config.h>
#include <linux/types.h>
#include <stdarg.h>

#ifdef CONFIG_ARM
#define asmlinkage	/* nothing */
#endif


#ifdef CONFIG_ARM
# include <asm/setup.h>
# include <asm/arch/mem.h>
# include <asm/x-load-arm.h>	/* ARM version to be fixed! */
#endif /* CONFIG_ARM */

#ifdef	CFG_PRINTF
extern int vsprintf(char *buf, const char *fmt, va_list args);
#define printf(fmt,args...)	serial_printf (fmt ,##args)
#define getc() serial_getc()
#else
#define printf(fmt,args...)
#define getc() ' '
#endif	/* CFG_PRINTF */

/* board/$(BOARD)/$(BOARD).c */
int 	board_init (void);
int 	nand_init (void);
int     mmc_boot (unsigned char *buf);
void	board_hang (void);

/* cpu/$(CPU)/cpu.c */
int 	cpu_init (void);

void 	udelay (unsigned long usec);

/* nand driver */
#define NAND_CMD_READ0		0
#define NAND_CMD_READ1		1
#define NAND_CMD_READOOB	0x50
#define NAND_CMD_STATUS		0x70
#define NAND_CMD_READID		0x90
#define NAND_CMD_SET_FEATURES	0xef
#define NAND_CMD_RESET		0xff

#define NAND_STS_RDY		0x40

/* Extended Commands for Large page devices */
#define NAND_CMD_READSTART	0x30

int 	nand_read_block(uchar *buf, ulong block_addr);

int	onenand_read_block(unsigned char *buf, ulong block);

#ifdef CFG_PRINTF

/* serial driver */
int	serial_init   (void);
void	serial_setbrg (void);
void	serial_putc   (const char);
void	serial_puts   (const char *);
int	serial_getc   (void);
int	serial_tstc   (void);

/* lib/printf.c */
void	serial_printf (const char *fmt, ...);
#endif

/* lib/crc.c */
void 	nand_calculate_ecc (const u_char *dat, u_char *ecc_code);
int 	nand_correct_data (u_char *dat, u_char *read_ecc, u_char *calc_ecc);

/* lib/malloc.c */
extern void *malloc(size_t size);
extern void *calloc(size_t nemb, size_t size);
extern void free(void *ptr);
extern void malloc_init(void *membase, size_t size);

/* lib/board.c */
void	hang		(void) __attribute__ ((noreturn));

#define	NAND_ECC_SW_1BIT	0x01	/* SW 1bit ECC */
#define	NAND_ECC_HW_1BIT	0x02	/* HW 1bit ECC */
#define NAND_ECC_4BIT		0x04	/* HW 4bit bch */
#define	NAND_ECC_8BIT		0x08	/* HW 8bit bch */
#define	NAND_ECC_SW_4BIT	0x10	/* SW 4bit bch */
#define NAND_ECC_CHIP		0x20	/* In-chip ECC */

extern int 	nand_chip(int nand_ecc);
extern int 	onenand_chip(int nand_ecc);

/* lcd */
void lcd_init(omap_boot_device_t boot_device);

/* timer */
int timer_init(void);
int init_gpt_timer(u32 timer, u32 value, u32 range);

struct gptimer {
	u32 tidr;	/* 0x00 r */
	u8 res[0xc];
	u32 tiocp_cfg;	/* 0x10 rw */
	u32 tistat;	/* 0x14 r */
	u32 tisr;	/* 0x18 rw */
	u32 tier;	/* 0x1c rw */
	u32 twer;	/* 0x20 rw */
	u32 tclr;	/* 0x24 rw */
	u32 tcrr;	/* 0x28 rw */
	u32 tldr;	/* 0x2c rw */
	u32 ttgr;	/* 0x30 rw */
	u32 twpc;	/* 0x34 r*/
	u32 tmar;	/* 0x38 rw*/
	u32 tcar1;	/* 0x3c r */
	u32 tcicr;	/* 0x40 rw */
	u32 tcar2;	/* 0x44 r */
};

struct prcm {
	u32 fclken_iva2;	/* 0x00 */
	u32 clken_pll_iva2;	/* 0x04 */
	u8 res1[0x1c];
	u32 idlest_pll_iva2;	/* 0x24 */
	u8 res2[0x18];
	u32 clksel1_pll_iva2 ;	/* 0x40 */
	u32 clksel2_pll_iva2;	/* 0x44 */
	u8 res3[0x8bc];
	u32 clken_pll_mpu;	/* 0x904 */
	u8 res4[0x1c];
	u32 idlest_pll_mpu;	/* 0x924 */
	u8 res5[0x18];
	u32 clksel1_pll_mpu;	/* 0x940 */
	u32 clksel2_pll_mpu;	/* 0x944 */
	u8 res6[0xb8];
	u32 fclken1_core;	/* 0xa00 */
	u32 res_fclken2_core;
	u32 fclken3_core;	/* 0xa08 */
	u8 res7[0x4];
	u32 iclken1_core;	/* 0xa10 */
	u32 iclken2_core;	/* 0xa14 */
	u32 iclken3_core;	/* 0xa18 */
	u8 res8[0x24];
	u32 clksel_core;	/* 0xa40 */
	u8 res9[0xbc];
	u32 fclken_gfx;		/* 0xb00 */
	u8 res10[0xc];
	u32 iclken_gfx;		/* 0xb10 */
	u8 res11[0x2c];
	u32 clksel_gfx;		/* 0xb40 */
	u8 res12[0xbc];
	u32 fclken_wkup;	/* 0xc00 */
	u8 res13[0xc];
	u32 iclken_wkup;	/* 0xc10 */
	u8 res14[0xc];
	u32 idlest_wkup;	/* 0xc20 */
	u8 res15[0x1c];
	u32 clksel_wkup;	/* 0xc40 */
	u8 res16[0xbc];
	u32 clken_pll;		/* 0xd00 */
	u32 clken2_pll;	        /* 0xd04 */
	u8 res17[0x18];
	u32 idlest_ckgen;	/* 0xd20 */
	u32 idlest2_ckgen;	/* 0xd24 */
	u8 res18[0x18];
	u32 clksel1_pll;	/* 0xd40 */
	u32 clksel2_pll;	/* 0xd44 */
	u32 clksel3_pll;	/* 0xd48 */
	u32 clksel4_pll;	/* 0xd4c */
	u32 clksel5_pll;	/* 0xd50 */
	u8 res19[0xac];
	u32 fclken_dss;		/* 0xe00 */
	u8 res20[0xc];
	u32 iclken_dss;		/* 0xe10 */
	u8 res21[0x2c];
	u32 clksel_dss;		/* 0xe40 */
	u8 res22[0xbc];
	u32 fclken_cam;		/* 0xf00 */
	u8 res23[0xc];
	u32 iclken_cam;		/* 0xf10 */
	u8 res24[0x2c];
	u32 clksel_cam;		/* 0xf40 */
	u8 res25[0xbc];
	u32 fclken_per;		/* 0x1000 */
	u8 res26[0xc];
	u32 iclken_per;		/* 0x1010 */
	u8 res27[0x2c];
	u32 clksel_per;		/* 0x1040 */
	u8 res28[0xfc];
	u32 clksel1_emu;	/* 0x1140 */
	u8 res29[0x2bc];
	u32 fclken_usbhost;	/* 0x1400 */
	u8 res30[0xc];
	u32 iclken_usbhost;	/* 0x1410 */
};

#endif	/* __COMMON_H_ */
