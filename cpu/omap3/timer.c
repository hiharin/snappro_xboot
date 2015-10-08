/*
 * (C) Copyright Idexx 2012
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
#include <asm/arch/omap3430.h>
#include <asm/arch/io.h>
#include <asm/arch/cpu.h>
#include <asm/types.h>
#include <linux/types.h>

#define CONFIG_SYS_TIMERBASE 	OMAP34XX_GPT2
#define CONFIG_SYS_PTV		2	/* Divisor: 2^(PTV+1) => 8 */
#define CONFIG_SYS_HZ		1000

/* Clock Defines */
#define V_OSCK			26000000	/* Clock output from T2 */
#define V_SCLK			(V_OSCK >> 1)

#define TIMER_CLOCK		(V_SCLK / (2 << CONFIG_SYS_PTV))
#define TIMER_OVERFLOW_VAL	0xffffffff
#define TIMER_LOAD_VAL		0

static struct gptimer *timer_base = (struct gptimer *) CONFIG_SYS_TIMERBASE;

int timer_init(void)
{
	/* start the counter ticking up, reload value on overflow */
	writel(TIMER_LOAD_VAL, &timer_base->tldr);
	/* enable timer */
	writel((CONFIG_SYS_PTV << 2) | TCLR_PRE | TCLR_AR | TCLR_ST,
		&timer_base->tclr);

	return 0;
}

/* delay x useconds */
void __udelay(unsigned long usec)
{
	long tmo = usec * (TIMER_CLOCK / 1000) / 1000;
	unsigned long now, last = readl(&timer_base->tcrr);

	while (tmo > 0) {
		now = readl(&timer_base->tcrr);
		if (last > now) /* count up timer overflow */
			tmo -= TIMER_OVERFLOW_VAL - last + now;
		else
			tmo -= now - last;
		last = now;
	}
}

int init_gpt_timer(u32 timer, u32 value, u32 range)
{
	struct prcm *prcm_base = (struct prcm *)PRCM_BASE;
	struct gptimer *timer_base;
	u32 reg;

	switch(timer) {
	case 10:
		writel(readl(&prcm_base->clksel_core) | (1 << 6), &prcm_base->clksel_core);
		writel(readl(&prcm_base->fclken1_core) | (1 << 11), &prcm_base->fclken1_core);
		writel(readl(&prcm_base->iclken1_core) | (1 << 11), &prcm_base->iclken1_core);
		timer_base = (struct gptimer *) OMAP34XX_GPT10;
		break;
	default:
		return -1;
	}

	/* dm_timer_set_load */
	reg = readl(&timer_base->tclr);
	reg |= TCLR_AR;
	writel(reg, &timer_base->tclr);
	writel(0xffff0000, &timer_base->tldr);
	writel(0, &timer_base->ttgr);

	/* dm_set_pwm */
	reg = readl(&timer_base->tclr);
	reg &= ~(TCLR_GPO_CFG | TCLR_SCPWM | TCLR_PT | (0x03 << 10));
	reg |= TCLR_PT;
	reg |= (0x2 << 10); /* OVERFLOW_AND_COMPARE */
	writel(reg, &timer_base->tclr);

	/* dm_set_match */
	reg = readl(&timer_base->tclr);
	reg |= TCLR_CE;
	writel(reg, &timer_base->tclr);
	writel(0xffff0000 | (((value * 0xff) / range) << 8), &timer_base->tmar);

	/* dm_timer_start */
	reg = readl(&timer_base->tclr);
	if (!(reg & TCLR_ST)) {
		reg |= TCLR_ST;
		writel(reg | TCLR_ST, &timer_base->tclr);
	}

	return 0;
}
