/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * Copyright (C) 2001  Erik Mouw (J.A.K.Mouw@its.tudelft.nl)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307	 USA
 *
 */

/* Refer 
	- http://www.simtec.co.uk/products/SWLINUX/files/booting_article.html 
	- http://www.arm.linux.org.uk/developer/booting.php
*/


#include <common.h>
#include <asm/setup.h>
#include <asm/string.h>
#include <asm/arch/cpu.h>
#include <asm/arch/cpu.h>
#include <asm/arch/bits.h>
#include <asm/arch/mux.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/sys_info.h>
#include <asm/arch/clocks.h>
#include <asm/arch/mem.h>
#include <linux.h>

static struct tag *params;


static char * strcpy(char * dest,const char *src)
{
        char *tmp = dest;

        while ((*dest++ = *src++) != '\0')
                /* nothing */;
        return tmp;
}

static size_t strlen(const char *s)
{
	size_t ret = 0;
	while (s[ret]) {
		ret++;
	}
	return ret;
}

/*
 * disable IRQ/FIQ interrupts
 * returns true if interrupts had been enabled before we disabled them
 */
#if 0
int disable_interrupts (void)
{
	return 0;
}
#else
int disable_interrupts (void)
{
	unsigned long old,temp;
	__asm__ __volatile__("mrs %0, cpsr\n"
			     "orr %1, %0, #0xc0\n"
			     "msr cpsr_c, %1"
			     : "=r" (old), "=r" (temp)
			     :
			     : "memory");
	return (old & 0x80) == 0;
}
#endif
static void cache_flush(void)
{
	unsigned long i = 0;

	/*clean entire data cache*/
	asm ("mcr p15, 0, %0, c7, c10, 0": :"r" (i));

	 /* invalidate both caches and flush btb */
	asm ("mcr p15, 0, %0, c7, c7, 0": :"r" (i)); 
	
	/* mem barrier to sync things */
	asm ("mcr p15, 0, %0, c7, c10, 4": :"r" (i));
}


static void announce_and_cleanup(void)
{
	printf("\nCleaning up...\n\n");

	/*
	 * this function is called just before we call linux
	 * it prepares the processor for linux
	 *
	 * we turn off caches etc ...
	 */
	
	//disable_interrupts ();
	// This should be default off, if it cause any problem
	// refer arch/arm/lib/interrupts.c
	disable_interrupts ();
	
	
	/* turn off I/D-cache */
	// By default cache is truned off
	//*** arch/arm/lib/cache-cp15.c:
	//icache_disable();
	//dcache_disable();

	/* flush I/D-cache */
	cache_flush();

}

static void setup_start_tag (void)
{
	params->hdr.tag = ATAG_CORE;
	params->hdr.size = tag_size (tag_core);

	params->u.core.flags = 0;
	params->u.core.pagesize = 0;
	params->u.core.rootdev = 0;

	params = tag_next (params);
}


static void setup_memory_tags (void)
{

	//bank: 0, start: 080000000, size 268435456
	//bank: 1, start: 0a0000000, size 0

	// We can also hardcode the size of ram in banks
	// For Bank 1
	params->hdr.tag = ATAG_MEM;
	params->hdr.size = tag_size (tag_mem32);

	params->u.mem.start = OMAP34XX_SDRC_CS0;
	params->u.mem.size = get_sdr_cs_size(SDRC_CS0_OSET);

	params = tag_next (params);

	// For Bank 2
	params->hdr.tag = ATAG_MEM;
	params->hdr.size = tag_size (tag_mem32);

	params->u.mem.start = OMAP34XX_SDRC_CS1;
	params->u.mem.size = get_sdr_cs_size(SDRC_CS1_OSET);

	params = tag_next (params);

}



static void setup_commandline_tag (char *commandline)
{
	char *p;

	if (!commandline)
		return;

	/* eat leading white space */
	for (p = commandline; *p == ' '; p++);

	/* skip non-existent command lines so the kernel will still
	 * use its default command line.
	 */
	if (*p == '\0')
		return;

	params->hdr.tag = ATAG_CMDLINE;
	params->hdr.size =
		(sizeof (struct tag_header) + strlen (p) + 1 + 4) >> 2;

	strcpy (params->u.cmdline.cmdline, p);

	params = tag_next (params);
}

/*************************************************************************
 * get_board_rev() - setup to pass kernel board revision information
 * returns:(bit[0-3] sub version, higher bit[7-4] is higher version)
 *************************************************************************/
static u32 get_board_rev(void)
{
	return 0x20;
}

static void setup_revision_tag(struct tag **in_params)
{
	u32 rev = 0;
	u32 get_board_rev(void);

	rev = get_board_rev();
	params->hdr.tag = ATAG_REVISION;
	params->hdr.size = tag_size (tag_revision);
	params->u.revision.rev = rev;
	params = tag_next (params);
}


static void setup_end_tag (void)
{
	params->hdr.tag = ATAG_NONE;
	params->hdr.size = 0;
}

static bootm_headers_t images;		/* pointers to os/initrd/fdt images */

// LMB related stuffs
static ulong get_sp(void)
{
	ulong ret;

	asm("mov %0, sp" : "=r"(ret) : );
	return ret;
}

void arch_lmb_reserve(struct lmb *lmb)
{
	ulong sp;

	/*
	 * Booting a (Linux) kernel image
	 *
	 * Allocate space for command line and board info - the
	 * address should be as high as possible within the reach of
	 * the kernel (see CONFIG_SYS_BOOTMAPSZ settings), but in unused
	 * memory, which means far enough below the current stack
	 * pointer.
	 */
	sp = get_sp();
	debug("## Current stack ends at 0x%08lx ", sp);

	/* adjust sp by 1K to be safe */
	sp -= 1024;
	lmb_reserve(lmb, sp,
		    gd->bd->bi_dram[0].start + gd->bd->bi_dram[0].size - sp);
}

static void bootm_start_lmb(void)
{

	ulong		mem_start;
	phys_size_t	mem_size;

	lmb_init(&images.lmb);

	mem_start = CFG_RAM_START;
	mem_size = CFG_RAM_SIZE;

	printf("lmb memstart: %x , size: %x\n", mem_start, mem_size);

	lmb_add(&images.lmb, (phys_addr_t)mem_start, mem_size);

	arch_lmb_reserve(&images.lmb);
	board_lmb_reserve(&images.lmb);

}


void boot_linux (void)
{
	image_header_t *image = (image_header_t *) CFG_LOADADDR;
	int machid = MACH_TYPE_DM3730_TORPEDO;
	void (*kernel_entry)(int zero, int arch, uint params);
	//char *s;

	// We should try to pass it to below some how
	char *commandline = " "; //COMMANDLINE;
	uint tag = (OMAP34XX_SDRC_CS0 + 0x100);
	
	params = (struct tag *) (OMAP34XX_SDRC_CS0 + 0x100);

	// Set the kernel function pointer here
	kernel_entry = (void (*)(int, int, uint))image->ih_ep;

	printf ("## Transferring control to Linux (at address %08lx) ...\n",
	       (ulong) kernel_entry);

	setup_start_tag ();
	setup_revision_tag (&params);
	setup_memory_tags ();
	setup_commandline_tag (commandline);
	setup_end_tag();

	announce_and_cleanup();

	kernel_entry(0, machid, (uint) tag);
	/* does not return */

}

void print_header(void)
{
	image_header_t *image = (image_header_t *) CFG_LOADADDR;
	printf("image address @ %p\n", image);
	printf("ih_magic: %x\n", image->ih_magic);
	printf("ih_hcrc: %x\n", image->ih_hcrc);
	printf("ih_time: %x\n", image->ih_time);
	printf("ih_szie: %x\n", image->ih_size);
	printf("ih_load: %x\n", image->ih_load);
	printf("ih_ep: %x\n", image->ih_ep);
	printf("ih_dcrc: %x\n", image->ih_dcrc);
	printf("ih_os: %x\n", image->ih_os);
	printf("ih_arch %x\n", image->ih_arch);
	printf("ih_type: %x\n", image->ih_type);
	printf("ih_comp: %x\n", image->ih_comp);
	printf("ih_name: %s\n", image->ih_name);

	/* uint32_t	ih_magic;	/\* Image Header Magic Number	*\/ */
	/* uint32_t	ih_hcrc;	/\* Image Header CRC Checksum	*\/ */
	/* uint32_t	ih_time;	/\* Image Creation Timestamp	*\/ */
	/* uint32_t	ih_size;	/\* Image Data Size		*\/ */
	/* uint32_t	ih_load;	/\* Data	 Load  Address		*\/ */
	/* uint32_t	ih_ep;		/\* Entry Point Address		*\/ */
	/* uint32_t	ih_dcrc;	/\* Image Data CRC Checksum	*\/ */
	/* uint8_t		ih_os;		/\* Operating System		*\/ */
	/* uint8_t		ih_arch;	/\* CPU architecture		*\/ */
	/* uint8_t		ih_type;	/\* Image Type			*\/ */
	/* uint8_t		ih_comp;	/\* Compression Type		*\/ */
	/* uint8_t		ih_name[IH_NMLEN];	/\* Image Name		*\/ */
}
