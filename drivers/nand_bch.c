/*
 * This file provides ECC correction for more than 1 bit per block of data,
 * using binary BCH codes. It relies on the generic BCH library lib/bch.c.
 *
 * Copyright © 2011 Ivan Djelic <ivan.djelic@parrot.com>
 *
 * This file is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 or (at your option) any
 * later version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this file; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#include <common.h>

#include <asm/arch/sys_proto.h>
#include <asm/arch/sys_info.h>

#if 1
#if 1
#include <config.h>
#include <common.h>
#include <linux/mtd/compat.h>
#include <malloc.h>
#endif
#include <linux/types.h>
// #include <linux/kernel.h>
// #include <linux/module.h>
// #include <linux/slab.h>
#include <linux/bitops.h>
// #include <linux/mtd/mtd.h>
// #include <linux/mtd/nand.h>
#include <linux/mtd/nand_bch.h>
#include <linux/bch.h>
#else
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_bch.h>
#include <linux/bch.h>
#endif

#if 1
int mtd_debug_verbose = 0x1f;
#define MTD_DEBUG_LEVEL0	(0)	/* Quiet   */
#define MTD_DEBUG_LEVEL1	(1)	/* Audible */
#define MTD_DEBUG_LEVEL2	(2)	/* Loud    */
#define MTD_DEBUG_LEVEL3	(3)	/* Noisy   */
#define MTD_DEBUG_LEVEL4	(4)	/* Picky   */
#define MTDDEBUG(n, args...)				\
	do {						\
		if (n <= mtd_debug_verbose)		\
			printk(KERN_INFO args);		\
	} while(0)
#else /* CONFIG_MTD_DEBUG */
#define MTDDEBUG(n, args...)				\
	do {						\
		if (0)					\
			printk(KERN_INFO args);		\
	} while(0)
#endif /* CONFIG_MTD_DEBUG */

/**
 * struct nand_bch_control - private NAND BCH control structure
 * @bch:       BCH control structure
 * @ecclayout: private ecc layout for this BCH configuration
 * @errloc:    error location array
 * @eccmask:   XOR ecc mask, allows erased pages to be decoded as valid
 */
struct nand_bch_control {
	struct bch_control   *bch;
//	struct nand_ecclayout ecclayout;
	unsigned int         *errloc;
	unsigned char        *eccmask;
};

/* For BCH4 */
static int bch4_ecc_size = 512;
static int bch4_ecc_bytes = 7;
static struct nand_bch_control *bch4_bch;

/**
 * nand_bch_calculate_ecc - [NAND Interface] Calculate ECC for data block
 * @mtd:	MTD block structure
 * @buf:	input buffer with raw data
 * @code:	output buffer with ECC
 */
int nand_bch_calculate_ecc(
// struct mtd_info *mtd,
	const unsigned char *buf, unsigned char *code)
{
//	const struct nand_chip *chip = mtd->priv;
	struct nand_bch_control *nbc = bch4_bch;
	unsigned int i;

	memset(code, 0, bch4_ecc_bytes);
	encode_soft_bch(nbc->bch, buf, bch4_ecc_size, code);

	/* apply mask so that an erased page is a valid codeword */
	for (i = 0; i < bch4_ecc_bytes; i++)
		code[i] ^= nbc->eccmask[i];

	return 0;
}
// EXPORT_SYMBOL(nand_bch_calculate_ecc);

/**
 * nand_bch_correct_data - [NAND Interface] Detect and correct bit error(s)
 * @mtd:	MTD block structure
 * @buf:	raw data read from the chip
 * @read_ecc:	ECC from the chip
 * @calc_ecc:	the ECC calculated from raw data
 *
 * Detect and correct bit errors for a data byte block
 */
int nand_bch_correct_data(
//	struct mtd_info *mtd,
	unsigned char *buf,
			  unsigned char *read_ecc, unsigned char *calc_ecc)
{
//	const struct nand_chip *chip = mtd->priv;
	struct nand_bch_control *nbc = bch4_bch;
	unsigned int *errloc = nbc->errloc;
	int i, count;

	count = decode_soft_bch(nbc->bch, NULL, bch4_ecc_size, read_ecc, calc_ecc,
			   NULL, errloc);
	if (count > 0) {
		for (i = 0; i < count; i++) {
			if (errloc[i] < (bch4_ecc_size*8))
				/* error is located in data, correct it */
				buf[errloc[i] >> 3] ^= (1 << (errloc[i] & 7));
			/* else error in ecc, no action needed */

			MTDDEBUG(MTD_DEBUG_LEVEL0, "%s: corrected bitflip %u\n",
			      __func__, errloc[i]);
		}
	} else if (count < 0) {
		printk(KERN_ERR "ecc unrecoverable error (count %d)\n", count);
		count = -1;
	}
	return count;
}
// EXPORT_SYMBOL(nand_bch_correct_data);

/**
 * nand_bch_init - [NAND Interface] Initialize NAND BCH error correction
 * @mtd:	MTD block structure
 * @eccsize:	ecc block size in bytes
 * @eccbytes:	ecc length in bytes
 * @ecclayout:	output default layout
 *
 * Returns:
 *  a pointer to a new NAND BCH control structure, or NULL upon failure
 *
 * Initialize NAND BCH error correction. Parameters @eccsize and @eccbytes
 * are used to compute BCH parameters m (Galois field order) and t (error
 * correction capability). @eccbytes should be equal to the number of bytes
 * required to store m*t bits, where m is such that 2^m-1 > @eccsize*8.
 *
 * Example: to configure 4 bit correction per 512 bytes, you should pass
 * @eccsize = 512  (thus, m=13 is the smallest integer such that 2^m-1 > 512*8)
 * @eccbytes = 7   (7 bytes are required to store m*t = 13*4 = 52 bits)
 */
struct nand_bch_control *
nand_bch_init(
//	struct mtd_info *mtd,
	unsigned int eccsize, unsigned int eccbytes
//	, struct nand_ecclayout **ecclayout
	, int mtdwritesize
	)
{
	unsigned int m, t, eccsteps, i;
	struct nand_ecclayout *layout;
	struct nand_bch_control *nbc = NULL;
	unsigned char *erased_page;

	if (!eccsize || !eccbytes) {
		printk(KERN_WARNING "ecc parameters not supplied\n");
		goto fail;
	}

	m = fls(1+8*eccsize);
	t = (eccbytes*8)/m;

	nbc = kzalloc(sizeof(*nbc), GFP_KERNEL);
	if (!nbc)
		goto fail;

	nbc->bch = init_soft_bch(m, t, 0);
	if (!nbc->bch)
		goto fail;

	/* verify that eccbytes has the expected value */
	if (nbc->bch->ecc_bytes != eccbytes) {
		printk(KERN_WARNING "invalid eccbytes %u, should be %u\n",
		       eccbytes, nbc->bch->ecc_bytes);
		goto fail;
	}

	eccsteps = mtdwritesize/eccsize;

#if 0
	/* if no ecc placement scheme was provided, build one */
	if (!*ecclayout) {

		/* handle large page devices only */
		if (mtd->oobsize < 64) {
			printk(KERN_WARNING "must provide an oob scheme for "
			       "oobsize %d\n", mtd->oobsize);
			goto fail;
		}

		layout = &nbc->ecclayout;
		layout->eccbytes = eccsteps*eccbytes;

		/* reserve 2 bytes for bad block marker */
		if (layout->eccbytes+2 > mtd->oobsize) {
			printk(KERN_WARNING "no suitable oob scheme available "
			       "for oobsize %d eccbytes %u\n", mtd->oobsize,
			       eccbytes);
			goto fail;
		}
		/* put ecc bytes at oob tail */
		for (i = 0; i < layout->eccbytes; i++)
			layout->eccpos[i] = mtd->oobsize-layout->eccbytes+i;

		layout->oobfree[0].offset = 2;
		layout->oobfree[0].length = mtd->oobsize-2-layout->eccbytes;

		*ecclayout = layout;
	}
#endif

	/* sanity checks */
	if (8*(eccsize+eccbytes) >= (1 << m)) {
		printk(KERN_WARNING "eccsize %u is too large\n", eccsize);
		goto fail;
	}
#if 0
	if ((*ecclayout)->eccbytes != (eccsteps*eccbytes)) {
		printk(KERN_WARNING "invalid ecc layout\n");
		goto fail;
	}
#endif

	nbc->eccmask = kmalloc(eccbytes, GFP_KERNEL);
	nbc->errloc = kmalloc(t*sizeof(*nbc->errloc), GFP_KERNEL);
	if (!nbc->eccmask || !nbc->errloc)
		goto fail;
	/*
	 * compute and store the inverted ecc of an erased ecc block
	 */
	erased_page = kmalloc(eccsize, GFP_KERNEL);
	if (!erased_page)
		goto fail;

	memset(erased_page, 0xff, eccsize);
	memset(nbc->eccmask, 0, eccbytes);
	encode_soft_bch(nbc->bch, erased_page, eccsize, nbc->eccmask);
	kfree(erased_page);

	for (i = 0; i < eccbytes; i++)
		nbc->eccmask[i] ^= 0xff;

	bch4_bch = nbc;
	return nbc;
fail:
	nand_bch_free(nbc);
	bch4_bch = NULL;
	return NULL;
}
// EXPORT_SYMBOL(nand_bch_init);

/**
 * nand_bch_free - [NAND Interface] Release NAND BCH ECC resources
 * @nbc:	NAND BCH control structure
 */
void nand_bch_free(struct nand_bch_control *nbc)
{
	if (nbc) {
		free_soft_bch(nbc->bch);
		kfree(nbc->errloc);
		kfree(nbc->eccmask);
		kfree(nbc);
	}
}
// EXPORT_SYMBOL(nand_bch_free);

// MODULE_LICENSE("GPL");
// MODULE_AUTHOR("Ivan Djelic <ivan.djelic@parrot.com>");
// MODULE_DESCRIPTION("NAND software BCH ECC support");


int omap_correct_data_soft_bch4(uint8_t *dat, uint8_t *read_ecc, uint8_t *calc_ecc)
{
	int ret;

#ifdef DEBUG_BCH
	{
		int i;
		printf("%s:%d dat %p read_ecc %p calc_ecc %p\n", __FUNCTION__, __LINE__, dat, read_ecc, calc_ecc);
		printf("%s:%d dat:", __FUNCTION__, __LINE__);
		for (i=0; i<16; ++i)
			printf(" %02x", dat[i]);
		printf("\n");
		printf("%s:%d read_ecc:", __FUNCTION__, __LINE__);
		for (i=0; i<7; ++i)
			printf(" %02x", read_ecc[i]);
		printf("\n");
		printf("%s:%d calc_ecc:", __FUNCTION__, __LINE__);
		for (i=0; i<7; ++i)
			printf(" %02x", calc_ecc[i]);
		printf("\n");
	}
#endif

	ret = nand_bch_correct_data(dat, read_ecc, calc_ecc);

#if 0
	printf("%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
#endif
	return ret;
}
int omap_calculate_ecc_soft_bch4(const uint8_t *dat, uint8_t *ecc_code)
{
	int ret;

#ifdef DEBUG_BCH
	{
		int i;
		printf("%s:%d dat %p ecc_code %p\n", __FUNCTION__, __LINE__, dat, ecc_code);

		printf("%s:%d dat:", __FUNCTION__, __LINE__);
		for (i=0; i<16; ++i)
			printf(" %02x", dat[i]);
		printf("\n");
	}
#endif

	ret = nand_bch_calculate_ecc(dat, ecc_code);

#ifdef DEBUG_BCH
	{
		int i;

		printf("%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		printf("%s:%d ecc_code:", __FUNCTION__, __LINE__);
		for (i=0; i<7; ++i)
			printf(" %02x", ecc_code[i]);
		printf("\n");
	}
#endif
	return ret;
}
