#include "crc-15.h"

/*
 * Calculate a CRC-15 of a data buffer passed in
 */

void crc_15_step(unsigned short *crc, unsigned char byte)
{
	int i;
	unsigned short crcnext;

	for (i=0; i<7; ++i) {
		crcnext = (byte & 1) ^ (*crc>>14);
		*crc = (*crc << 1) & 0x7fff;
		if (crcnext)
			*crc ^= 0x4599;
		byte >>= 1;
	}
}

unsigned short crc_15(void *buf, int len)
{
	unsigned char *p = buf;
	unsigned short xsum = 0;
	int i;

	for (i=0; i<len; ++i) {
		crc_15_step(&xsum, p[i]);
	}
	return xsum;
}
