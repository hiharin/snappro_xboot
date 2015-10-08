#include "interface.h"
#include "internals.h"
#include "id-errno.h"
#include "debug.h"

unsigned int extract_unsigned_pnum(struct id_cookie *cookie, int start_bit, int *err)
{
	unsigned int value=0;
	unsigned int bit_offset=0;
	unsigned char bits;
	unsigned char ch;
	int oor;

	*err = ID_EOK;
	for (;;) {
		ch = id_fetch_byte(cookie->offset++, &oor);
		if (oor != ID_EOK) {
			*err = oor;
			id_error("extract runs oor");
			return 0;
		}
		if (ch & (1<<(start_bit-1))) {
			/* more to go, accumulate bits */
			bits = ch & ((1<<(start_bit - 1)) - 1);
			value |= (bits << bit_offset);
			bit_offset += start_bit-1;
			start_bit = 8;
		} else {
			/* last byte of number */
			bits = ch & ((1<<(start_bit - 1)) - 1);
			value |= (bits << bit_offset);
			break;
		}
	}
	return value;
}

int extract_signed_pnum(struct id_cookie *cookie, int start_bit, int *err)
{
	int value=0;
	unsigned int bit_offset=0;
	unsigned char bits;
	unsigned char ch;
	int oor;

	*err = ID_EOK;
	for (;;) {
		ch = id_fetch_byte(cookie->offset++, &oor);
		if (oor != ID_EOK) {
			*err = oor;
			id_error("extract runs oor");
			return 0;
		}
		if (ch & (1<<(start_bit-1))) {
			/* more to go, accumulate bits */
			bits = ch & ((1<<(start_bit - 1)) - 1);
			value |= (bits << bit_offset);
			bit_offset += start_bit-1;
			start_bit = 8;
		} else {
			/* last byte of number */
			bits = ch & ((1<<(start_bit - 2)) - 1);
			value |= (bits << bit_offset);
			if (ch & (1<<(start_bit - 2)))
				value = -value;
			break;
		}
	}
	return value;
}
