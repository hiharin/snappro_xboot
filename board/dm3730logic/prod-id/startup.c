#include "interface.h"
#include "internals.h"
#include "id-errno.h"
#include "crc-15.h"

/* struct id_data id_data; */

struct __attribute__ ((packed)) id_header { 
	unsigned char signature[4];
	unsigned char id_fmt_ver;
	unsigned char unused0;
	unsigned short data_length;
} ;

struct __attribute__ ((packed)) id_checksums { 
	unsigned short header;
	unsigned short data;
} ;

int id_startup(struct id_data *data)
{
	int i, err;
	struct id_cookie cookie;
	unsigned char byte, *p;
	char *header_tag= "LpId";
	unsigned short xsum;
	struct id_header hdr;
	struct id_checksums xsums;

	cookie.offset = 0;

	/* Data starts with the header, should be 'LpId' */
	for (i=0; i<4; ++i) {
		hdr.signature[i] = id_fetch_byte(cookie.offset++, &err);
		if (err != ID_EOK) {
			id_printf("%s[%u]\n", __FILE__, __LINE__);
			return err;
		}
		if (hdr.signature[i] != header_tag[i]) {
			id_printf("%s[%u]\n", __FILE__, __LINE__);
			return ID_ENODEV;
		}
	}

	/* First LE 8-bit value is ID format version */
	hdr.id_fmt_ver = id_fetch_byte(cookie.offset++, &err);
	
	/* Second LE 8-bit value is currently not used */
	hdr.unused0 = id_fetch_byte(cookie.offset++, &err);
	
	/* Next LE 16-bit value is length of data */
	hdr.data_length = id_fetch_byte(cookie.offset++, &err);
	hdr.data_length |= (id_fetch_byte(cookie.offset++, &err) << 8);
	
	/* Next LE 16-bit value is xsum of header */
	xsums.header = id_fetch_byte(cookie.offset++, &err);
	xsums.header |= (id_fetch_byte(cookie.offset++, &err) << 8);

	/* Checksum the header */
	xsum = 0;
	p = (unsigned char *)&hdr;
	for (i = 0; i < sizeof(hdr); ++i)
		crc_15_step(&xsum, p[i]);

	if (xsum != xsums.header) {
		id_printf("%s[%u] xsum: 0x%04x, xsums.header: 0x%04x\n", 
		        __FILE__, __LINE__, xsum, xsums.header);
		return -ID_EL2NSYNC;
	}

	/* Next LE 16-bit value is xsum of data */
	xsums.data = id_fetch_byte(cookie.offset++, &err);
	xsums.data |= (id_fetch_byte(cookie.offset++, &err) << 8);

	/* Checksum the data (next id_len bytes), must match xsums.data */
	xsum = 0;
	for (i = 0; i < hdr.data_length; ++i) {
		byte = id_fetch_byte(cookie.offset + i, &err);
		if (err != ID_EOK) {
			id_printf("%s[%u]\n", __FILE__, __LINE__);
			return err;
		}
		crc_15_step(&xsum, byte);
	}
	if (xsum != xsums.data) {
		id_printf("%s[%u] xsum: 0x%04x, xsums.data: 0x%04x\n", 
		        __FILE__, __LINE__, xsum, xsums.data);
		return -ID_EL2NSYNC;
	}

	/* offset is now at the first byte of the root dictionary which
	   contains its span */
	data->root_offset = cookie.offset;
	data->root_size = extract_unsigned_pnum(&cookie, 5, &err);
	if (err != ID_EOK) {
		id_printf("%s[%u]\n", __FILE__, __LINE__);
		return err;
	}

	data->root_size += cookie.offset - data->root_offset;

#if 0
	id_printf("Data format version: %u\n", hdr.id_fmt_ver);	
#endif	
	return ID_EOK;
}

/*
 * Reset the cookie to cover the whole root dictionary
 */
int id_init_cookie(struct id_data *data, struct id_cookie *cookie)
{
	if (!cookie)
		return -ID_EINVAL;
	cookie->start_offset = data->root_offset;
	cookie->size = data->root_size;
	cookie->offset = cookie->start_offset;
	return ID_EOK;
}
