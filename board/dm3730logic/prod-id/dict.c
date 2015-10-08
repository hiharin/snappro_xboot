#include "interface.h"
#include "internals.h"
#include "id-errno.h"

int id_dict_size(struct id_data *data, struct id_cookie *cookie)
{
	idenum_t type;
	int err;
	int count = 0;
	unsigned int size, keyval;
	struct id_cookie d_cookie;

	d_cookie = *cookie;

	/* It has to be a dictionary */
	err = id_whatis(&d_cookie, &type);
	if (type != IDENUM_DICT)
		return -ID_EINVAL;

	size = extract_unsigned_pnum(&d_cookie, 5, &err);
	if (err != ID_EOK)
		return err;

	d_cookie.size = size;
	d_cookie.start_offset = d_cookie.offset;
	while (d_cookie.offset < d_cookie.start_offset+d_cookie.size) {
		/* It has to be a key */
		err = id_whatis(&d_cookie, &type);
		if (type != IDENUM_KEY)
			return -ID_EINVAL;
		keyval = extract_unsigned_pnum(&d_cookie, 5, &err);
		if (err != ID_EOK)
			return err;
		
		/* Get the size of the object */
		size = id_extract_size(&d_cookie, &err);
		if (err != ID_EOK)
			return err;

		/* Move the offset forward by the object size */
		d_cookie.offset += size;

		/* Increment the count */
		count++;
	}
	return count;
}
