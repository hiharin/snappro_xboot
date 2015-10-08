#include "interface.h"
#include "internals.h"
#include "id-errno.h"

int id_whatis(struct id_cookie *cookie, idenum_t *type)
{
	unsigned char byte;
	int oor;
	if (!cookie)
		return -ID_EINVAL;

	byte = id_fetch_byte(cookie->offset, &oor);
	if (oor != ID_EOK)
		return -ID_ERANGE;

	byte >>= 5;
	*type = (idenum_t)byte;
			
	return ID_EOK;
}
