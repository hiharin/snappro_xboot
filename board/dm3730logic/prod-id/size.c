#include "interface.h"
#include "internals.h"
#include "id-errno.h"

int id_extract_size(struct id_cookie *cookie, int *err)
{
	idenum_t type;
	struct id_cookie s_cookie;
	int size;

	s_cookie = *cookie;

	*err = id_whatis(&s_cookie, &type);
	if (*err != ID_EOK)
		return *err;

	switch(type) {
	case IDENUM_DICT:
		size = extract_unsigned_pnum(&s_cookie, 5, err);
		size += (s_cookie.offset - cookie->offset);
		break;
	case IDENUM_ARRAY:
		size = extract_unsigned_pnum(&s_cookie, 5, err);
		size += (s_cookie.offset - cookie->offset);
		break;
	case IDENUM_STR:
	case IDENUM_HEXSTR:
		size = extract_unsigned_pnum(&s_cookie, 5, err);
		size += (s_cookie.offset - cookie->offset);
		break;
	case IDENUM_POS_NUM:
	case IDENUM_NEG_NUM:
		extract_signed_pnum(&s_cookie, 5, err);
		size = (s_cookie.offset - cookie->offset);
		break;
	case IDENUM_KEY:
		extract_unsigned_pnum(&s_cookie, 5, err);
		size = (s_cookie.offset - cookie->offset);
		break;
	default:
		*err = -ID_EDOM;
		break;
	}
	if (*err != ID_EOK)
		return *err;

	return size;
}
