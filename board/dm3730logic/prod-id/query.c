#include "interface.h"
#include "internals.h"
#include "id-errno.h"

static int id_extract_key(struct id_cookie *cookie, id_keys_t *key)
{
	int err;
	id_keys_t keyval;

	keyval = (id_keys_t)extract_unsigned_pnum(cookie, 5, &err);
	if (err != ID_EOK)
		return err;
	*key = keyval;
	return ID_EOK;
}

/* in dictionary that cookie points to find key "key"; if found
 * update cookie to associated "key" entry and return ID_EOK;
 * else return -ID_ENOENT */
int id_dict_find_key(struct id_cookie *cookie, id_keys_t key)
{
	int err;
	unsigned int size;
	id_keys_t d_key;
	idenum_t type;
	struct id_cookie d_cookie = *cookie;
	struct id_cookie t_cookie;

	err = id_whatis(cookie, &type);
	if (err != ID_EOK)
		return err;

	/* Header has to be a dictionary */
	if (type != IDENUM_DICT)
		return -ID_EINVAL;

	/* Extract size of dictionary */
	size = extract_unsigned_pnum(&d_cookie, 5, &err);
	if (err != ID_EOK)
		return err;

	d_cookie.size = size;
	d_cookie.start_offset = d_cookie.offset;

	/* cookie->offset is now at first key */
	while (d_cookie.offset < d_cookie.start_offset + d_cookie.size) {
		/* Extract the key and move the cookie over key */
		err = id_extract_key(&d_cookie, &d_key);
		if (err != ID_EOK)
			return err;
		t_cookie = d_cookie;
		/* move forward over the value */
		size = id_extract_size(&d_cookie, &err);
		if (err != ID_EOK)
			return err;
		if (key == d_key) {
			d_cookie.size = size;
			d_cookie.start_offset = t_cookie.offset;
			d_cookie.offset = t_cookie.offset;
			*cookie = d_cookie;
			return ID_EOK;
		}
		d_cookie.offset += size;
	}
	return -ID_ENOENT;
}

/* Are these two types a match? */
static int id_match_type(idenum_t type_a, idenum_t type_b)
{
	idenum_t tmp;

	if (type_a == type_b)
		return 1;

	/* Oder the types (so the "*ANY*" types are in type_b) */
	if ((int)type_a > (int)type_b) {
		tmp = type_a;
		type_a = type_b;
		type_b = tmp;
	}
	if (type_b == IDENUM_ANY_STRING && (type_a == IDENUM_STR || type_a == IDENUM_HEXSTR))
		return 1;

	if (type_b == IDENUM_ANY_NUMBER && (type_a == IDENUM_NEG_NUM || type_a == IDENUM_POS_NUM))
		return 1;

	return 0;
}

/* Find in dictionary (that cookie points to) key "key" that is type "type" */
int id_find_dict(struct id_cookie *cookie, id_keys_t key, idenum_t type)
{
	int err;
	struct id_cookie d_cookie = *cookie;
	idenum_t l_type;

	err = id_dict_find_key(&d_cookie, key);
	if (err != ID_EOK)
		return err;
	err = id_whatis(&d_cookie, &l_type);
	if (err != ID_EOK)
		return err;
	if (!id_match_type(l_type, type))
		return -ID_EINVAL;
	*cookie = d_cookie;
	return ID_EOK;
}

/* in dictionary pointed at by cookie, find the key "key"; verify its a
 * string and copy its value */
int id_find_string(struct id_cookie *cookie, id_keys_t key, unsigned char *str_ptr, unsigned int *str_size)
{
	int err, i;
	unsigned char byte;
	unsigned int size;
	struct id_cookie d_cookie = *cookie;

	err = id_find_dict(&d_cookie, key, IDENUM_ANY_STRING);

	if (err != ID_EOK)
		return err;
	/* Extract the string size */
	size = extract_unsigned_pnum(&d_cookie, 5, &err);
	if (err != ID_EOK)
		return err;

	if (size > *str_size)
		return -ID_ERANGE;

	for(i=0; i<size; ++i) {
		byte = id_fetch_byte(d_cookie.offset++, &err);
		if (err)
			return err;
		str_ptr[i] = byte;
	}
	*str_size = size;

	return ID_EOK;
}

/* in dictionary pointed at by cookie, find the key "key"; verify its a
 * number (either pos/neg) and return its value through *num */
int id_find_number(struct id_cookie *cookie, id_keys_t key, int *num)
{
	int err;
	int l_num;
	idenum_t l_type;
	struct id_cookie d_cookie = *cookie;

	err = id_find_dict(&d_cookie, key, IDENUM_ANY_NUMBER);

	if (err != ID_EOK)
		return err;
	err = id_whatis(&d_cookie, &l_type);
	if (err != ID_EOK)
		return err;
	/* Extract the number size */
	l_num = extract_unsigned_pnum(&d_cookie, 5, &err);
	if (err != ID_EOK)
		return err;

	if (l_type == IDENUM_NEG_NUM)
		l_num = -l_num;

	*num = l_num;
	return ID_EOK;
}

/* in dictionary pointed at by cookie, find the list of keys; verify they are
 * numbers (either pos/neg) and return their value through *nums */
int id_find_numbers(struct id_cookie *cookie, id_keys_t *keys, int key_size, int *nums)
{
	int i, err;
	struct id_cookie d_cookie;

	for (i=0;i<key_size; ++i) {
		d_cookie = *cookie;
		err = id_find_number(&d_cookie, keys[i], &nums[i]);
		if (err != ID_EOK)
			return err;
	}
	return ID_EOK;
}
