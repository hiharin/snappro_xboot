/*
 * Extract an unsigned packed number, first byte is in 'pack_bits'
 * of first byte, starting at offset 'offset' */
extern unsigned int extract_unsigned_pnum(struct id_cookie *cookie, int pack_bits, int *err);
extern int extract_signed_pnum(struct id_cookie *cookie, int pack_bits, int *err);



// extern struct id_data id_data;

extern int id_read_keys(struct id_data *data);
extern int id_verify_builtin_keys(struct id_data *data);

#define ID_MAX_KEY_SIZE 32

extern int id_get_key(struct id_data *data, int keyval, struct id_key *key);
extern int id_iterate_dict(struct id_data *data, struct id_cookie *cookie);

extern int id_extract_size(struct id_cookie *cookie, int *err);
