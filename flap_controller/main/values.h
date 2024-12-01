#ifndef __VALUES_H__
#define __VALUES_H__

typedef enum {
	e_nvs_ch1_src,
	e_nvs_ch2_src,
	e_nvs_ch1_dty,
	e_nvs_ch2_dty,
	e_nvs_ch1_in,
	e_nvs_ch2_in,

	e_nvs_tst,
	e_nvs_max,
} value_id_e;

void get_value(value_id_e id, void *val, int len);
void set_value(value_id_e id, void *val, int len);

void values_init(void);

#endif // __VALUES_H__
