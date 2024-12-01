#ifndef __SETTINGS_H__
#define __SETTINGS_H__

enum setting_id {
	e_setting_max,
};

void get_setting(enum setting_id, uint8_t *val, int len);
void set_setting(enum setting_id, uint8_t *val, int len);

void settings_init(void);

#endif // __SETTINGS_H__
