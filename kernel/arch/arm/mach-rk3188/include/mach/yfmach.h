
#ifndef __YF_MACH_H_
#define __YF_MACH_H_
#include <linux/input.h>

int acc_supported(char * name);
void acc_register(char * name, int units);
void acc_report(struct input_dev *input, int x, int y, int z);

int ctp_supported(char * name);
void ctp_register(char * name);

void yf_mux_api_set(unsigned pin, unsigned mode);
int yf_mux_api_get(int pin);

void env_fixup(void);
const char * env_get_str(char * name, const char * def);
unsigned env_get_len(void * buffer);
unsigned env_get_u32(char *name, unsigned value);
unsigned * env_get_u32s(char *name);
unsigned env_cpy_u8s(char *name, void * buffer, unsigned len);
unsigned env_cpy_u32s(char *name, unsigned * buffer, unsigned len);
int env_enum_u32(char * name,void * param,int (*on_enum)(char *,unsigned *,void *));
int env_enum_str(char * name,void * param,int (*on_enum)(char *,char *,void *));
void env_set(void * src, unsigned size);

#define SYS_DATA_MARK 0x10000
#define SYS_DATA_MASK 0xFFFF
int sys_data_read(int index, int size, void * buf);
int sys_data_write(int index, int size, void * buf);

#define PMU_BAT_CAP   0
int pmu_data_read(int index);
int pmu_data_write(int index, u8 value);

#define STATUS_AC_IN             (1 << 0)
#define STATUS_USB_IN            (1 << 1)
#define STATUS_CHARGE_DONE       (1 << 2)
#define STATUS_CHARGING          (1 << 3)
#define STATUS_CHARGE_MASK       (3 << 2)
#define STATUS_CAPACITY          (1 << 4)
#define STATUS_MASK              (0xF)

struct power_callback {
	int mask;
	void * param;
	void ( * callback)(void * param, int status, int capacity);
	struct list_head list;
};
void power_register_callback(struct power_callback * callback);
void power_unregister_callback(struct power_callback * callback);
void power_changed(int mask, int value);
#endif
