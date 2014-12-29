/*
 *  ADC based battery driver
 *
 * Battery driver for YF GPS
 * Copyright (C) 2009 YuanFeng <liqm@yfgps.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>

#include <linux/adc.h>
#include <mach/yfmach.h>
static struct adc_client * adc_client;
static struct adc_client * cur_client;

#ifndef GPIO_VALID
#define GPIO_VALID(x)            ((x) != 0xFFFFFFFF)
#endif
#define PMU_BAT_RDC              1

struct adc_item {
	int capacity;
	int voltage;
	int cur;
	struct adc_item * pre;
	struct adc_item * next;
};

static int adc_poll_time;
static int adc_poll_count;
static int adc_poll_index;
static int adc_poll_init;
static struct adc_item * adc_poll_head;
static struct adc_item * adc_poll_items;

static int adc_bat_status;
static int adc_capacity;
static int adc_voltage;
static int adc_rdc;
static int adc_max_vol;
static int adc_min_cur;
static struct mutex adc_work_lock; /* protects data */
static struct delayed_work adc_bat_work;
static int * adc_v2c_table; //voltage to capaicty table

static int adc_poll_supply;
static int adc_supply_status;
static int adc_running;
static struct delayed_work adc_supply_work;

static int adc_irq1; //ac or status irq
static int adc_irq2; //charge irq
static int adc_irq_type;
static int adc_ac_gpio;
static int adc_ac_level;
static int adc_usb_gpio;
static int adc_usb_level;
static int adc_charge_gpio;
static int adc_charge_level;
static int adc_switch_gpio;

static void  (*adc_update_capacity)(int capacity);

static int adc_debug;
static int adc_offset;

void power_changed(int mask, int value)
{
}
//============ platform specific ==============//
#include <linux/slab.h>
#include <mach/yfmach.h>
#include <mach/board.h>
//static struct platform_device adc_battery_device;

static int voltage_cof1;
static int voltage_cof2;
static int pmu_capacity = -1;

static u8 irq_hang;
extern struct work_struct dac_mute_work;

void update_pmu_capacity(int capacity)
{
	if(pmu_capacity >= 0) {
		if(capacity <= 0) {
			capacity = 0x80;
		}
		else if(capacity <= 5) {
			capacity |= (pmu_capacity & 0x80);
		}
		if(capacity != pmu_capacity) {
			pmu_data_write(PMU_BAT_CAP, capacity);
			pmu_capacity = capacity;
		}
	}
	power_changed(STATUS_CAPACITY, capacity);
}
static int charge_max1;
static int charge_max2;
static int current_cof1;
static int current_cof2;

static int last_voltage;
static int last_status;
static int status_cof1;
static int status_cof2;
static int status_offset;

static int get_adc(struct adc_client * client)
{
	int i;
	int t;
	int sum = 0, min = 0xffff, max = 0;
	for(i = 0; i < 6; i++) {
		t = adc_sync_read(client);
		if(t > max) max = t;
		if(t < min) min = t;
		sum += t;
	}
	return sum-min-max;
}

static void get_rdc(void)
{
	int rdc, count = 4, sum = 0, min = 10000, max = - 10000;
	int index1 = adc_poll_index;
	int index2 = adc_poll_index - 4;
	while(count--) {
		if(index1 < 0) index1 += adc_poll_count;
		if(index2 < 0) index2 += adc_poll_count;
		rdc = (adc_poll_items[index1].voltage - adc_poll_items[index2].voltage) * 1000 / 
		      (adc_poll_items[index1].cur - adc_poll_items[index2].cur);
		if(rdc < min) min = rdc;
		if(rdc > max) max = rdc;
		sum += rdc;
		index1--;
		index2--;
	}
	rdc = (sum - min - max) >> 1;
	if(adc_rdc != rdc) {
		adc_rdc = rdc;
		pmu_data_write(PMU_BAT_RDC, adc_rdc);
		printk("battery rdc updated to %d\n", rdc);
	}
}

static void adc_get_voltage(struct adc_item * item)
{
	int voltage, cur = 0, status = 0;
	gpio_direction_output(adc_switch_gpio, 0);
	msleep(1);
	voltage = get_adc(adc_client);
	if(cur_client) {
		cur = get_adc(cur_client);
		gpio_direction_output(adc_switch_gpio, 1);
	}
	else {
		gpio_direction_output(adc_switch_gpio, 1);
		msleep(1);
		cur = get_adc(adc_client);
	}

	voltage = ((voltage * voltage_cof1) >> 18) + voltage_cof2;
	cur = ((cur * voltage_cof1) >> 18) + voltage_cof2;

	if(cur >= adc_max_vol) {
		status = 1 << 2;
	}

	cur = (((cur - voltage) * current_cof1) >> 16) + current_cof2;
	status |= 1 << (cur < 0); //dischage 2, else 1
	if(cur <= adc_min_cur && cur >= -adc_min_cur) {
		status |= 1 << 2;
	}
	if(adc_poll_init == 1) {
		last_status = status | (1 << 3);
		status_offset = 0;
	}
	//voltage may return normal curve slowly
	if((last_status ^ status) & 3) {
		if(status_cof1) {
			if(cur >= 0 && voltage >= charge_max2) {
				status_offset = 0;
			}
			else {
				status_offset = ((voltage - last_voltage) * status_cof1) / 65536;
				if(cur >= 0 && voltage > charge_max1) {
					status_offset = status_offset * (charge_max2 - voltage) / (charge_max2 - charge_max1);
				}
			}
		}
	}
	last_status = (last_status << 4) | status;
	last_voltage = voltage;
	if(status_offset) {
		if(adc_debug) printk("status offset %d, %d\n", voltage, status_offset);
		voltage += status_offset;
		//may be negative, should use >> 16
		status_offset = (status_offset * status_cof2) / 65536;
	}
	if(adc_debug) printk("battery vol %d, cur %d\n", voltage, cur);
	item->voltage = voltage;
	item->cur = cur;
	if(last_status == 0x11112222 || last_status == 0x22221111) {
		get_rdc();
	}
}

int env_get_gpio(char *label, int pull)
{
	int gpio = env_get_u32(label, INVALID_GPIO);
	if(gpio != INVALID_GPIO) {
		int ret = gpio_request(gpio, label);
		if (ret != 0) {
			printk("env_get_gpio request %s fail %d\n", label, ret);
			return INVALID_GPIO;
		}
		gpio_direction_input(gpio);
		gpio_pull_updown(gpio, pull);
	}
	return gpio;
}

static int * __init v2c_init(int * v2cs, int len)
{
	int i, k;
	int v1, v2, c1, c2;
	int * v2c = kzalloc((len + 1) * 3 * sizeof(int), GFP_KERNEL);
	int * v2cd = v2c;
	v1 = *v2cs++;
	c1 = *v2cs++;
	for(i = 1; i < len; i++) {
		v2 = *v2cs++;
		c2 = *v2cs++;
		*v2cd++ = v1;
		k = (c2 - c1) * 0x10000 / (v2 - v1);
		*v2cd++ = k;
		*v2cd++ = c1 * 0x10000 - v1 * k;
		v1 = v2;
		c1 = c2;
	}
	*v2cd++ = v1;
	*v2cd++ = 0;
	*v2cd++ = 100 * 0x10000;
	*v2cd++ = 0xFFFFFFF;
	return v2c;
}

static int __initdata v2c_table[] = {
	3600,  0, 3648, 10, 3680, 20, 3744, 40, 3855, 60, 
	3916, 80, 4060, 90, 4200, 100
};

static void __init poll_init(struct adc_item * head, int count)
{
	struct adc_item * item = head;
	while(1) {
		item->next = item+1;
		item->pre = item-1;
		if(count == 1) break;
		count--;
		item++;
	}
	item->next = head;
	head->pre = item;
}
static int __init platform_init(void)
{
	int len;
	int adc, cur;
	int v2c[40];

	adc = env_get_u32("power_adc_channel", 0);
	adc_client = adc_register(adc, 0, 0);
	if(!adc_client) {
		printk("failed to register adc %d\n", adc);
		return -1;
	}
	cur = env_get_u32("power_cur_channel", 0);
	if(cur != adc) {
		cur_client = adc_register(cur, 0, 0);
		if(!cur_client) {
			printk("failed to register cur %d\n", cur);
			return -1;
		}
	}
//	adc_battery_device.name = "adc-battery";
//	platform_device_register(&adc_battery_device);

	current_cof1 = env_get_u32("power_current_coff1", 124830); //25milliohm * 21
	current_cof2 = env_get_u32("power_current_coff2", 0);

	status_cof1 = env_get_u32("power_status_coff1", 0);
	status_cof2 = env_get_u32("power_status_coff2", 0);

	voltage_cof1  = env_get_u32("power_vol_coff1",320313);//1023:5V
	voltage_cof2  = env_get_u32("power_vol_coff2",0);
	//charge mode offset
	charge_max1  = env_get_u32("power_charge_max1",4190);
	charge_max2  = env_get_u32("power_charge_max2",4215);

	adc_rdc = pmu_data_read(PMU_BAT_RDC);
	if(adc_rdc <= 0) {
		adc_rdc = env_get_u32("power_bat_rdc", 100);
	}
	printk("adc rdc init to %d\n", adc_rdc);
	adc_max_vol = env_get_u32("power_max_vol", (1023 * voltage_cof1) >> 16);
	adc_min_cur = env_get_u32("power_min_cur", 300);
	adc_ac_level = env_get_u32("power_ac_level", 1);
	adc_usb_level = env_get_u32("power_usb_level", 1);
	adc_charge_level = env_get_u32("power_charge_level", 1);
	adc_ac_gpio = env_get_gpio("power_ac_gpio", adc_ac_level ? GPIOPullDown : GPIOPullUp);
	adc_usb_gpio = env_get_gpio("power_usb_gpio", adc_usb_level ? GPIOPullDown : GPIOPullUp);
	adc_charge_gpio = env_get_gpio("power_charge_gpio", GPIOPullUp);
	adc_switch_gpio = env_get_gpio("power_switch_gpio", PullDisable);
	adc_poll_time  = env_get_u32("power_poll_time",10000);
	adc_poll_count  = env_get_u32("power_poll_count",12);
	if(adc_poll_count < 8) adc_poll_count = 8;
	adc_poll_head = kzalloc(adc_poll_count * sizeof(struct adc_item), GFP_KERNEL);
	adc_poll_items = adc_poll_head;
	poll_init(adc_poll_head, adc_poll_count);

	len = env_cpy_u32s("power_v2c_table",v2c,40);
	if(len < 4) {
		adc_v2c_table = v2c_init(v2c_table, sizeof(v2c_table) >> 3);
	}
	else {
		adc_v2c_table = v2c_init(v2c, len >> 1);
	}

	adc_irq_type = IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING;
	pmu_capacity = pmu_data_read(PMU_BAT_CAP);
	if(pmu_capacity >= 0) {
		adc_update_capacity = update_pmu_capacity;
		printk("adc battery last capacity is %d\n", pmu_capacity);
	}
	return 0;
}

static int adc_bat_get_property(struct power_supply *supply_bat,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = (adc_supply_status & STATUS_CHARGE_DONE) ? POWER_SUPPLY_STATUS_FULL :
		              (adc_supply_status & STATUS_CHARGING) ? POWER_SUPPLY_STATUS_CHARGING : POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = adc_capacity + adc_offset;
		if(val->intval > 100) val->intval = 100;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = adc_voltage*1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 4300*1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = adc_v2c_table[0] * 1000;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (adc_supply_status & STATUS_AC_IN) ? 1 : 0;
		else
			val->intval = (adc_supply_status & STATUS_USB_IN) ? 1 : 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

int adc_get_status(void)
{
	int status = 0;
	if(GPIO_VALID(adc_ac_gpio) && gpio_get_value(adc_ac_gpio) == adc_ac_level) {
		status |= STATUS_CHARGING;
		if(GPIO_VALID(adc_charge_gpio) && gpio_get_value(adc_charge_gpio) == adc_charge_level) {
			status |= STATUS_CHARGE_DONE;
		}
	}
	return status;
}

static void adc_bat_external_power_changed(struct power_supply *supply_bat)
{
	cancel_delayed_work(&adc_bat_work);
	schedule_delayed_work(&adc_bat_work,msecs_to_jiffies(200));
}

static irqreturn_t ac_detect_irq_handler(int irq, void *dev_id)
{
	if (0x00 == irq_hang)
		irq_hang = 0x5a;
	if(adc_running)
		schedule_delayed_work(&adc_supply_work,msecs_to_jiffies(100)); //filter
	if(dac_mute_work.func != NULL)
		schedule_work(&dac_mute_work);
	return IRQ_HANDLED;
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	if(adc_running)
		schedule_delayed_work(&adc_supply_work,msecs_to_jiffies(100)); //filter
	return IRQ_HANDLED;
}

static enum power_supply_property adc_bat_main_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_PRESENT,
};

static enum power_supply_property power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *power_supplied_to[] = {
	"battery",
};

struct power_supply supply_bat = {
	.name			= "battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= adc_bat_main_props,
	.num_properties		= ARRAY_SIZE(adc_bat_main_props),
	.get_property		= adc_bat_get_property,
	.external_power_changed = adc_bat_external_power_changed,
	.use_for_apm		= 1,
};

static struct power_supply supply_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.supplied_to = power_supplied_to,
	.num_supplicants = ARRAY_SIZE(power_supplied_to),
	.properties = power_props,
	.num_properties = ARRAY_SIZE(power_props),
	.get_property = power_get_property,
};

static struct power_supply supply_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.supplied_to = power_supplied_to,
	.num_supplicants = ARRAY_SIZE(power_supplied_to),
	.properties = power_props,
	.num_properties = ARRAY_SIZE(power_props),
	.get_property = power_get_property,
};

static void adc_supply_worker(struct work_struct *work)
{
	int status;
	mutex_lock(&adc_work_lock);
	status = adc_get_status();
	if(status & STATUS_CHARGING) {
		status |= (GPIO_VALID(adc_usb_gpio) && gpio_get_value(adc_usb_gpio) == adc_usb_level) ? 
		          STATUS_USB_IN : STATUS_AC_IN;
	}
	if(status != adc_supply_status) {
		printk("adc_battery status changed from %x to %x\n", adc_supply_status, status);
		if((status & STATUS_AC_IN) != (adc_supply_status & STATUS_AC_IN)) {
			power_supply_changed(&supply_ac);
		}
		if((status & STATUS_USB_IN) != (adc_supply_status & STATUS_USB_IN)) {
			power_supply_changed(&supply_usb);
		}
		if((status & STATUS_CHARGE_DONE) != (adc_supply_status & STATUS_CHARGE_DONE)) {
			adc_bat_external_power_changed(&supply_bat);
		}
		/*
		else if(!(status & STATUS_CHARGING)) {
			//filter out voltage drop caused by plug out of DC
			cancel_delayed_work(&adc_bat_work);
			schedule_delayed_work(&adc_bat_work,msecs_to_jiffies(2000));
		}
		*/
		power_changed(adc_supply_status ^ status, status);
		adc_supply_status = status;
	}
	mutex_unlock(&adc_work_lock);
}

static int get_capacity(int status, struct adc_item * item)
{
	int * table;
	int capacity, voltage;
	if(status & STATUS_CHARGE_DONE) return 100;

	voltage = item->voltage - (adc_rdc * item->cur + 500) / 1000;
	table = adc_v2c_table;

	if(voltage <= *adc_v2c_table) return 0;

	while(voltage > table[3]) table += 3;
	capacity = (voltage* table[1] + table[2]) >> 16;
	if(capacity > 100) capacity = 100;
	return capacity;
}

static struct adc_item * adc_add_item(struct adc_item * head, struct adc_item * item)
{
	int first = 1;
	int capacity = item->capacity;
	int voltage = item->voltage;
	struct adc_item * cur = head;
	while(cur->capacity > capacity || (cur->capacity == capacity && cur->voltage > voltage)) {
		first = 0;
		cur = cur->next;
		if(cur == head) {
			break;
		}
	}
	item->next = cur;
	item->pre = cur->pre;
	cur->pre->next = item;
	cur->pre = item;
	return first ? item : head;
}

static int adc_get_average(struct adc_item * head, int count,int charging)
{
	int skip, sum = 0;
	if(charging) {
		skip = 2;
		count -= 4;
	}
	else {
		skip = 1;
		count -= ((count >> 1) + 1);
	}
	while(skip--) head = head->next;
	skip = count;
	while(skip--) {
		sum += head->capacity;
		head = head->next;
	}
	return (sum + (count >> 1)) / count;
}

static void adc_bat_worker(struct work_struct *work)
{
	struct adc_item * item, * head;
	int status, capacity, delay, update = 0, count = adc_poll_count;

	if(adc_poll_supply || adc_poll_init == 1) {
		adc_supply_worker(0);
	}
	mutex_lock(&adc_work_lock);
	status = adc_supply_status & STATUS_CHARGE_MASK;
	if (status != adc_bat_status) {
		adc_bat_status = status;
		update = 1;
		if(adc_poll_init) {
			//reset sample buffer
			adc_poll_init = 1;
		}
		goto update;
	}
	item = adc_poll_items + adc_poll_index;
	adc_get_voltage(item);
	adc_voltage = item->voltage;
	capacity = get_capacity(status, item);
	//update charge status by current
	if(item->cur < 0) {
		status &= ~STATUS_CHARGING;
	}
	else {
		status |= STATUS_CHARGING;
	}
	if(!adc_poll_init && ((!status) == (capacity > adc_capacity))) {
		capacity = adc_capacity;
	}
	item->capacity = capacity;
	item->next->pre = item->pre;
	item->pre->next = item->next;
	if(adc_poll_head == item) {
		adc_poll_head = item->next;
	}
	head = adc_add_item(adc_poll_head, item);
	if(adc_debug) {
		struct adc_item * cur = head;
		printk("battery capacity %d:", capacity);
		while(1) {
			printk("%d ", cur->capacity);
			cur = cur->next;
			if(cur == head) {
				break;
			}
		}
		printk("\n");
	}
	adc_poll_head = head;

	adc_poll_index++;
	if(adc_poll_index == count) {
		adc_poll_index = 0;
	}
	if(adc_poll_init) {
		if(adc_poll_init == count) {
			if(head->capacity + 4 > head->pre->capacity) {
				adc_poll_init = 0;
				update = 2;
			}
			else {
				printk("battery init min %d, max %d\n", head->capacity, head->pre->capacity);
				if(adc_capacity == 100) {
					adc_capacity = adc_get_average(head, count, status);
					if(adc_capacity == 0) {
						adc_capacity = 1; //not valid vol,should not poweroff now
					}
				}
			}
		}
		else {
			adc_poll_init++;
		}
	}
	if(adc_poll_init == 0 && (update == 2 || capacity != adc_capacity)) {
		capacity = adc_get_average(head, count, status);
		if(capacity == 100 && status == STATUS_CHARGING) {
			capacity = 99;
		}
		if(update == 2 || capacity != adc_capacity) {
			if(update != 2) {
				if((status ? 1 : 0) == (capacity > adc_capacity)) {
					adc_capacity += status ? 1 : -1;
					update = 2;
				}
			}
			else {
				adc_capacity = capacity;
			}
			printk("battery report %d %d real %d\n", adc_voltage, adc_capacity, capacity);
			if(adc_update_capacity) {
				adc_update_capacity(adc_capacity + adc_offset);
			}
		}
	}
update:
	if(adc_poll_init == 0) {
		if(adc_debug >= 1000) {
			delay = adc_debug;
		}
		else {
			delay = adc_poll_time;
		}
		if (update) {
			power_supply_changed(&supply_bat);
		}
	}
	else {
		delay = 150;
	}

	if(delay) {
		schedule_delayed_work(&adc_bat_work,msecs_to_jiffies(delay));
	}
	mutex_unlock(&adc_work_lock);
}

#ifdef CONFIG_PM
static int adc_bat_suspend(struct platform_device *dev, pm_message_t state)
{
	adc_running = 0;
	cancel_delayed_work(&adc_bat_work);
	cancel_delayed_work(&adc_supply_work);
	flush_scheduled_work();

	irq_hang = 0x00;
	return 0;
}

static int adc_bat_resume(struct platform_device *dev)
{
	adc_poll_init = 1;
	schedule_delayed_work(&adc_supply_work,msecs_to_jiffies(10));
	schedule_delayed_work(&adc_bat_work,msecs_to_jiffies(10));
	adc_running = 1;

	if (0x5a == irq_hang) {
		rk28_send_wakeup_key();
	}
	irq_hang = 0xa5;

	return 0;
}
#else
#define adc_bat_suspend NULL
#define adc_bat_resume NULL
#endif

//return -1 need poll
static int request_gpio_irq(int gpio,int type,int * pirq, irq_handler_t handler)
{
	int ret;
	if(!GPIO_VALID(gpio)) {
		*pirq = -1;
		return 0;
	}
	*pirq = gpio_to_irq(gpio);
	if(*pirq < 0) {
		printk("adc_battery get irq for 0x%x fail\n",gpio);
		return *pirq;
	}

	ret = request_irq(*pirq, handler,type, supply_bat.name, 0);
	if (ret < 0) {
		printk("adc_battery request irq for 0x%x fail\n",gpio);
	}
	return ret;
}

static ssize_t adc_show_debug(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "battery debug %d\nmax offset %d\nbattery rdc %d\n", 
	       adc_debug, adc_offset, adc_rdc);
}

static ssize_t adc_store_debug(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	if(!memcmp(buf, "offset ", 7)) {
		int offset = 0;
		if(sscanf(buf+7, "%d", &offset) == 1) {
			adc_offset = offset;
			printk("battery cap offset set to %d\n", adc_offset);
		}
	}
	else if(!memcmp(buf, "rdc ", 4)) {
		int rdc = 0;
		if(sscanf(buf+4, "%d", &rdc) == 1) {
			adc_rdc = rdc;
			printk("battery rdc set to %d\n", adc_rdc);
		}
	}
	else {
		int debug = 0;
		if(!memcmp(buf, "debug ", 6)) buf += 6;
		if(sscanf(buf, "%d", &debug) == 1) {
			adc_debug = debug;
			printk("battery debug set to %d\n", adc_debug);
		}
	}
	return count;
}

static ssize_t adc_show_voltage_raw(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",get_adc(adc_client));
}

static ssize_t adc_store_voltage_raw(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}
static ssize_t adc_show_temp(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",290);
}

static ssize_t adc_store_temp(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t adc_show_health(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n","Good");
}

static ssize_t adc_store_health(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}
#define ADC_CHG_ATTR(_name)					\
{									\
	.attr = { .name = #_name,.mode = 0644 },					\
	.show =  adc_show_##_name,				\
	.store = adc_store_##_name, \
}

static struct device_attribute adc_charger_attrs[] = {
	ADC_CHG_ATTR(debug),
	ADC_CHG_ATTR(voltage_raw),
	ADC_CHG_ATTR(temp),
	ADC_CHG_ATTR(health),
};

static void adc_init_attrs(struct power_supply *psy)
{
	int j, ret;
	for (j = 0; j < ARRAY_SIZE(adc_charger_attrs); j++) {
		ret = device_create_file(psy->dev, &adc_charger_attrs[j]);
	}
}

static int adc_bat_probe(struct platform_device *dev)
{
	int ret;
	adc_poll_init = 1;
	adc_capacity = 100;

	printk("adc_battery ac 0x%x usb 0x%x charge 0x%x\n", adc_ac_gpio, adc_usb_gpio, adc_charge_gpio);

	mutex_init(&adc_work_lock);
	INIT_DELAYED_WORK(&adc_bat_work, adc_bat_worker);
	INIT_DELAYED_WORK(&adc_supply_work, adc_supply_worker);

	ret = power_supply_register(&dev->dev, &supply_bat);
	if(ret)
		goto bat_supply_failed;
	ret = power_supply_register(&dev->dev, &supply_ac);
	if(ret)
		goto ac_supply_failed;
	ret = power_supply_register(&dev->dev, &supply_usb);
	if(ret)
		goto usb_supply_failed;

	adc_irq1 = adc_irq2 = -1;
	if(adc_irq_type) {
		if(request_gpio_irq(adc_ac_gpio,adc_irq_type,&adc_irq1, ac_detect_irq_handler) < 0) {
			adc_poll_supply = 1;
		} else {
			enable_irq_wake(gpio_to_irq(adc_ac_gpio));
		}
		if(request_gpio_irq(adc_charge_gpio,adc_irq_type,&adc_irq2, gpio_irq_handler) < 0) {
			adc_poll_supply = 1;
		}
	}
	else {
		adc_poll_supply = 1;
	}
	if(adc_poll_supply) {
		printk("dc detect run in poll mode\n");
	}
	adc_init_attrs(&supply_bat);
	schedule_delayed_work(&adc_supply_work,msecs_to_jiffies(10));
	adc_running = 1;
	return 0;

usb_supply_failed:
	power_supply_unregister(&supply_ac);
ac_supply_failed:
	power_supply_unregister(&supply_bat);
bat_supply_failed:
	return ret;
}

static int __devexit adc_bat_remove(struct platform_device *dev)
{
	cancel_delayed_work(&adc_bat_work);
	cancel_delayed_work(&adc_supply_work);
	flush_scheduled_work();
	power_supply_unregister(&supply_bat);
	power_supply_unregister(&supply_ac);
	power_supply_unregister(&supply_usb);
	if(adc_irq1 >= 0) free_irq(adc_irq1,0);
	if(adc_irq2 >= 0) free_irq(adc_irq2,0);
	return 0;
}

static struct platform_driver adc_bat_driver = {
	.probe		= adc_bat_probe,
	.remove		= __devexit_p(adc_bat_remove),
	.suspend	= adc_bat_suspend,
	.resume		= adc_bat_resume,
	.driver		= {
		.name	= "adc-battery",
		.owner	= THIS_MODULE,
	},
};

static int __init adc_bat_init(void)
{
	if(platform_init()) return -EINVAL;
	return platform_driver_register(&adc_bat_driver);
}

static void __exit adc_bat_exit(void)
{
	kfree(adc_v2c_table);
	kfree(adc_poll_items);
	platform_driver_unregister(&adc_bat_driver);
}

fs_initcall(adc_bat_init);
module_exit(adc_bat_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("liqiangman@yftech.com");
MODULE_DESCRIPTION("ADC based battery driver");

