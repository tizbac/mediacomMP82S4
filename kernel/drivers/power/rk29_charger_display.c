	 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/reboot.h>
#include <asm/unaligned.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <asm/uaccess.h>
#include <linux/power_supply.h>
#include <mach/yfmach.h>


#if 1
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif

//#define RK29_PLAY_ON_PIN RK29_PIN6_PA7
//#define MAX_PRE_CNT 2
//#define DET_CNT   5
#define PWR_ON_THRESHD 5       //power on threshd of capacity
//unsigned int   pre_cnt = 0;   //for long press counter 
//int charge_disp_mode = 0;
static int pwr_on_thrsd = 5;          //power on capcity threshold

//extern int boot_mode_init(char * s);

static int __init pwr_on_thrsd_setup(char *str)
{

	pwr_on_thrsd = simple_strtol(str,NULL,10);
	printk(KERN_INFO "power on threshold:%d",pwr_on_thrsd);
	return 0;
}

__setup("pwr_on_thrsd=", pwr_on_thrsd_setup);

static int usb_status;
static int ac_status;
static int __rk_get_system_battery_status(struct device *dev, void *data)
{
	union power_supply_propval val_status = {POWER_SUPPLY_STATUS_DISCHARGING};
	struct power_supply *psy = dev_get_drvdata(dev);

	psy->get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val_status);

	if (val_status.intval != 0) {
		if (psy->type == POWER_SUPPLY_TYPE_USB)
			usb_status = POWER_SUPPLY_TYPE_USB;
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			ac_status = POWER_SUPPLY_TYPE_MAINS;
	}

	return 0;
}

// POWER_SUPPLY_TYPE_BATTERY --- discharge
// POWER_SUPPLY_TYPE_USB     --- usb_charging
// POWER_SUPPLY_TYPE_MAINS   --- AC_charging
int rk_get_system_battery_status(void)
{
	class_for_each_device(power_supply_class, NULL, NULL, __rk_get_system_battery_status);

	if (ac_status == POWER_SUPPLY_TYPE_MAINS) {
		return POWER_SUPPLY_TYPE_MAINS;
	} else if (usb_status == POWER_SUPPLY_TYPE_USB) {
		return POWER_SUPPLY_TYPE_USB;
	}

	return POWER_SUPPLY_TYPE_BATTERY;
}
EXPORT_SYMBOL(rk_get_system_battery_status);

static union power_supply_propval battery_capacity = { 100 };
static int __rk_get_system_battery_capacity(struct device *dev, void *data)
{
	struct power_supply *psy = dev_get_drvdata(dev);

	if (psy->type == POWER_SUPPLY_TYPE_BATTERY) {
		psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &battery_capacity);
	}

	return 0;
}

int rk_get_system_battery_capacity(void)
{
	class_for_each_device(power_supply_class, NULL, NULL, __rk_get_system_battery_capacity);

	return battery_capacity.intval;
}
EXPORT_SYMBOL(rk_get_system_battery_capacity);

static union power_supply_propval battery_voltage = { 100 };
static int __rk_get_system_battery_voltage(struct device *dev, void *data)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	if (psy->type == POWER_SUPPLY_TYPE_BATTERY) {
		psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &battery_voltage);
	}
	return 0;
}

static int battery_voltage_adc(void);

static int rk_get_system_battery_voltage(void)
{
	class_for_each_device(power_supply_class, NULL, NULL, __rk_get_system_battery_voltage);
#if defined (CONFIG_KP_AXP22)
	if (pmic_is_axp228()) {		// 开机阶段取不到电压值
		battery_voltage.intval = battery_voltage_adc();
	}
#endif
	return battery_voltage.intval;
}

#ifdef CONFIG_POWER_ON_CHARGER_DISPLAY
//int charger_mode=0;	     	//1:charge,0:not charge
static void add_bootmode_charger_to_cmdline(void)
{
	char *pmode=" androidboot.mode=charger";
	//int off = strlen(saved_command_line);
	char *new_command_line = kzalloc(strlen(saved_command_line) + strlen(pmode) + 1, GFP_KERNEL);
	sprintf(new_command_line, "%s%s", saved_command_line, pmode);
	saved_command_line = new_command_line;
	//strcpy(saved_command_line+off,pmode);

	//int off = strlen(boot_command_line);
	//strcpy(boot_command_line+off,pmode);

	printk("Kernel command line: %s\n", saved_command_line);
}

//display charger logo in kernel CAPACITY

// yftech
#include <linux/adc.h>
extern int env_get_gpio(char *label, int pull);
#ifndef GPIO_VALID
	#define GPIO_VALID(x)   ((x) != 0xFFFFFFFF)
#endif

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

static int ac_inside(void)
{
	int adc_ac_gpio, adc_ac_level, ret = 0;

#if defined (CONFIG_KP_AXP22)
	extern int axp22_ac_valid;
	if (axp22_ac_valid)
		return 1;
#endif

	adc_ac_level = env_get_u32("power_ac_level", 1);
	adc_ac_gpio = env_get_gpio("power_ac_gpio", adc_ac_level ? GPIOPullDown : GPIOPullUp);

	if (GPIO_VALID(adc_ac_gpio)) {
		if (gpio_get_value(adc_ac_gpio) == adc_ac_level)
			ret = 1;
		gpio_free(adc_ac_gpio);
	}

	return ret;
}

static int battery_voltage_adc(void) {
	int adc, voltage;
	int voltage_cof1, voltage_cof2;
	struct adc_client * adc_client;

	adc = env_get_u32("power_adc_channel", 0);
	voltage_cof1  = env_get_u32("power_vol_coff1", 320313);//1023:5V
	voltage_cof2  = env_get_u32("power_vol_coff2", 0);

	adc_client = adc_register(adc, 0, 0);
	if (!adc_client) {
		printk("failed to register adc %d\n", adc);
		return 0;
	}

	voltage = get_adc(adc_client);
	voltage = ((voltage * voltage_cof1) >> 18) + voltage_cof2;
	printk("battery adc=%d\n", voltage);

	adc_unregister(adc_client);

	return voltage;
}

static int battery_voltage_low(void) {
	int voltage, protect_voltage;

#if defined (CONFIG_KP_AXP22)
	if (pmic_is_axp228()) {
		int cap = rk_get_system_battery_capacity();
		return cap <= 0;
	}
#endif

	voltage = battery_voltage_adc();
	protect_voltage = env_get_u32("battery_protect_voltage", 3420);
	return voltage < protect_voltage;
}

static int  __init check_battery_low(void)
{
	int i;

	DBG("%s\n", __func__);
	for (i=0; i<5; i++) {
		if (ac_inside() || !battery_voltage_low())
			return 0;

		mdelay(5);
	}
	printk("battery low!\n");
	kernel_power_off();
	while(1);
}
fs_initcall(check_battery_low);
// end

static int  __init start_charge_logo_display(void)
{
	int capacity;
	union power_supply_propval val_status = {POWER_SUPPLY_STATUS_DISCHARGING};
	union power_supply_propval val_capacity ={ 100} ;
	union power_supply_propval val_voltage = {0};
	bool charger_cmd = false;

	printk("start_charge_logo_display\n");

	if(board_boot_mode() == BOOT_MODE_RECOVERY)  //recovery mode
	{
		printk("recovery mode \n");
		return 0;
	}

	if(env_get_u32("power_disable_charger", 0))
	{
		printk("charger mode disabled\n");
		return 0;
	}

	capacity = pmu_data_read(PMU_BAT_CAP);
	if(capacity > 0 && (capacity & 0x80))
	{
		charger_cmd = true;
		printk("power in low power mode\n");
	}

	if (rk_get_system_battery_status() != POWER_SUPPLY_TYPE_BATTERY)
		val_status.intval = POWER_SUPPLY_STATUS_CHARGING;

	val_capacity.intval = rk_get_system_battery_capacity();
	val_voltage.intval = rk_get_system_battery_voltage();
	DBG("mode=%d, cap=%d,vol=%d\n", board_boot_mode(), val_capacity.intval, val_voltage.intval);

	// low power   and  discharging
#if 0
	if((val_capacity.intval < pwr_on_thrsd )&&(val_status.intval != POWER_SUPPLY_STATUS_CHARGING))
	{
		printk("low power\n");
		kernel_power_off();
		while(1);
		return 0;
	}
#endif


	//low power and charging
#if 0
	if((val_capacity.intval < pwr_on_thrsd )&&(val_status.intval == POWER_SUPPLY_STATUS_CHARGING))
	{
		while((val_capacity.intval < pwr_on_thrsd ))
		{
			list_for_each_entry(psy, &rk_psy_head, rk_psy_node)
			{
				psy->get_property(psy,POWER_SUPPLY_PROP_CAPACITY,&val_capacity); 
			}

			//printk("charging ... \n");
		}
	}

#endif

	if(val_status.intval == POWER_SUPPLY_STATUS_CHARGING)
	{
		int mode = board_boot_mode();
		switch (mode) {
			case BOOT_MODE_NORMAL:
			case BOOT_MODE_CHARGE:
				charger_cmd = true;
				printk("power in charge mode\n");
				break;

			case BOOT_MODE_REBOOT:
				if (charger_cmd &&
					((val_capacity.intval >= 5) || (val_voltage.intval > 3600000))) {
					DBG("force start\n");
					charger_cmd = false;
				}
				break;
		}
	}

	if (charger_cmd)
		add_bootmode_charger_to_cmdline();

	return 0;
} 

//subsys_initcall_sync(start_charge_logo_display);
module_init(start_charge_logo_display);
#endif
