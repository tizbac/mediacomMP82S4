/*
 * RockChip ADC Battery Driver 
 * Copyright (C) 2012, RockChip
 *
 * Authors: xuhuicong <xhc@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/power/cw2015_battery.h>
#include <linux/time.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <mach/board.h>

#define REG_VERSION             0x0
#define REG_VCELL               0x2
#define REG_SOC                 0x4
#define REG_RRT_ALERT           0x6
#define REG_CONFIG              0x8
#define REG_MODE                0xA
#define REG_BATINFO             0x10

#define MODE_SLEEP_MASK         (0x3<<6)
#define MODE_SLEEP              (0x3<<6)
#define MODE_NORMAL             (0x0<<6)
#define MODE_QUICK_START        (0x3<<4)
#define MODE_RESTART            (0xf<<0)

#define CONFIG_UPDATE_FLG       (0x1<<1)
#define ATHD                    (0x0<<3)        //ATHD =0%

#define CW_I2C_SPEED            100000          // default i2c speed set 100khz

#define BATTERY_UP_MAX_CHANGE   300             // the max time allow battery change quantity
#define BATTERY_DOWN_CHANGE   60                // the max time allow battery change quantity
#define BATTERY_DOWN_MIN_CHANGE_RUN 30          // the min time allow battery change quantity when run
#define BATTERY_DOWN_MIN_CHANGE_SLEEP 1800      // the min time allow battery change quantity when run 30min

#define BATTERY_DOWN_MAX_CHANGE_RUN_AC_ONLINE 3600
#define SYSTEM_SHUTDOWN_VOLTAGE  3450000        //set system shutdown voltage related in battery info.

#define Enable_BATDRV_LOG       0
//#define Enable_USB_CHG          1               //!!!!if use USB_Charge must open this line!!!!
#define BAT_LOW_INTERRUPT    1

#define USB_CHARGER_MODE        1
#define AC_CHARGER_MODE         2


#if Enable_BATDRV_LOG
#define xprintk(format, arg...)  printk("func: %s\n" format,  __FUNCTION__, ##arg)
#else
#define xprintk(format, ...)
#endif

struct cw_battery {
        struct i2c_client *client;
        struct workqueue_struct *battery_workqueue;
        struct delayed_work battery_delay_work;
        struct delayed_work dc_wakeup_work;
        struct delayed_work bat_low_wakeup_work;
        const struct cw_bat_platform_data *plat_data;

        struct power_supply rk_bat;
        struct power_supply rk_ac;
        struct power_supply rk_usb;

        long sleep_time_capacity;      // the sleep time from capacity change to present, it will set 0 when capacity change 
        long run_time_capacity;

        long sleep_time_ac_online;      // the sleep time from insert ac to present, it will set 0 when insert ac
        long run_time_ac_online;

        int dc_online;
        int usb_online;
        int charger_mode;
        //int charger_init_mode;
        int capacity;
        int voltage;
        int status;
        int time_to_empty;
        int alt;

        int bat_change;
};

// yftech
#include <mach/yfmach.h>

// 赛微微电子（CellWise）电池测试中心, 电池条码：109313500049&109313500041, 电池容量：8000mAh（两并一串）
static const u8 bat_profile_8000ah[SIZE_BATINFO] = {
0x15, 0x7E, 0x5D, 0x5F, 0x5C, 0x5A, 0x56,
0x52, 0x4E, 0x4B, 0x46, 0x43, 0x40, 0x3A,
0x35, 0x2E, 0x24, 0x1F, 0x1C, 0x1B, 0x21,
0x2C, 0x40, 0x43, 0x48, 0x4A, 0x0C, 0xCD,
0x1E, 0x3C, 0x33, 0x56, 0x56, 0x4F, 0x4F,
0x52, 0x41, 0x16, 0x3F, 0x37, 0x00, 0x1E,
0x52, 0x87, 0x8F, 0x91, 0x94, 0x52, 0x82,
0x8C, 0x92, 0x96, 0x80, 0x5F, 0xA1, 0xCB,
0x2F, 0x7D, 0x72, 0xA5, 0xB5, 0xC1, 0x46,
0xAE
};

static const u8 bat_profile_8000_ah_91[SIZE_BATINFO] = {
0x15, 0x7E, 0x5D, 0x5F, 0x5C, 0x5A, 0x56,
0x52, 0x4E, 0x4B, 0x46, 0x43, 0x40, 0x3A,
0x35, 0x2E, 0x24, 0x1F, 0x1C, 0x1B, 0x21,
0x2C, 0x40, 0x43, 0x48, 0x4A, 0x0C, 0xCD,
0x1E, 0x3C, 0x33, 0x56, 0x56, 0x4F, 0x4F,
0x52, 0x41, 0x16, 0x3F, 0x37, 0x00, 0x1E,
0x52, 0x87, 0x8F, 0x91, 0x94, 0x52, 0x82,
0x8C, 0x92, 0x96, 0x80, 0x5F, 0xA1, 0xCB,
0x2F, 0x7D, 0x72, 0xA5, 0xB5, 0xC1, 0x36,
0x31
};

static const u8 bat_profile_7000ah[SIZE_BATINFO] = {
0x15, 0x7E, 0x6D, 0x66, 0x5C, 0x64, 0x56,
0x52, 0x50, 0x4C, 0x49, 0x49, 0x45, 0x40,
0x34, 0x12, 0x08, 0x0B, 0x17, 0x42, 0x5B,
0x60, 0x5B, 0x5A, 0x5E, 0x59, 0x0C, 0xCD,
0x25, 0x45, 0x59, 0x5F, 0x60, 0x61, 0x62,
0x61, 0x3B, 0x17, 0x8C, 0x00, 0x00, 0x28,
0x52, 0x87, 0x8F, 0x91, 0x94, 0x52, 0x82,
0x8C, 0x92, 0x96, 0x81, 0x5D, 0x8C, 0xCB,
0x2F, 0x7D, 0x72, 0xA5, 0xB5, 0xC1, 0x46,
0xAE
};

// BP4700_ProfileV3LT
static const u8 bat_profile_4700ah[SIZE_BATINFO] = {
0x15, 0xC5, 0x62, 0x64, 0x60, 0x5E, 0x5A,
0x52, 0x50, 0x4C, 0x47, 0x43, 0x3E, 0x3A,
0x37, 0x33, 0x28, 0x22, 0x1C, 0x1B, 0x18,
0x24, 0x3F, 0x49, 0x46, 0x33, 0x0C, 0xCD,
0x15, 0x2A, 0x4B, 0x83, 0x6F, 0x67, 0x62,
0x62, 0x3E, 0x18, 0x10, 0x39, 0x00, 0x35,
0x52, 0x87, 0x8F, 0x91, 0x94, 0x52, 0x82,
0x8C, 0x92, 0x96, 0xFF, 0xA8, 0xDE, 0xCB,
0x2F, 0x7D, 0x72, 0xA5, 0xB5, 0xC1, 0x46,
0xAE
};

struct cw_bat_platform_data platform_data = {
       .dc_det_pin = INVALID_GPIO,
       .dc_det_level = 0,
       .bat_low_pin = INVALID_GPIO,
       .bat_low_level = 0,
       .chg_ok_pin = INVALID_GPIO,
       .chg_ok_level = 0,
       .cw_bat_config_info = NULL,
       .chg_mode_sel_pin = INVALID_GPIO,
       .chg_mode_sel_level = 0,
};

#define CW2015_I2C_ADDR                0x62
static struct i2c_board_info i2c_info_dev =  {
       I2C_BOARD_INFO("cw201x", CW2015_I2C_ADDR),
       .platform_data  = &platform_data,
};

static int add_device(void) {
       int twi_id = 0;
       int count = 3;
       struct i2c_adapter *adapter;
       struct i2c_client * client;
       struct i2c_msg msgs[2] = {{0}, {0}};
       int status, capacity;
       char addr = REG_VERSION, data;

       adapter = i2c_get_adapter(twi_id);

       msgs[0].addr = CW2015_I2C_ADDR;
       msgs[0].len = 1;
       msgs[0].buf = &addr;
       msgs[0].scl_rate = CW_I2C_SPEED;

       msgs[1].addr = CW2015_I2C_ADDR;
       msgs[1].flags = I2C_M_RD;
       msgs[1].len = 1;
       msgs[1].buf = &data;
       msgs[1].scl_rate = CW_I2C_SPEED;

       while((status = i2c_transfer(adapter, msgs, 2)) != 2 && count--);
       printk("cw2015 try count %d\n", count);
       if (status != 2) {
               printk("Not find cw2015!\n");
               return -1;
       }
       printk("Find cw2015 version=%#x!\n", data);

       platform_data.dc_det_level = env_get_u32("power_ac_level", 1);
       platform_data.dc_det_pin   = env_get_u32("power_ac_gpio", INVALID_GPIO);
       if(platform_data.dc_det_pin == INVALID_GPIO) {
               platform_data.dc_det_pin = 0;
               printk("Must be set power_ac_gpio!");
       }

       platform_data.chg_ok_level = env_get_u32("power_charge_level", 1);
       platform_data.chg_ok_pin   = env_get_u32("power_charge_gpio", INVALID_GPIO);

		capacity = env_get_u32("battery_capacity", 4700);
		printk("cw2015 use %dmAH!\n", capacity);
		switch (capacity) {
			default:
			case 4700:
				platform_data.cw_bat_config_info = bat_profile_4700ah;
				break;

			case 7000:
				platform_data.cw_bat_config_info = bat_profile_7000ah;
				break;

			case 8000:
				platform_data.cw_bat_config_info = bat_profile_8000ah;
				break;

			case 8091:
				platform_data.cw_bat_config_info = bat_profile_8000_ah_91;
				break;
		}

       client = i2c_new_device(adapter, &i2c_info_dev);

       return 0;
}

extern void update_pmu_capacity(int capacity);
// end

static int cw_read(struct i2c_client *client, u8 reg, u8 buf[])
{
        int ret;
        ret = i2c_master_reg8_recv(client, reg, buf, 1, CW_I2C_SPEED);
        return ret;
}

static int cw_write(struct i2c_client *client, u8 reg, u8 const buf[])
{
        int ret;
        ret = i2c_master_reg8_send(client, reg, buf, 1, CW_I2C_SPEED);
        return ret;
}
static int cw_read_word(struct i2c_client *client, u8 reg, u8 buf[])
{
        int ret;
        ret = i2c_master_reg8_recv(client, reg, buf, 2, CW_I2C_SPEED);
        return ret;
}

#if 0
static int cw_write_word(struct i2c_client *client, u8 reg, u8 const buf[])
{
        int ret;
        ret = i2c_master_reg8_send(client, reg, buf, 2, CW_I2C_SPEED);
        return ret;
}
#endif



static int cw_update_config_info(struct cw_battery *cw_bat)
{
        int ret;
        u8 reg_val;
        int i;
        u8 reset_val;

        // printk("func: %s-------\n", __func__);
        /* make sure no in sleep mode */
        ret = cw_read(cw_bat->client, REG_MODE, &reg_val);
        if (ret < 0)
                return ret;

        reset_val = reg_val;
        if((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
                printk("Error, device in sleep mode, cannot update battery info\n");
                return -1;
        }

        /* update new battery info */

        for (i = 0; i < SIZE_BATINFO; i++) {
                xprintk("cw_bat->plat_data->cw_bat_config_info[%d] = 0x%x\n", i, \
                                cw_bat->plat_data->cw_bat_config_info[i]);
                ret = cw_write(cw_bat->client, REG_BATINFO + i, &cw_bat->plat_data->cw_bat_config_info[i]);

                if (ret < 0) 
                        return ret;
        }

        /* readback & check */
        for (i = 0; i < SIZE_BATINFO; i++) {
                ret = cw_read(cw_bat->client, REG_BATINFO + i, &reg_val);
                if (reg_val != cw_bat->plat_data->cw_bat_config_info[i])
                        return -1;
        }

        
        /* set cw2015 to use new battery info */
        ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
        if (ret < 0)
                return ret;

        reg_val |= CONFIG_UPDATE_FLG;/* set UPDATE_FLAG */
        reg_val &= 0x07;    // clear ATHD
        reg_val |= ATHD;    // set ATHD
        ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
        if (ret < 0)
                return ret;

        /* check 2015 for ATHD & update_flag */ 
        ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
        if (ret < 0)
                return ret;
        
        if (!(reg_val & CONFIG_UPDATE_FLG)) {
                printk("update flag for new battery info have not set..\n");
        }

        if ((reg_val & 0xf8) != ATHD) {
                printk("the new ATHD have not set..\n");
        }

        /* reset */
        reset_val &= ~(MODE_RESTART);
        reg_val = reset_val | MODE_RESTART;
        ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
        if (ret < 0)
                return ret;

        msleep(10);
        ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
        if (ret < 0)
                return ret;
        
        return 0;
}

static int cw_init(struct cw_battery *cw_bat)
{
        int ret;
        int i;
        u8 reg_val = MODE_SLEEP;
#if 0
        ret = cw_read(cw_bat->client, REG_MODE, &reg_val);
        if (ret < 0)
                return ret;
#endif
        if ((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
                reg_val = MODE_NORMAL;
                ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
                if (ret < 0) 
                        return ret;
        }

        ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
        if (ret < 0)
                return ret;

        if ((reg_val & 0xf8) != ATHD) {
                printk("the new ATHD have not set\n");
                reg_val &= 0x07;    // clear ATHD
                reg_val |= ATHD;    // set ATHD
                ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
                if (ret < 0)
                        return ret;
        }

        ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
        if (ret < 0) 
                return ret;

        if (!(reg_val & CONFIG_UPDATE_FLG)) {
                printk("update flag for new battery info have not set\n");
                ret = cw_update_config_info(cw_bat);
                if (ret < 0)
                        return ret;
        } else {
                for(i = 0; i < SIZE_BATINFO; i++) { 
                        ret = cw_read(cw_bat->client, (REG_BATINFO + i), &reg_val);
                        if (ret < 0)
                                return ret;
                        
                        if (cw_bat->plat_data->cw_bat_config_info[i] != reg_val)
                                break;
                }

                if (i != SIZE_BATINFO) {
                        printk("update flag for new battery info have not set\n"); 
                        ret = cw_update_config_info(cw_bat);
                        if (ret < 0)
                                return ret;
                }
        }

        for (i = 0; i < 30; i++) {
                ret = cw_read(cw_bat->client, REG_SOC, &reg_val);
                if (ret < 0)
                        return ret;
                else if (ret != 0xff) 
                        break;
                
                msleep(100);
                if (i > 25)
                        printk("cw2015 input unvalid power error\n");

        }
        
        return 0;
}

static void cw_update_time_member_ac_online(struct cw_battery *cw_bat)
{
        struct timespec ts;
        int new_run_time;
        int new_sleep_time;

        ktime_get_ts(&ts);
        new_run_time = ts.tv_sec;

        get_monotonic_boottime(&ts);
        new_sleep_time = ts.tv_sec - new_run_time;

        cw_bat->run_time_ac_online = new_run_time;
        cw_bat->sleep_time_ac_online = new_sleep_time; 
}

static void cw_update_time_member_capacity(struct cw_battery *cw_bat)
{
        struct timespec ts;
        int new_run_time;
        int new_sleep_time;

        ktime_get_ts(&ts);
        new_run_time = ts.tv_sec;

        get_monotonic_boottime(&ts);
        new_sleep_time = ts.tv_sec - new_run_time;

        cw_bat->run_time_capacity = new_run_time;
        cw_bat->sleep_time_capacity = new_sleep_time; 
}

static int cw_quickstart(struct cw_battery *cw_bat)
{
        int ret = 0;
        u8 reg_val = MODE_QUICK_START;

        xprintk("---------function: %s\n", __func__);
        ret = cw_write(cw_bat->client, REG_MODE, &reg_val);     //(MODE_QUICK_START | MODE_NORMAL));  // 0x30
        if(ret < 0) {
                printk("Error quick start1\n");
                return ret;
        }
        
        reg_val = MODE_NORMAL;
        ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
        if(ret < 0) {
                printk("Error quick start2\n");
                return ret;
        }
        //printk("---------function: %s, cw_get_capacity = %d\n", __func__, cw_get_capacity(cw_bat));
        return 1;
}

static int cw_get_capacity(struct cw_battery *cw_bat)
{
        int cw_capacity;
        int ret;
        u8 reg_val[2];

        struct timespec ts;
        long new_run_time;
        long new_sleep_time;
        long capacity_or_aconline_time;
        int allow_change;
        int allow_capacity;
        static int if_quickstart = 0;
        static int jump_flag =0;


        ret = cw_read_word(cw_bat->client, REG_SOC, reg_val);
        if (ret < 0)
                return ret;
        
        // printk("----------------the cw201x capacity = %d, funciton: %s, line: %d\n", reg_val,  __func__, __LINE__);
        cw_capacity = reg_val[0];
        if ((cw_capacity < 0) || (cw_capacity > 100)) {
                printk("get cw_capacity error; cw_capacity = %d\n", cw_capacity);
                return cw_capacity;
        } 

        if (cw_capacity == 0){ 
              xprintk("the cw201x capacity is 0 !!!!!!!, funciton: %s, line: %d\n", __func__, __LINE__);
              }
        else 
                xprintk("the cw201x capacity is %d, funciton: %s\n", cw_capacity, __func__);

        //ret = cw_read(cw_bat->client, REG_SOC + 1, &reg_val);

        ktime_get_ts(&ts);
        new_run_time = ts.tv_sec;

        get_monotonic_boottime(&ts);
        new_sleep_time = ts.tv_sec - new_run_time;
        if (((cw_bat->charger_mode > 0) && (cw_capacity <= (cw_bat->capacity - 1)) && (cw_capacity >= (cw_bat->capacity - 8)))
                        || ((cw_bat->charger_mode == 0) && (cw_capacity == (cw_bat->capacity + 1)))) {             // modify battery level swing

                if (!(cw_capacity == 0 && cw_bat->capacity == 1)) {			
		                  cw_capacity = cw_bat->capacity;
		            }
        }

         if ((cw_capacity == 0) && (cw_bat->capacity > 1)) {              // avoid battery level jump to 0% at a moment from more than 2%
                allow_change = ((new_run_time - cw_bat->run_time_capacity) / BATTERY_DOWN_MIN_CHANGE_RUN);
                allow_change += ((new_sleep_time - cw_bat->sleep_time_capacity) / BATTERY_DOWN_MIN_CHANGE_SLEEP);

                allow_capacity = cw_bat->capacity - allow_change;
                cw_capacity = (allow_capacity >= cw_capacity) ? allow_capacity: cw_capacity;
                reg_val[0] = MODE_NORMAL;
                ret = cw_write(cw_bat->client, REG_MODE, reg_val);
                  if (ret < 0)
                    return ret;
        }
        if ((cw_bat->charger_mode > 0) && (cw_capacity >= 95) && (cw_capacity <= cw_bat->capacity)) {     // avoid no charge full

                capacity_or_aconline_time = (cw_bat->sleep_time_capacity > cw_bat->sleep_time_ac_online) ? cw_bat->sleep_time_capacity : cw_bat->sleep_time_ac_online;
                capacity_or_aconline_time += (cw_bat->run_time_capacity > cw_bat->run_time_ac_online) ? cw_bat->run_time_capacity : cw_bat->run_time_ac_online;
                allow_change = (new_sleep_time + new_run_time - capacity_or_aconline_time) / BATTERY_UP_MAX_CHANGE;
                if (allow_change > 0) {
                        allow_capacity = cw_bat->capacity + allow_change; 
                        cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
                        jump_flag =1;
                }
                if (cw_capacity <= cw_bat->capacity)
                	   cw_capacity = cw_bat->capacity;            
        }  else if ((cw_bat->charger_mode == 0) && (cw_capacity <= cw_bat->capacity ) && (cw_capacity >= 90) && (jump_flag == 1)) {     // avoid battery level jump to CW_BAT
                capacity_or_aconline_time = (cw_bat->sleep_time_capacity > cw_bat->sleep_time_ac_online) ? cw_bat->sleep_time_capacity : cw_bat->sleep_time_ac_online;
                capacity_or_aconline_time += (cw_bat->run_time_capacity > cw_bat->run_time_ac_online) ? cw_bat->run_time_capacity : cw_bat->run_time_ac_online;
                allow_change = (new_sleep_time + new_run_time - capacity_or_aconline_time) / BATTERY_DOWN_CHANGE;
                if (allow_change > 0) {
                        allow_capacity = cw_bat->capacity - allow_change; 
                        if (cw_capacity >= allow_capacity){
                        	jump_flag =0;
                        }
                        else{
                        cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
                        }
                }
                else if (cw_capacity <= cw_bat->capacity)
                	   cw_capacity = cw_bat->capacity;
                
        } 
 
#if 1	
	if((cw_bat->charger_mode > 0) &&(cw_capacity == 0))
	{		  
                if (((new_sleep_time + new_run_time - cw_bat->sleep_time_ac_online - cw_bat->run_time_ac_online) > BATTERY_DOWN_MAX_CHANGE_RUN_AC_ONLINE) && (if_quickstart == 0)) {
        		            cw_quickstart(cw_bat);      // if the cw_capacity = 0 the cw2015 will qstrt
                        if_quickstart = 1;
                }
	} else if ((if_quickstart == 1)&&(cw_bat->charger_mode == 0)) {
                if_quickstart = 0;
        }
#endif
#if 0
        if (cw_bat->plat_data->chg_ok_pin != INVALID_GPIO) {
                if(gpio_get_value(cw_bat->plat_data->chg_ok_pin) != cw_bat->plat_data->chg_ok_level) {
                        if (cw_capacity == 100) {
                                cw_capacity = 99;
                        }
                } else {
                        if (cw_bat->charger_mode > 0) {
                                cw_capacity = 100;
                        }
                }
        }
#endif

#ifdef SYSTEM_SHUTDOWN_VOLTAGE
        if ((cw_bat->charger_mode == 0) && (cw_capacity <= 10) && (cw_bat->voltage <= SYSTEM_SHUTDOWN_VOLTAGE)){      	     
                if (if_quickstart == 10){       	      	
                        cw_quickstart(cw_bat);
                        if_quickstart = 12;
                        cw_capacity = 0;
                } else if (if_quickstart <= 10)
                        if_quickstart =if_quickstart+2;
                dev_info(&cw_bat->client->dev, "the cw201x voltage is less than SYSTEM_SHUTDOWN_VOLTAGE !!!!!!!, funciton: %s, line: %d\n", __func__, __LINE__);
        } else if ((cw_bat->charger_mode > 0)&& (if_quickstart <= 12)) {
                if_quickstart = 0;
        }
#endif
        return cw_capacity;
}

static int cw_get_vol(struct cw_battery *cw_bat)
{
        int ret;
        u8 reg_val[2];
        u16 value16, value16_1, value16_2, value16_3;
        int voltage;

        ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
        if (ret < 0)
                return ret;
                
        value16 = reg_val[0];
        value16 = (value16 <<8) +reg_val[1];
       
        ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
        if (ret < 0)
                return ret;

        value16_1 = reg_val[0];
        value16_1 = (value16_1 <<8) +reg_val[1];
       
        ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
        if (ret < 0)
                return ret;

        //value16_2 = reg_val;
        value16_2 = (reg_val[0] <<8) +reg_val[1];
              
        // printk("===========the cw201x voltage is voltage_2 = %d, funciton: %s, line: %d\n", value16_2,  __func__, __LINE__);
		
        if(value16 > value16_1)
	    {	 
	    	value16_3 = value16;
		    value16 = value16_1;
		    value16_1 = value16_3;
        }
		
        if(value16_1 > value16_2)
	    {
	    	value16_3 =value16_1;
			  value16_1 =value16_2;
			  value16_2 =value16_3;
	    }
			
        if(value16 >value16_1)
	    {	 
	    	value16_3 =value16;
			  value16 =value16_1;
			  value16_1 =value16_3;
        }			

        voltage = value16_1 * 312 / 1024;
        
        voltage = voltage * 1000;
        // printk("++++++++++++the cw201x voltage is voltage = %d, funciton: %s, line: %d\n", voltage,  __func__, __LINE__);

        return voltage;
}

static int cw_get_alt(struct cw_battery *cw_bat)
{
	      int ret = 0;
	      u8 reg_val;
	      u8 value8 = 0;
	      int alrt;

	      ret = cw_read(cw_bat->client, REG_RRT_ALERT, &reg_val);
          if (ret < 0)
             return ret;
	      value8 = reg_val;
	      alrt = value8 >>7;

	//dev_info(&client->dev, "read RRT %d%%. value16 0x%x\n", alrt, value16);
	      value8 = value8&0x7f;
	      reg_val = value8;
	      ret = cw_write(cw_bat->client, REG_RRT_ALERT, &reg_val);
        if(ret < 0) {
                printk("Error clear ALRT\n");
                return ret;
        }
		
	      return alrt;
}

static int cw_get_time_to_empty(struct cw_battery *cw_bat)
{
        int ret;
        u8 reg_val;
        u16 value16;

        ret = cw_read(cw_bat->client, REG_RRT_ALERT, &reg_val);
        if (ret < 0)
                return ret;

        value16 = reg_val;

        ret = cw_read(cw_bat->client, REG_RRT_ALERT + 1, &reg_val);
        if (ret < 0)
                return ret;

        value16 = ((value16 << 8) + reg_val) & 0x1fff;
        return value16;
}

static void rk_bat_update_capacity(struct cw_battery *cw_bat)
{
        int cw_capacity;

        cw_capacity = cw_get_capacity(cw_bat);
        if ((cw_capacity >= 0) && (cw_capacity <= 100) && (cw_bat->capacity != cw_capacity)) {
                cw_bat->capacity = cw_capacity;
                cw_bat->bat_change = 1;
                cw_update_time_member_capacity(cw_bat);

                if (cw_bat->capacity == 0)
                        printk("report battery capacity 0 and will shutdown if no changing");
				update_pmu_capacity(cw_capacity);
        }
}



static void rk_bat_update_vol(struct cw_battery *cw_bat)
{
        int ret;
        ret = cw_get_vol(cw_bat);
        if ((ret >= 0) && (cw_bat->voltage != ret)) {
                cw_bat->voltage = ret;
                cw_bat->bat_change = 1;
        }
}

static void rk_bat_update_status(struct cw_battery *cw_bat)
{
        int status;

        if ((cw_bat->dc_online == 1)||(cw_bat->usb_online == 1)) {  
                if (cw_bat->capacity >= 100) 
                        status=POWER_SUPPLY_STATUS_FULL;
                else
                        status=POWER_SUPPLY_STATUS_CHARGING;
        } else {
                status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        }

        if (cw_bat->status != status) {
                cw_bat->status = status;
                cw_bat->bat_change = 1;
       
        } 
}

static void rk_bat_update_time_to_empty(struct cw_battery *cw_bat)
{
        int ret;
        ret = cw_get_time_to_empty(cw_bat);
        if ((ret >= 0) && (cw_bat->time_to_empty != ret)) {
                cw_bat->time_to_empty = ret;
                cw_bat->bat_change = 1;
        }
        
}

static int rk_ac_update_online(struct cw_battery *cw_bat)
{
    if(cw_bat->plat_data->dc_det_pin == INVALID_GPIO)return 0;
    
        if (gpio_get_value(cw_bat->plat_data->dc_det_pin) == cw_bat->plat_data->dc_det_level) {
                if (cw_bat->dc_online != 1) {
                        cw_update_time_member_ac_online(cw_bat);
                        cw_bat->dc_online = 1;
                        if (cw_bat->charger_mode != AC_CHARGER_MODE)
                           cw_bat->charger_mode = AC_CHARGER_MODE;
                        return 1;
                }
        } else {
                if (cw_bat->dc_online != 0) {
                        cw_update_time_member_ac_online(cw_bat);
                        cw_bat->dc_online = 0;
                        if ((cw_bat->usb_online == 0)&&(cw_bat->charger_mode != 0))
                        	 cw_bat->charger_mode = 0;
                        return 1;
                }
        }
        return 0;
}

extern int get_gadget_connect_flag(void);
extern int dwc_vbus_status( void );

static int rk_usb_update_online(struct cw_battery *cw_bat)
{
#ifdef Enable_USB_CHG

    int usb_status = dwc_vbus_status();
    int gadget_status =get_gadget_connect_flag();
    xprintk("%s usb_status=[%d],cw_bat->charger_mode=[%d],cw_bat->gadget_status=[%d]\n",__func__,usb_status,cw_bat->charger_mode,gadget_status);
        if (usb_status == 0){
                   if (cw_bat->plat_data->chg_mode_sel_pin != INVALID_GPIO)
                         gpio_direction_output(cw_bat->plat_data->chg_mode_sel_pin, GPIO_LOW);
                   if ((cw_bat->dc_online == 0)&&(cw_bat->charger_mode != 0))
                         cw_bat->charger_mode = 0;
                   if (cw_bat->usb_online != 0) {
                       cw_update_time_member_ac_online(cw_bat);
                       cw_bat->usb_online = 0;
                       return 1;
                   }
        }     
        else if (usb_status == 1){
             if (0 == gadget_status){
                   if (cw_bat->charger_mode != AC_CHARGER_MODE)
                         cw_bat->charger_mode = AC_CHARGER_MODE;
                   if (cw_bat->plat_data->chg_mode_sel_pin != INVALID_GPIO)
                         gpio_direction_output(cw_bat->plat_data->chg_mode_sel_pin, GPIO_HIGH);
                   if (cw_bat->usb_online != 1){
        		            cw_bat->usb_online = 1;
                        cw_update_time_member_ac_online(cw_bat);
                        return 1; 
                   } 
             }
             else{
             	     if (cw_bat->charger_mode != USB_CHARGER_MODE)
                         cw_bat->charger_mode = USB_CHARGER_MODE;
                   if (cw_bat->plat_data->chg_mode_sel_pin != INVALID_GPIO)
                         gpio_direction_output(cw_bat->plat_data->chg_mode_sel_pin, GPIO_LOW);
                   if (cw_bat->usb_online != 1){
        		            cw_bat->usb_online = 1;
                        cw_update_time_member_ac_online(cw_bat);
                        return 1; 
                   } 
             }           	            
        }
        else if (usb_status == 2){

                	 if (cw_bat->charger_mode != AC_CHARGER_MODE)
                	     cw_bat->charger_mode = AC_CHARGER_MODE;
                	 if (cw_bat->plat_data->chg_mode_sel_pin != INVALID_GPIO)
                       gpio_direction_output(cw_bat->plat_data->chg_mode_sel_pin, GPIO_HIGH);
                   if (cw_bat->usb_online != 1){
        	     	       cw_bat->usb_online = 1;
                	     cw_update_time_member_ac_online(cw_bat);
                	     return 1;
                	 }
        }              
        return 0;
#else        
        cw_bat->usb_online = 0;
           return 0;
#endif
}

static void cw_bat_work(struct work_struct *work)
{
        struct delayed_work *delay_work;
        struct cw_battery *cw_bat;
        int ret;

        delay_work = container_of(work, struct delayed_work, work);
        cw_bat = container_of(delay_work, struct cw_battery, battery_delay_work);

        ret = rk_ac_update_online(cw_bat);
        if (ret == 1) {
                power_supply_changed(&cw_bat->rk_ac);
        }
        
#ifdef Enable_USB_CHG	
        ret = rk_usb_update_online(cw_bat);
        if (ret == 1) {
            power_supply_changed(&cw_bat->rk_usb);     
            power_supply_changed(&cw_bat->rk_ac);          /////////////////////////////////////////////////////
        }
#endif

        rk_bat_update_status(cw_bat);
        rk_bat_update_capacity(cw_bat);
        rk_bat_update_vol(cw_bat);
        rk_bat_update_time_to_empty(cw_bat);

        if (cw_bat->bat_change) {
                power_supply_changed(&cw_bat->rk_bat);
                cw_bat->bat_change = 0;
        }

        queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(1000));

        xprintk("cw_bat->bat_change = %d, cw_bat->time_to_empty = %d, cw_bat->capacity = %d, cw_bat->voltage = %d, cw_bat->dc_online = %d, cw_bat->usb_online = %d, cw_bat->status = %d\n",\
                        cw_bat->bat_change, cw_bat->time_to_empty, cw_bat->capacity, cw_bat->voltage, cw_bat->dc_online, cw_bat->usb_online, cw_bat->status);
}

static int rk_usb_get_property (struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
        int ret = 0;
        struct cw_battery *cw_bat;

        cw_bat = container_of(psy, struct cw_battery, rk_usb);
        switch (psp) {
        case POWER_SUPPLY_PROP_ONLINE:
                val->intval = (cw_bat->charger_mode == USB_CHARGER_MODE);   
                break;
        default:
                break;
        }
        return ret;
}

static enum power_supply_property rk_usb_properties[] = {
        POWER_SUPPLY_PROP_ONLINE,
};


static int rk_ac_get_property (struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
        int ret = 0;
        struct cw_battery *cw_bat;

        cw_bat = container_of(psy, struct cw_battery, rk_ac);
        switch (psp) {
        case POWER_SUPPLY_PROP_ONLINE:
                val->intval = (cw_bat->charger_mode == AC_CHARGER_MODE);
                break;
        default:
                break;
        }
        return ret;
}

static enum power_supply_property rk_ac_properties[] = {
        POWER_SUPPLY_PROP_ONLINE,
};

static int rk_battery_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
        int ret = 0;
        struct cw_battery *cw_bat;

        cw_bat = container_of(psy, struct cw_battery, rk_bat); 
        switch (psp) {
        case POWER_SUPPLY_PROP_CAPACITY:
                val->intval = cw_bat->capacity;
                break;
        case POWER_SUPPLY_PROP_STATUS:
                val->intval = cw_bat->status;
                break;
                
        case POWER_SUPPLY_PROP_HEALTH:
                val->intval= POWER_SUPPLY_HEALTH_GOOD;
                break;
        case POWER_SUPPLY_PROP_PRESENT:
                val->intval = cw_bat->voltage <= 0 ? 0 : 1;
                break;
                
        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
                val->intval = cw_bat->voltage;
                break;
                
        case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
                val->intval = cw_bat->time_to_empty;			
                break;
            
        case POWER_SUPPLY_PROP_TECHNOLOGY:
                val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
                break;

        default:
                break;
        }
        return ret;
}

static enum power_supply_property rk_battery_properties[] = {
        POWER_SUPPLY_PROP_CAPACITY,
        POWER_SUPPLY_PROP_STATUS,
        POWER_SUPPLY_PROP_HEALTH,
        POWER_SUPPLY_PROP_PRESENT,
        POWER_SUPPLY_PROP_VOLTAGE_NOW,
        POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
        POWER_SUPPLY_PROP_TECHNOLOGY,
};

static int cw_bat_gpio_init(struct cw_battery *cw_bat)
{

        int ret;
        gpio_free(cw_bat->plat_data->dc_det_pin);
        if (cw_bat->plat_data->dc_det_pin != INVALID_GPIO) {
                ret = gpio_request(cw_bat->plat_data->dc_det_pin, NULL);
                if (ret) {
                        printk("failed to request dc_det_pin gpio\n");
                        goto request_dc_det_pin_fail;
                }

                gpio_pull_updown(cw_bat->plat_data->dc_det_pin, GPIOPullUp);
                ret = gpio_direction_input(cw_bat->plat_data->dc_det_pin);
                if (ret) {
                        printk("failed to set dc_det_pin input\n");
                        goto request_bat_low_pin_fail;
                }
        }
        if (cw_bat->plat_data->bat_low_pin != INVALID_GPIO) {
                ret = gpio_request(cw_bat->plat_data->bat_low_pin, NULL);
                if (ret) {
                        printk("failed to request bat_low_pin gpio\n");
                        goto request_bat_low_pin_fail;
                }

                gpio_pull_updown(cw_bat->plat_data->bat_low_pin, GPIOPullUp);
                ret = gpio_direction_input(cw_bat->plat_data->bat_low_pin);
                if (ret) {
                        printk("failed to set bat_low_pin input\n");
                        goto request_chg_ok_pin_fail;
                }
        }
        if (cw_bat->plat_data->chg_ok_pin != INVALID_GPIO) {
                ret = gpio_request(cw_bat->plat_data->chg_ok_pin, NULL);
                if (ret) {
                        printk("failed to request chg_ok_pin gpio\n");
                        goto request_chg_ok_pin_fail;
                }

                gpio_pull_updown(cw_bat->plat_data->chg_ok_pin, GPIOPullUp);
                ret = gpio_direction_input(cw_bat->plat_data->chg_ok_pin);
                if (ret) {
                        printk("failed to set chg_ok_pin input\n");
                        gpio_free(cw_bat->plat_data->chg_ok_pin); 
                        goto request_chg_ok_pin_fail;
                }
        }
        if (cw_bat->plat_data->chg_mode_sel_pin!= INVALID_GPIO) {
                ret = gpio_request(cw_bat->plat_data->chg_mode_sel_pin, NULL);
                if (ret) {
                        printk("failed to request chg_mode_sel_pin gpio\n");
                        goto request_chg_ok_pin_fail;
                }
                ret = gpio_direction_output(cw_bat->plat_data->chg_mode_sel_pin, (cw_bat->plat_data->chg_mode_sel_level==GPIO_HIGH) ? GPIO_LOW : GPIO_HIGH);
                if (ret) {
                        printk("failed to set chg_mode_sel_pin input\n");
                        gpio_free(cw_bat->plat_data->chg_mode_sel_pin); 
                        goto request_chg_ok_pin_fail;
                }
        }
        return 0;

        
request_chg_ok_pin_fail:
        if (cw_bat->plat_data->bat_low_pin != INVALID_GPIO)
                gpio_free(cw_bat->plat_data->bat_low_pin);

request_bat_low_pin_fail:
        if (cw_bat->plat_data->dc_det_pin != INVALID_GPIO) 
                gpio_free(cw_bat->plat_data->dc_det_pin);

request_dc_det_pin_fail:
        return ret;

}


static void dc_detect_do_wakeup(struct work_struct *work)
{
        int ret;
        int irq;
        unsigned int type;

        struct delayed_work *delay_work;
        struct cw_battery *cw_bat;

        delay_work = container_of(work, struct delayed_work, work);
        cw_bat = container_of(delay_work, struct cw_battery, dc_wakeup_work);

        rk28_send_wakeup_key();
        //if(cw_bat->charger_init_mode)cw_bat->charger_init_mode=0;

        irq = gpio_to_irq(cw_bat->plat_data->dc_det_pin);
        type = gpio_get_value(cw_bat->plat_data->dc_det_pin) ? IRQ_TYPE_EDGE_FALLING : IRQ_TYPE_EDGE_RISING;
        ret = irq_set_irq_type(irq, type);
        if (ret < 0) {
                pr_err("%s: irq_set_irq_type(%d, %d) failed\n", __func__, irq, type);
        }
        enable_irq(irq);
}

static irqreturn_t dc_detect_irq_handler(int irq, void *dev_id)
{
        struct cw_battery *cw_bat = dev_id;
        disable_irq_nosync(irq); // for irq debounce
        //wake_lock_timeout(&usb_wakelock, WAKE_LOCK_TIMEOUT);
        //schedule_delayed_work(&wakeup_work, HZ / 10);
        queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->dc_wakeup_work, msecs_to_jiffies(20));
        return IRQ_HANDLED;
}

#ifdef BAT_LOW_INTERRUPT

#define WAKE_LOCK_TIMEOUT       (10 * HZ)
static struct wake_lock bat_low_wakelock;

static void bat_low_detect_do_wakeup(struct work_struct *work)
{
        struct delayed_work *delay_work;
        struct cw_battery *cw_bat;

        delay_work = container_of(work, struct delayed_work, work);
        cw_bat = container_of(delay_work, struct cw_battery, bat_low_wakeup_work);
        dev_info(&cw_bat->client->dev, "func: %s-------\n", __func__);
        cw_get_alt(cw_bat);
        //enable_irq(irq);
}

static irqreturn_t bat_low_detect_irq_handler(int irq, void *dev_id)
{
        struct cw_battery *cw_bat = dev_id;
        // disable_irq_nosync(irq); // for irq debounce
        wake_lock_timeout(&bat_low_wakelock, WAKE_LOCK_TIMEOUT);
        queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->bat_low_wakeup_work, msecs_to_jiffies(20));
        return IRQ_HANDLED;
}
#endif

static int cw_bat_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
        struct cw_battery *cw_bat;
        int ret;
        int irq;
        int irq_flags;
        int loop = 0;

        cw_bat = devm_kzalloc(&client->dev, sizeof(*cw_bat), GFP_KERNEL);
        if (!cw_bat) {
                printk("fail to allocate memory\n");
                return -ENOMEM;
        }

        i2c_set_clientdata(client, cw_bat);
        cw_bat->plat_data = client->dev.platform_data;
        ret = cw_bat_gpio_init(cw_bat);
        if (ret) {
                printk("cw_bat_gpio_init error\n");
                return ret;
        }
        
        cw_bat->client = client;
        do {
			ret = cw_init(cw_bat);
		} while ((loop++ < 100) && ret);
        if (ret) 
                return ret;

        cw_bat->rk_bat.name = "cw2015-bat";
        cw_bat->rk_bat.type = POWER_SUPPLY_TYPE_BATTERY;
        cw_bat->rk_bat.properties = rk_battery_properties;
        cw_bat->rk_bat.num_properties = ARRAY_SIZE(rk_battery_properties);
        cw_bat->rk_bat.get_property = rk_battery_get_property;
        ret = power_supply_register(&client->dev, &cw_bat->rk_bat);
        if(ret < 0) {
                printk("power supply register rk_bat error\n");
                goto rk_bat_register_fail;
        }

        cw_bat->rk_ac.name = "cw2015-ac";
        cw_bat->rk_ac.type = POWER_SUPPLY_TYPE_MAINS;
        cw_bat->rk_ac.properties = rk_ac_properties;
        cw_bat->rk_ac.num_properties = ARRAY_SIZE(rk_ac_properties);
        cw_bat->rk_ac.get_property = rk_ac_get_property;
        ret = power_supply_register(&client->dev, &cw_bat->rk_ac);
        if(ret < 0) {
                printk("power supply register rk_ac error\n");
                goto rk_ac_register_fail;
        }

        cw_bat->rk_usb.name = "cw2015-usb";
        cw_bat->rk_usb.type = POWER_SUPPLY_TYPE_USB;
        cw_bat->rk_usb.properties = rk_usb_properties;
        cw_bat->rk_usb.num_properties = ARRAY_SIZE(rk_usb_properties);
        cw_bat->rk_usb.get_property = rk_usb_get_property;
        ret = power_supply_register(&client->dev, &cw_bat->rk_usb);
        if(ret < 0) {
                printk("power supply register rk_ac error\n");
                goto rk_usb_register_fail;
        }

        //cw_bat->charger_init_mode = dwc_otg_check_dpdm();
        cw_bat->dc_online = 0;
        cw_bat->usb_online = 0;
        cw_bat->charger_mode = 0;
        cw_bat->capacity = 2;
        cw_bat->voltage = 0;
        cw_bat->status = 0;
        cw_bat->time_to_empty = 0;
        cw_bat->bat_change = 0;

        cw_update_time_member_capacity(cw_bat);
        cw_update_time_member_ac_online(cw_bat);

        cw_bat->battery_workqueue = create_singlethread_workqueue("rk_battery");
        INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);
        INIT_DELAYED_WORK(&cw_bat->dc_wakeup_work, dc_detect_do_wakeup);
        queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(10));
        
        irq = gpio_to_irq(cw_bat->plat_data->dc_det_pin);
        irq_flags = gpio_get_value(cw_bat->plat_data->dc_det_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
        ret = request_irq(irq, dc_detect_irq_handler, irq_flags, "usb_detect", cw_bat);
        if (ret < 0) {
                pr_err("%s: request_irq(%d) failed\n", __func__, irq);
        }
        enable_irq_wake(irq);

#ifdef BAT_LOW_INTERRUPT
        INIT_DELAYED_WORK(&cw_bat->bat_low_wakeup_work, bat_low_detect_do_wakeup);
        wake_lock_init(&bat_low_wakelock, WAKE_LOCK_SUSPEND, "bat_low_detect");
        if (cw_bat->plat_data->bat_low_pin != INVALID_GPIO) {
                irq = gpio_to_irq(cw_bat->plat_data->bat_low_pin);
                ret = request_irq(irq, bat_low_detect_irq_handler, IRQF_TRIGGER_RISING, "bat_low_detect", cw_bat);
                if (ret < 0) {
                        gpio_free(cw_bat->plat_data->bat_low_pin);
                }
                enable_irq_wake(irq);
        }
#endif

        printk("cw2015 driver v1.0 probe sucess\n");
        return 0;

rk_usb_register_fail:
        power_supply_unregister(&cw_bat->rk_bat);
rk_ac_register_fail:
        power_supply_unregister(&cw_bat->rk_ac);
rk_bat_register_fail:
        printk("cw2015 driver v1.0 probe error!!!!\n");
        return ret;
}

static int cw_bat_remove(struct i2c_client *client)
{
        struct cw_battery *cw_bat = i2c_get_clientdata(client);
        xprintk();
        cancel_delayed_work(&cw_bat->battery_delay_work);
        return 0;
}

#ifdef CONFIG_PM
static int cw_bat_suspend(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct cw_battery *cw_bat = i2c_get_clientdata(client);
        xprintk();
        cancel_delayed_work(&cw_bat->battery_delay_work);
        return 0;
}

static int cw_bat_resume(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct cw_battery *cw_bat = i2c_get_clientdata(client);
        xprintk();
        queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(100));
        return 0;
}

static const struct i2c_device_id cw_id[] = {
	{ "cw201x", 0 },
};
MODULE_DEVICE_TABLE(i2c, cw_id);

static const struct dev_pm_ops cw_bat_pm_ops = {
        .suspend  = cw_bat_suspend,
        .resume   = cw_bat_resume,
};
#endif

static struct i2c_driver cw_bat_driver = {
        .driver         = {
                .name   = "cw201x",
#ifdef CONFIG_PM
                .pm     = &cw_bat_pm_ops,
#endif
        },
        
        .probe          = cw_bat_probe,
        .remove         = cw_bat_remove,
	.id_table	= cw_id,
};

static struct platform_device adc_battery_device;

static int __init cw_bat_init(void)
{
	if (pmic_is_axp228()) {
		return 0;
	}

       xprintk();
       if (add_device() < 0) {
               adc_battery_device.name = "adc-battery";
               platform_device_register(&adc_battery_device);
               return 0;
       }

       return i2c_add_driver(&cw_bat_driver);
}

static void __exit cw_bat_exit(void)
{
        xprintk();
        i2c_del_driver(&cw_bat_driver);
}

fs_initcall(cw_bat_init);
module_exit(cw_bat_exit);

MODULE_AUTHOR("xhc<xhc@rock-chips.com>");
MODULE_DESCRIPTION("cw2015 battery driver");
MODULE_LICENSE("GPL");

