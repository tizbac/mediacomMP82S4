/*
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
 *
 *	note: only support mulititouch	Wenfs 2010-10-01
 *  for this touchscreen to work, it's slave addr must be set to 0x7e | 0x70
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include "ft5x_ts.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/pm.h>
#include <linux/earlysuspend.h>
#endif

#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <mach/gpio.h>
#include <mach/yfmach.h>
#include <mach/irqs.h>

#include <asm/uaccess.h>

/* YF */
struct goodix_platform_data {
	int model ;
	int rest_pin;
	int irq_pin ;
	int (*get_pendown_state)(void);
	int (*init_platform_hw)(void);
	int (*platform_sleep)(void);
	int (*platform_wakeup)(void);
	void (*exit_platform_hw)(void);
};

extern struct goodix_platform_data goodix_info;
#define	TS_INT			(gpio_to_irq(goodix_info.irq_pin))

#define FOR_TSLIB_TEST
#define SCREEN_MAX_X    screen_max_x
#define SCREEN_MAX_Y    screen_max_y
#define TP_NUMBER_MAX   5
#define KEY_NUMBER_MAX  5
#define PRESS_MAX       255

//#define PRINT_INT_INFO
#ifdef PRINT_INT_INFO
#define dbg_int printk
#else
#define dbg_int(...)
#endif

//#define PRINT_POINT_INFO
#ifdef PRINT_POINT_INFO
#define dbg_pnt printk
#else
#define dbg_pnt(...)
#endif

static struct i2c_client *this_client;

static int screen_max_x = 1024;
static int screen_max_y = 768;
static int revert_x_flag = 0;
static int revert_y_flag = 0;
static int exchange_x_y_flag = 0;
static int scale_x = 0;
static int offset_x = 0;
static int scale_y = 0;
static int offset_y = 0;
static int tp_number = 5;
static int x_store = 0;
static int y_store = 0;

static int key_number = 0;
static int key_x_min = 0;
static int key_x_max = 0;
static int key_id = 0; //current report key id
static int key_val; //current report key value
static int tp_id = 0;
static char fw_status[50];
static char ctpfwinfo=0;
static const char *pft5x_ts_fw;


struct ts_key_map {
	int y_min;
	int y_max;
	int key;
} ts_keys[KEY_NUMBER_MAX];

struct ft5x_ts_data {
	struct input_dev * input_dev;
	struct work_struct pen_event_work;
	struct workqueue_struct * ts_workqueue;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

static int ft5x_get_device_id(void);

static int ft5x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
			.scl_rate = 200*1000,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: 0x%x, %d\n", __func__, this_client->addr, ret);

	return ret;
}

static int ft5x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
			.scl_rate = 200*1000,
		},
	};

   	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int ft5x_set_reg(u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = addr;
    buf[1] = para;
    ret = ft5x_i2c_txdata(buf, 2);
    if (ret < 0) {
        pr_err("write reg failed! %#x ret: %d", addr, ret);
        return -1;
    }

    return 0;
}

//********************************************************************************//
//****************************download FW*****************************************//
#define    FTS_PACKET_LENGTH        128

/*
#define FW_NAME_RUISENSI "ft5406_ruisensi_app.i"
#define FW_NAME_RUISHI	 "ft5406_ruishi_app.i"


static const u8 FW_DATA_RUISENSI[]={
#include  FW_NAME_RUISENSI
};
static const u8 FW_DATA_RUISHI[]={
#include  FW_NAME_RUISHI
};
*/
struct CTP_FW_LIST
{
	char deviceid;
	char * pFW_NAME;
	u8   * pFW_DATA;
	u32  FW_LEN;
}ctp_fw_list[]=
{
	//{0x79,FW_NAME_RUISENSI, FW_DATA_RUISENSI, sizeof(FW_DATA_RUISENSI)},
	//{0x2d,FW_NAME_RUISHI,   FW_DATA_RUISHI,   sizeof(FW_DATA_RUISHI)},

	{0x00,NULL,NULL,0}
};

static u8 * CTPM_FW=NULL;
static u32  CTPM_FW_LEN=0;

typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;


/***********************************************************************
    [function]:
		           callback:               write data to ctpm by i2c interface;
    [parameters]:
			    buffer[in]:             data buffer;
			    length[in]:            the length of the data buffer;
    [return]:
			    FTS_TRUE:            success;
			    FTS_FALSE:           fail;
************************************************************************/
static bool  i2c_write_interface(u8* pbt_buf, int dw_lenth)
{
    int ret;
    ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return false;
    }

    return true;
}


/***********************************************************************
    [function]:
		           callback:                read register value ftom ctpm by i2c interface;
    [parameters]:
                        reg_name[in]:         the register which you want to write;
			    tx_buf[in]:              buffer which is contained of the writing value;
    [return]:
			    FTS_TRUE:              success;
			    FTS_FALSE:             fail;
************************************************************************/
static bool fts_register_write(u8 reg_name, u8* tx_buf)
{
	u8 write_cmd[2] = {0};

	write_cmd[0] = reg_name;
	write_cmd[1] = *tx_buf;

	/*call the write callback function*/
	return i2c_write_interface(write_cmd, 2);
}

/***********************************************************************
[function]:
                      callback:         send a command to ctpm.
[parameters]:
			  btcmd[in]:       command code;
			  btPara1[in]:     parameter 1;
			  btPara2[in]:     parameter 2;
			  btPara3[in]:     parameter 3;
                      num[in]:         the valid input parameter numbers,
                                           if only command code needed and no
                                           parameters followed,then the num is 1;
[return]:
			  FTS_TRUE:      success;
			  FTS_FALSE:     io fail;
************************************************************************/
static bool cmd_write(u8 btcmd,u8 btPara1,u8 btPara2,u8 btPara3,u8 num)
{
    u8 write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(write_cmd, num);
}

/***********************************************************************
    [function]:
		           callback:              read data from ctpm by i2c interface;
    [parameters]:
			    buffer[in]:            data buffer;
			    length[in]:           the length of the data buffer;
    [return]:
			    FTS_TRUE:            success;
			    FTS_FALSE:           fail;
************************************************************************/
static bool i2c_read_interface(u8* pbt_buf, int dw_lenth)
{
    int ret;

    ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        printk("[TSP]i2c_read_interface error\n");
        return false;
    }

    return true;
}


/***********************************************************************
[function]:
                      callback:         read a byte data  from ctpm;
[parameters]:
			  buffer[in]:       read buffer;
			  length[in]:      the size of read data;
[return]:
			  FTS_TRUE:      success;
			  FTS_FALSE:     io fail;
************************************************************************/
static bool byte_read(u8* buffer, int length)
{
    return i2c_read_interface(buffer, length);
}

/***********************************************************************
[function]:
                      callback:         write a byte data  to ctpm;
[parameters]:
			  buffer[in]:       write buffer;
			  length[in]:      the size of write data;
[return]:
			  FTS_TRUE:      success;
			  FTS_FALSE:     io fail;
************************************************************************/
static bool byte_write(u8* buffer, int length)
{

    return i2c_write_interface(buffer, length);
}

/***********************************************************************
    [function]:
		           callback:                 read register value ftom ctpm by i2c interface;
    [parameters]:
                        reg_name[in]:         the register which you want to read;
			    rx_buf[in]:              data buffer which is used to store register value;
			    rx_length[in]:          the length of the data buffer;
    [return]:
			    FTS_TRUE:              success;
			    FTS_FALSE:             fail;
************************************************************************/
static bool fts_register_read(u8 reg_name, u8* rx_buf, int rx_length)
{
	u8 read_cmd[2]= {0};
	u8 cmd_len 	= 0;

	read_cmd[0] = reg_name;
	cmd_len = 1;

	/*send register addr*/
	if(!i2c_write_interface(&read_cmd[0], cmd_len))
	{
		return false;
	}

	/*call the read callback function to get the register value*/
	if(!i2c_read_interface(rx_buf, rx_length))
	{
		return false;
	}
	return true;
}


/***********************************************************************
[function]:
                        callback:          burn the FW to ctpm.
[parameters]:
			    pbt_buf[in]:     point to Head+FW ;
			    dw_lenth[in]:   the length of the FW + 6(the Head length);
[return]:
			    ERR_OK:          no error;
			    ERR_MODE:      fail to switch to UPDATE mode;
			    ERR_READID:   read id fail;
			    ERR_ERASE:     erase chip fail;
			    ERR_STATUS:   status error;
			    ERR_ECC:        ecc error.
************************************************************************/
E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(u8* pbt_buf, int dw_lenth)
{
    u8  cmd,reg_val[2] = {0};
	u8  buffer[2] = {0};
    u8  packet_buf[FTS_PACKET_LENGTH + 6];
    u8  auc_i2c_write_buf[10];
    u8  bt_ecc;
    u8  k=0,time=5;

    int  j,temp,lenght,i_ret,packet_number, i = 0;
    int  i_is_new_protocol = 0;

STEP1:
    /******write 0xaa to register 0xfc******/
    cmd=0xaa;
    fts_register_write(0xfc,&cmd);
    mdelay(50);

     /******write 0x55 to register 0xfc******/
    cmd=0x55;
    fts_register_write(0xfc,&cmd);
    printk("[TSP] Step 1: Reset CTPM test\n");

    mdelay(30);


    /*******Step 2:Enter upgrade mode ****/
    printk("\n[TSP] Step 2:enter new update mode\n");
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        k ++;
        i_ret = ft5x_i2c_txdata(auc_i2c_write_buf, 2);
        mdelay(time);
        printk("\n[TSP] Step 2:enter new update mode\n");
    }while(i_ret < 0 && i < 10 );

    if (i > 1)
    {
        i_is_new_protocol = 1;
    }
    /********Step 3:check READ-ID********/
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    if (reg_val[0] == 0x79 && (reg_val[1] == 0x3 || reg_val[1] == 0x7))
    {
        printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {	
		if(k<10){
			mdelay(10);
			goto STEP1;
		}
		if(time==5){
			time=10;
			k=0;
			i=0;
			goto STEP1;			
		}else if(time==10){
			time=15;
			k=0;
			i=0;
			goto STEP1;
		}else if(time==15){
			time=20;
			k=0;
			i=0;
			goto STEP1;
		}else if(time==20){
			time=30;
			k=0;
			i=0;
			goto STEP1;
		}
		strcpy(fw_status,"**error: read id error**\n");
        // return ERR_READID;
        //i_is_new_protocol = 1;
    }

     /*********Step 4:erase app**********/
    if (i_is_new_protocol)
    {
        cmd_write(0x61,0x00,0x00,0x00,1);
    }
    else
    {
        cmd_write(0x60,0x00,0x00,0x00,1);
    }
    mdelay(1500);
    printk("[TSP] Step 4: erase. \n");


    /*Step 5:write firmware(FW) to ctpm flash*/
    bt_ecc = 0;
    printk("[TSP] Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8)(temp>>8);
        packet_buf[3] = (u8)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8)(lenght>>8);
        packet_buf[5] = (u8)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        mdelay(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }
    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8)(temp>>8);
        packet_buf[3] = (u8)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8)(temp>>8);
        packet_buf[5] = (u8)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);
        mdelay(20);
    }
    /***********send the last six byte**********/
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (u8)(temp>>8);
        packet_buf[3] = (u8)temp;
        temp =1;
        packet_buf[4] = (u8)(temp>>8);
        packet_buf[5] = (u8)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i];
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);
        mdelay(20);
    }

    /********send the opration head************/
    cmd_write(0xcc,0x00,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
		 strcpy(fw_status,"***error: ctp ecc error***\n");
        return ERR_ECC;
    }
    /*******Step 7: reset the new FW**********/
    cmd_write(0x07,0x00,0x00,0x00,1);
    mdelay(300);//100ms
    fts_register_read(0xfc, buffer, 1);
    if (buffer[0] == 1)
    {
		cmd=4;
		fts_register_write(0xfc, &cmd);
		mdelay(2500);//2500ms
	 	do
	 	{
			fts_register_read(0xfc, buffer, 1);
			mdelay(100);//100ms
	 	}while (buffer[0] != 1);
    }
    return ERR_OK;
}


int fts_ctpm_auto_clb(void)
{
	#define FTS_FACTORYMODE_VALUE		0x40
	#define FTS_WORKMODE_VALUE		0x00
	unsigned char uc_temp = 0x00;
	unsigned char i = 0;

	/*start auto CLB */
	msleep(200);

	ft5x_set_reg(0, FTS_FACTORYMODE_VALUE);
	/*make sure already enter factory mode */
	msleep(100);
	/*write command to start calibration */
	ft5x_set_reg(2, 0x4);
	msleep(300);
	for (i = 0; i < 100; i++) {
		fts_register_read(0, &uc_temp,1);
		/*return to normal mode, calibration finish */
		if (0x0 == ((uc_temp & 0x70) >> 4))
			break;
	}

    /*calibration OK */
	msleep(300);
	ft5x_set_reg(0, FTS_FACTORYMODE_VALUE);	/*goto factory mode for store */
	msleep(100);	/*make sure already enter factory mode */
	ft5x_set_reg(2, 0x5);	/*store CLB result */
	msleep(300);
	ft5x_set_reg(0, FTS_WORKMODE_VALUE);	/*return to normal mode */
	msleep(300);

	/*store CLB result OK */
	return 0;
}


/***********************************************************************/

int fts_ctpm_fw_upgrade_with_i_file(void)
{
   u8*     pbt_buf = 0;
   int i_ret;

   pbt_buf = CTPM_FW;
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,CTPM_FW_LEN);
   fts_ctpm_auto_clb();
   return i_ret;
}

/***********************************************************************/

unsigned char fts_ctpm_get_upg_ver(void)
{
    unsigned int ui_sz;

    ui_sz = CTPM_FW_LEN;
    if (ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
        return 0xff;

}

unsigned char fts_ctpm_get_upg_id(u8 *pFW, u32 fw_len)
{
    unsigned int ui_sz;

    ui_sz = fw_len;
    if (ui_sz > 2)
    {
        return pFW[ui_sz - 1];
    }
    else
        return 0xff;

}

static int fts_download_fw(bool force)
{
	 #define FT5X0X_REG_FIRMID         0xA6         //firmware version /
	 #define FT5X0X_REG_REPORT_RATE    0x88         //report rate, in unit of 10Hz
	 #define FT5X0X_REG_THRES          0x80         //Thresshold, the threshold be low, the sensitivy will be high
	 #define FT5X0X_REG_NOISE_MODE     0xb2         //to enable or disable power noise, 1 -- enable, 0 -- disable
	 unsigned char reg_version,reg_value;
	 int err = 0;

	 if((CTPM_FW==NULL) || (CTPM_FW_LEN==0))
	 {
		strcpy(fw_status,"***err: can't find fw***\n");
	 	return -1;
	 }

	 if(fts_ctpm_get_upg_id(CTPM_FW,CTPM_FW_LEN)!=ft5x_get_device_id() && !force)
	 {
		 strcpy(fw_status,"***err: Firmware and the ctp does not match***\n");
		 printk("Firmware and the ctp does not match\n");
		 return -1;
	 }

	 msleep(100);  //wait CTP to bootup normally
	 fts_register_read(FT5X0X_REG_FIRMID, &reg_version,1);
	 printk("[TSP] firmware version = 0x%02x, app.i version=0x%02x\n", reg_version,fts_ctpm_get_upg_ver());
	 fts_register_read(FT5X0X_REG_REPORT_RATE, &reg_value,1);
	 printk("[TSP]firmware report rate = %dHz\n", reg_value*10);
	 fts_register_read(FT5X0X_REG_THRES, &reg_value,1);
	 printk("[TSP]firmware threshold = %d\n", reg_value * 4);
	 fts_register_read(FT5X0X_REG_NOISE_MODE, &reg_value,1);
	 printk("[TSP]nosie mode = 0x%02x\n", reg_value);
     if ((fts_ctpm_get_upg_ver() > reg_version) || force)
	  {
		  printk("[TSP] start upgrade new verison 0x%02x\n", fts_ctpm_get_upg_ver());
		  msleep(200);
		  err =  fts_ctpm_fw_upgrade_with_i_file();
		  if (err == 0)
		  {
		  	  printk("[TSP] ugrade successfuly.\n");
			  msleep(300);
			  fts_register_read(FT5X0X_REG_FIRMID, &reg_value,1);
			  printk("FTS_DBG from old version 0x%2x to new version = 0x%2x\n", reg_version, reg_value);
			  msleep(2000);
			  strcpy(fw_status,"***OK: ft5x_ts FW ugrade successfuly***\n");
		  }
		  else
		  {
			  printk("[TSP]  ugrade fail err=%d, line = %d.\n",err, __LINE__);

		  }
	  }else
	  {
		   strcpy(fw_status,"***err: fw version is already latest***\n");
	  }
	  return err;
}
//**********************************end download fw***********************************//
//************************************************************************************//

static void ft5x_report_value(struct input_dev * dev, int x, int y, int id)
{
	int mask = 1 << id;
	dbg_pnt("report %d, (%d %d)\n", id, x, y);
	tp_id |= mask;
	if(key_number > 0) {
		if(key_id == 0) {
			if(x >= key_x_min && x <= key_x_max) {
				int number = key_number;
				int * vals = &ts_keys->y_min;
				while(number--) {
					if(y >= vals[0] && y <= vals[1]) {
						key_id = mask;
						key_val = vals[2];
						dbg_pnt("report key down %d\n", key_val);
						input_report_key(dev, key_val, 1);
						break;
					}
					vals += 3;
				}
			}
		}
		if(key_id == mask) return;
	}
	if(tp_number > 1) {
		input_report_key(dev, BTN_TOUCH, 1);
		input_report_abs(dev, ABS_MT_TRACKING_ID, id);
		input_report_abs(dev, ABS_MT_TOUCH_MAJOR, 10);	// yftech
		input_report_abs(dev, ABS_MT_POSITION_X, x);
		input_report_abs(dev, ABS_MT_POSITION_Y, y);
		input_mt_sync(dev);
	}
	else {
		input_report_abs(dev, ABS_X, x);
		input_report_abs(dev, ABS_Y, y);
		input_report_abs(dev, ABS_PRESSURE, PRESS_MAX);
		input_report_key(dev, BTN_TOUCH, 1);
	}
}
static int ft5x_read_data(void)
{
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
	unsigned char buf[TP_NUMBER_MAX * 6 + 1];
	struct input_dev * dev = data->input_dev;
	int ret = -1, id = tp_id;
	int x, y, n;

	buf[0] = 2;
	ret = ft5x_i2c_rxdata(buf, 1);
	if (ret < 0) {
		printk("%s i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
	n = buf[0] & 7;
	if(n) {
		buf[0] = 2;
		if(n > tp_number) n = tp_number;
		ret = ft5x_i2c_rxdata(buf, n*6 + 1);
		if (ret < 0) {
			printk("%s i2c_rxdata failed: %d\n", __func__, ret);
			return ret;
		}
		if(n > (buf[0] & 7)) n = buf[0] & 7;
	}
	dbg_pnt("report point %d\n", n);
	n *= 6;
	tp_id = 0;

	while(n) {
		n -= 6;
		x = (buf[n+1] & 0x0F) << 8 | buf[n+2];
		y = (buf[n+3] & 0x0F) << 8 | buf[n+4];
		if(exchange_x_y_flag) swap(x, y);
		if(revert_x_flag) x = SCREEN_MAX_X - 1 - x;
		if(revert_y_flag) y = SCREEN_MAX_Y - 1 - y;
		if(scale_x) {
			x = (x * scale_x + offset_x) >> 16;
		}
		if(scale_y) {
			y = (y * scale_y + offset_y) >> 16;
		}
		ft5x_report_value(dev, x, y, buf[n+3]>>4);
		if (n==0)
		{
			x_store = x;
			y_store = y;
		}
	}
	if(key_id) {
		if(!(tp_id & key_id)) {
			//key release
			dbg_pnt("report key up %d\n", key_val);
			input_report_key(dev, key_val, 0);
			key_id = 0;
		}
		tp_id &= ~key_id;
	}
	if(id && !tp_id) {
		//touch release
		if(tp_number > 1) {
			input_report_abs(dev, ABS_MT_TRACKING_ID, 0);
			input_report_abs(dev, ABS_MT_POSITION_X, x_store);
			input_report_abs(dev, ABS_MT_POSITION_Y, y_store);
			input_report_key(dev, BTN_TOUCH, 0);
			input_report_abs(dev, ABS_MT_TOUCH_MAJOR, 0);
		}
		else {
			input_report_abs(dev, ABS_PRESSURE, 0);
			input_report_key(dev, BTN_TOUCH, 0);
		}
	}
	input_sync(dev);
    return 0;
}

static void ft5x_ts_pen_irq_work(struct work_struct *work)
{
	ft5x_read_data();
}

static irqreturn_t ft5x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x_ts_data *ft5x_ts = dev_id;
	if (!work_pending(&ft5x_ts->pen_event_work)) {
		dbg_int("Enter work\n");
		queue_work(ft5x_ts->ts_workqueue, &ft5x_ts->pen_event_work);
	}

	return IRQ_HANDLED;
}

static void ft5x_ts_wakeup(void)
{
}

static void ft5x_ts_reset(void)
{
	gpio_direction_output(goodix_info.rest_pin, 1);
	msleep(15);
	gpio_direction_output(goodix_info.rest_pin, 0);
	msleep(50);
	gpio_direction_output(goodix_info.rest_pin, 1);
	msleep(15);

}

#ifdef CONFIG_HAS_EARLYSUSPEND

static void ft5x_ts_suspend(struct early_suspend *handler)
{
	struct ft5x_ts_data *ft5x_ts= container_of(handler, struct ft5x_ts_data, early_suspend);

	if(ctpfwinfo != 2)//is upgrading
	{
		printk("ft5x_ts_suspend\n");
		disable_irq(TS_INT);
		cancel_work_sync(&ft5x_ts->pen_event_work);
		//ft5x_set_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
  		msleep(5);
		gpio_direction_output(goodix_info.rest_pin, 0);
	}
}

static void ft5x_ts_resume(struct early_suspend *handler)
{
	if(ctpfwinfo != 2)//is upgrading
	{   
		struct ft5x_ts_data *ft5x_ts= container_of(handler, struct ft5x_ts_data, early_suspend);
	    printk("ft5x_ts_resume\n");
	    //reset
	    ft5x_ts_reset();
	    //wakeup
	    ft5x_ts_wakeup();
	    enable_irq(TS_INT);
	    input_report_key(ft5x_ts->input_dev, BTN_TOUCH, 0);
		input_sync(ft5x_ts->input_dev);
	}
 }
#endif  //CONFIG_HAS_EARLYSUSPEND

static int ft5x_get_device_id(void)
{
	int ret = 0;
	int count = 0;
	char buf[2];
	while(count++ < 3) {
		msleep(20);
		buf[0] = 0xA8;
		ret = ft5x_i2c_rxdata(buf, 2);
		if(ret > 0) {
			printk("ft5x device id %02x\n", buf[0]);
			return buf[0];
		}
	}
	return -1;
}

static int ft5x_check_id(void)
{
	int ret = 0;
	int count = 0;
	char buf[2];
	while(count++ < 3) {
		msleep(20);
		buf[0] = 0xA6;
		ret = ft5x_i2c_rxdata(buf, 2);
		if(ret > 0) {
			printk("ft5x FW version %02x\n", buf[0]);
			return buf[0];
		}
	}
	return -1;
}


static int GetFirmwareSize(char *fw_path)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;

	if (NULL == pfile)
		pfile = filp_open(fw_path, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", fw_path);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);

	return fsize;
}

static int ReadFirmwareData(char *fw_path, unsigned char *fw_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	loff_t pos;
	mm_segment_t old_fs;

	if (NULL == pfile)
		pfile = filp_open(fw_path, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", fw_path);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, fw_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}

static ssize_t ft5x_ctpfwupdate_show(struct device *dev,struct device_attribute *attr, char *buf)
{
		if(ctpfwinfo==0)
		{
			sprintf(buf, "device id: 0x%x<br><br>fw version: 0x%x\n",ft5x_get_device_id(),ft5x_check_id());
		}else if(ctpfwinfo==1)
		{
			sprintf(buf, "%s", fw_status);
			strcpy(fw_status,"***err: Unkown***\n");
		}

		ctpfwinfo=0;
		return strlen(buf);
}

static ssize_t ft5x_ctpfwupdate_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	 char fwpath[128];
	 int fwsize=0;
	 unsigned char *pfwbuf = NULL;
	 static bool bforce=false;

	 if(buf==NULL){
	 	strcpy(fw_status,"err: fw name is null\n");

	 }else if(!strncmp(buf,"force",strlen("force"))){
		bforce=true;
	 	printk("[%d] ft5x use force update fw.\n",'\n');

	 }else if(!strncmp(buf,"info",strlen("info"))){
		 ctpfwinfo=0;

     }else if(!strncmp(buf,"status",strlen("status"))){
		 ctpfwinfo=1;

     }else if(strstr(buf,"/")){
		 memset(fwpath, 0, sizeof(fwpath));
		 sprintf(fwpath, "%s", buf);
		 fwpath[count - 1] = '\0';

		 fwsize=GetFirmwareSize(fwpath);
		 printk("fwpath: %s\n",fwpath);
		 printk("fwsize: 0x%x\n",fwsize);

		 if (fwsize <= 0) {
			strcpy(fw_status,"***err: get fw size error ***\n");
			return count;
		 }

		 if (fwsize < 8 || fwsize > 32 * 1024) {
			strcpy(fw_status,"***err: fw length error***\n");
			return count;
		 }

		 pfwbuf = kmalloc(fwsize + 32, GFP_ATOMIC);
		 if (ReadFirmwareData(fwpath, pfwbuf)) {
			 strcpy(fw_status,"***err: read fw failed***\n");
			 kfree(pfwbuf);
			 return count;
		 }

		 if ((pfwbuf[fwsize - 8] ^ pfwbuf[fwsize - 6]) == 0xFF
			&& (pfwbuf[fwsize - 7] ^ pfwbuf[fwsize - 5]) == 0xFF
			&& (pfwbuf[fwsize - 3] ^ pfwbuf[fwsize - 4]) == 0xFF) {

			CTPM_FW=pfwbuf;
			CTPM_FW_LEN=fwsize;
			ctpfwinfo=2;
			disable_irq(TS_INT);
			fts_download_fw(bforce);
			enable_irq(TS_INT);
			ctpfwinfo=0;
			bforce=false;
			kfree(pfwbuf);
		 }else
			 strcpy(fw_status,"***err: fw format error***\n");
	 }

     return count;
}
static DEVICE_ATTR(ctpfwupdate, S_IRUGO|S_IWUSR, ft5x_ctpfwupdate_show, ft5x_ctpfwupdate_store);


static void ft5x_load_param(void)
{
	pft5x_ts_fw = env_get_str("ft5x_ts_fw", NULL);
	screen_max_x = env_get_u32("lcd_h_vd", 1024);
	screen_max_y = env_get_u32("lcd_v_vd", 768);

	revert_x_flag = env_get_u32("ctp_ft5x_ts_x_flag", 0);
	revert_y_flag = env_get_u32("ctp_ft5x_ts_y_flag", 0);
	exchange_x_y_flag = env_get_u32("ctp_ft5x_ts_x_y_flag", 0);

	if(pft5x_ts_fw!=NULL)
	{
		int num=0;
		printk("env get ft5x_ts_fw: %s\n",pft5x_ts_fw);
		if(!strcmp(pft5x_ts_fw,"auto"))
		{
			char deviceid=ft5x_get_device_id();
			char fw_deviceid;
			while(ctp_fw_list[num].pFW_NAME!=NULL)
			{
				CTPM_FW=ctp_fw_list[num].pFW_DATA;
				CTPM_FW_LEN=ctp_fw_list[num].FW_LEN;
				fw_deviceid = fts_ctpm_get_upg_id(CTPM_FW,CTPM_FW_LEN);
				printk("ft5x_ts %s  deviceid:%x?=%x\n",ctp_fw_list[num].pFW_NAME,deviceid,fw_deviceid);
				if(deviceid==fw_deviceid)
					return;
				num++;
			}

		}else
		{
			while(ctp_fw_list[num].pFW_NAME!=NULL)
			{
				if(!strcmp(pft5x_ts_fw,ctp_fw_list[num].pFW_NAME))
				{
					CTPM_FW=ctp_fw_list[num].pFW_DATA;
					CTPM_FW_LEN=ctp_fw_list[num].FW_LEN;
					return;
				}
				num++;
			}
	   }
	}
	CTPM_FW=NULL;
	CTPM_FW_LEN=0;
}


static void request_gpio(void) {
	int ret;
	if (goodix_info.init_platform_hw) {
		goodix_info.init_platform_hw();
	}
	ret = gpio_request(goodix_info.irq_pin, "TS_INT");    //Request IO
	if (ret < 0)  {
		printk("Failed to request GPIO:%d, ERRNO:%d\n",goodix_info.irq_pin, ret);
	}
	else {
		gpio_direction_input(goodix_info.irq_pin);
		gpio_pull_updown(goodix_info.irq_pin, 1);
	}
}

static int ft5x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x_ts_data *ft5x_ts;
	struct input_dev *input_dev;

	int err = 0;

	printk("ft5x_ts_probe\n");

	if(!ctp_supported("ft5x06")) {
		return -ENODEV;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}


	ft5x_ts = kzalloc(sizeof(*ft5x_ts), GFP_KERNEL);
	if (!ft5x_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	this_client = client;

	this_client->addr = client->addr;
	i2c_set_clientdata(client, ft5x_ts);

	request_gpio();
	//reset
	ft5x_ts_reset();
	//wakeup
	ft5x_ts_wakeup();

	if(ft5x_check_id() < 0) {
		dev_err(&client->dev, "ft5x_ts_probe: not found devices\n");
		err = -ENODEV;
		goto exit_create_singlethread;
	}
	//ft5x_set_device_id(0x78);
	ft5x_get_device_id();


	ft5x_load_param();
	//fts_download_fw(false);
	ctp_register("ft5x06");

	INIT_WORK(&ft5x_ts->pen_event_work, ft5x_ts_pen_irq_work);

	ft5x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft5x_ts->input_dev = input_dev;

	if(key_number > 0) {
		int i;
		input_dev->evbit[0] = BIT_MASK(EV_KEY);
		for (i = 0; i < key_number; i++)
			set_bit(ts_keys[i].key, input_dev->keybit);
	}
	if(tp_number > 1) {
		set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
		set_bit(ABS_MT_POSITION_X, input_dev->absbit);
		set_bit(ABS_MT_POSITION_Y, input_dev->absbit);

#ifdef FOR_TSLIB_TEST
		set_bit(BTN_TOUCH, input_dev->keybit);
#endif

		input_set_abs_params(input_dev,
		         ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
		input_set_abs_params(input_dev,
		         ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
		input_set_abs_params(input_dev,
		         ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
		input_set_abs_params(input_dev,
		         ABS_MT_TRACKING_ID, 0, tp_number-1, 0, 0);
	}
	else {
		set_bit(ABS_X, input_dev->absbit);
		set_bit(ABS_Y, input_dev->absbit);
		set_bit(ABS_PRESSURE, input_dev->absbit);
		set_bit(BTN_TOUCH, input_dev->keybit);

		input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
		input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
		input_set_abs_params(input_dev,
		         ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
	}

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	input_dev->name = FT5X_NAME; //dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"ft5x_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
	/*device attribute*/
	// /sys/devices/virtual/input/input*/fw
	if (0 != sysfs_create_file(&input_dev->dev.kobj, &dev_attr_ctpfwupdate.attr)){
		printk("sysfs_create_file dev_attr_ctpfwupdate failed \r\n");
		sysfs_remove_file(&client->dev.kobj, &dev_attr_ctpfwupdate.attr);
	}


#ifdef CONFIG_HAS_EARLYSUSPEND
	ft5x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x_ts->early_suspend.suspend = ft5x_ts_suspend;
	ft5x_ts->early_suspend.resume	= ft5x_ts_resume;
	register_early_suspend(&ft5x_ts->early_suspend);
#endif

	err = request_irq(TS_INT, ft5x_ts_interrupt, IRQF_TRIGGER_FALLING | IRQF_SHARED, "ft5x_ts", ft5x_ts);

	if (err < 0) {
		dev_err(&client->dev, "ft5x_ts_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	printk("probe over\n");
    return 0;

exit_irq_request_failed:
	cancel_work_sync(&ft5x_ts->pen_event_work);
	destroy_workqueue(ft5x_ts->ts_workqueue);
	enable_irq(TS_INT);
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(TS_INT, ft5x_ts);
exit_create_singlethread:
	i2c_set_clientdata(client, NULL);
	kfree(ft5x_ts);
	gpio_free(goodix_info.rest_pin);
	gpio_free(goodix_info.irq_pin);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int __devexit ft5x_ts_remove(struct i2c_client *client)
{
    struct ft5x_ts_data *ft5x_ts = i2c_get_clientdata(client);

    ft5x_set_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);

    printk("ft5x_ts_remove\n");
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ft5x_ts->early_suspend);
#endif
	free_irq(TS_INT, ft5x_ts);
	input_unregister_device(ft5x_ts->input_dev);
	kfree(ft5x_ts);
	cancel_work_sync(&ft5x_ts->pen_event_work);
	destroy_workqueue(ft5x_ts->ts_workqueue);
	if(pft5x_ts_fw != NULL)
    		sysfs_remove_file(&client->dev.kobj, &dev_attr_ctpfwupdate.attr);
    i2c_set_clientdata(client, NULL);
    return 0;
}

static const struct i2c_device_id ft5x_ts_id[] = {
	{ FT5X_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ft5x_ts_id);

static struct i2c_driver ft5x_ts_driver = {
	.probe		= ft5x_ts_probe,
	.remove		= __devexit_p(ft5x_ts_remove),
	.id_table	= ft5x_ts_id,
	.driver	= {
		.name	= FT5X_NAME,
		.owner	= THIS_MODULE,
	},
};

static struct i2c_board_info i2c_info_dev =  {
	I2C_BOARD_INFO(FT5X_NAME, 0x38),
	.platform_data	= NULL,
};

static int add_ctp_device(void) {
	int twi_id = 2;
	struct i2c_adapter *adap;

	adap = i2c_get_adapter(twi_id);
	i2c_info_dev.addr = env_get_u32("ft5x_ts_addr", i2c_info_dev.addr);
	i2c_new_device(adap, &i2c_info_dev);

	return 0;
}

static int __init ft5x_ts_init(void)
{
    int ret = -1;

    printk("ft5x_ts_init\n");

	if(!ctp_supported("ft5x06")) {
		return -ENODEV;
	}

    add_ctp_device();
    ret = i2c_add_driver(&ft5x_ts_driver);

	return ret;
}

static void __exit ft5x_ts_exit(void)
{
	printk("ft5x_ts_exit\n");
	i2c_del_driver(&ft5x_ts_driver);
}

late_initcall(ft5x_ts_init);
module_exit(ft5x_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x TouchScreen driver");
MODULE_LICENSE("GPL");

