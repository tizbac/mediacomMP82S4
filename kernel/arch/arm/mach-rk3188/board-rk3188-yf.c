/*
 *
 * Copyright (C) 2012 ROCKCHIP, Inc.
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/mmc/host.h>
#include <linux/ion.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <mach/dvfs.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/hardware/gic.h>

#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <linux/rk_fb.h>
#include <linux/regulator/machine.h>
#include <linux/rfkill-rk.h>
#include <linux/sensor-dev.h>
#include <linux/mfd/tps65910.h>
#include <linux/regulator/act8846.h>
#include <linux/regulator/rk29-pwm-regulator.h>
#include <plat/efuse.h>
#include <mach/yfmach.h>
#if defined(CONFIG_CT36X_TS)
#include <linux/ct36x.h>
#endif
#if defined(CONFIG_MFD_RK610)
#include <linux/mfd/rk610_core.h>
#endif

#if defined(CONFIG_MFD_RK616)
#include <linux/mfd/rk616.h>
#endif

#if defined(CONFIG_DP_ANX6345)
	#include<linux/anx6345.h>
#endif

#if defined(CONFIG_RK_HDMI)
	#include "../../../drivers/video/rockchip/hdmi/rk_hdmi.h"
#endif

#if defined(CONFIG_SPIM_RK29)
#include "../../../drivers/spi/rk29_spim.h"
#endif
#if defined(CONFIG_MT6229)
#include <linux/mt6229.h>
#endif
#if defined(CONFIG_GPS_RK)
#include "../../../drivers/misc/gps/rk_gps/rk_gps.h"
#endif

#if defined(CONFIG_MT6620)
#include <linux/gps.h>
#endif

#include "yfcam.c"
#include <plat/key.h>

static struct rk29_keys_button key_button[] = {
	{
		.desc	= "play",
		.code	= KEY_POWER,
		.gpio	= RK30_PIN0_PA4, 
		.active_low = PRESS_LEV_LOW,
		.wakeup	= 1,
	},
	{
		.desc	= "vol+",
		.code	= KEY_VOLUMEUP,
		.adc_value	= 1,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc	= "vol-",
		.code	= KEY_VOLUMEDOWN,
		.adc_value	= 180,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc	= "esc",
		.code	= KEY_BACK,
		.adc_value	= 510,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
};
struct rk29_keys_platform_data rk29_keys_pdata = {
	.buttons	= key_button,
	.nbuttons	= ARRAY_SIZE(key_button),
	.chn	= 1,  //chn: 0-7, if do not use ADC,set 'chn' -1
};

/*
     v1.0 : 	ignore
     v1.1 :      rk610 lvds + rk610 codec + MT5931_MT6622 + light photoresistor + adc/cw2015
     v1.2 :      lvds       + rt5631      + M500          + us5151              + adc
*/
#define DS1006H_V1_2_SUPPORT  1
int get_harware_version()
{
    #if DS1006H_V1_2_SUPPORT
        return 2;
    #else
        return 1;
    #endif
}
EXPORT_SYMBOL_GPL(get_harware_version);

#if defined(CONFIG_MACH_RK_YF)
static int s_touch_reset;
static int s_touch_int;
#define TOUCH_RESET_PIN    s_touch_reset
#define TOUCH_INT_PIN      s_touch_int
#define TOUCH_PWR_PIN      INVALID_GPIO

int goodix_init_platform_hw(void)
{
	int ret;

	if (TOUCH_PWR_PIN != INVALID_GPIO) {
		//yf_mux_api_set(TOUCH_PWR_PIN, 0);
		ret = gpio_request(TOUCH_PWR_PIN, "goodix power pin");
		if (ret != 0) {
			gpio_free(TOUCH_PWR_PIN);
			printk("goodix request power error\n");
			return -EIO;
		}
		gpio_direction_output(TOUCH_PWR_PIN, 0);
		gpio_set_value(TOUCH_PWR_PIN, GPIO_LOW);
		msleep(100);
	}

	if (TOUCH_RESET_PIN != INVALID_GPIO) {
		//yf_mux_api_set(TOUCH_RESET_PIN, 0);
		ret = gpio_request(TOUCH_RESET_PIN, "goodix reset pin");
		if (ret != 0) {
			gpio_free(TOUCH_RESET_PIN);
			printk("goodix request reset error\n");
			return -EIO;
		}
		gpio_direction_output(TOUCH_RESET_PIN, 1);
		msleep(100);
	}
	if (TOUCH_PWR_PIN != INVALID_GPIO) {
		//yf_mux_api_set(TOUCH_INT_PIN, 0);
	}
	return 0;
}

struct goodix_platform_data goodix_info = {
	.model = 8105,
	//.irq_pin = TOUCH_INT_PIN,
	//.rest_pin = TOUCH_RESET_PIN,
	.init_platform_hw = goodix_init_platform_hw,
};
#endif

#if defined(CONFIG_CT36X_TS)

#define TOUCH_MODEL		363
#define TOUCH_MAX_X		1280
#define TOUCH_MAX_y		800
#define TOUCH_RESET_PIN		RK30_PIN0_PB6
#define TOUCH_INT_PIN		RK30_PIN1_PB7

static struct ct36x_platform_data ct36x_info = {
	.model   = TOUCH_MODEL,
	.x_max   = TOUCH_MAX_X,
	.y_max   = TOUCH_MAX_y,

	.rst_io = {
		.gpio = TOUCH_RESET_PIN,
		.active_low = 1,
	},
	.irq_io = {
		.gpio = TOUCH_INT_PIN,
		.active_low = 1,
	},
	.orientation = {1, 0, 0, 1},
};
#endif
static struct spi_board_info board_spi_devices[] = {
};

/***********************************************************
*	rk30  backlight
************************************************************/
#ifdef CONFIG_BACKLIGHT_RK29_BL
#define PWM_ID            3
#define PWM_MODE	  PWM3
#define PWM_EFFECT_VALUE  0

#define LCD_DISP_ON_PIN

#ifdef  LCD_DISP_ON_PIN
#define BL_EN_PIN         RK30_PIN0_PA2
#define BL_EN_VALUE       GPIO_HIGH
static int s_bl_suspend;
#endif

static int s_lcd_en2 = INVALID_GPIO;

static int rk29_backlight_io_init(void)
{
	int ret = 0;

	if(s_lcd_en2 != INVALID_GPIO) {
		gpio_direction_output(s_lcd_en2, GPIO_LOW);
		msleep(50);
	}
	iomux_set(PWM_MODE);
#ifdef  LCD_DISP_ON_PIN
	msleep(100);
	ret = gpio_request(BL_EN_PIN, NULL);
	if (ret != 0) {
		gpio_free(BL_EN_PIN);
	}

	gpio_direction_output(BL_EN_PIN, 0);
	gpio_set_value(BL_EN_PIN, BL_EN_VALUE);
#endif

	return ret;
}

static int rk29_backlight_io_deinit(void)
{
	int ret = 0, pwm_gpio;

#ifdef  LCD_DISP_ON_PIN
	gpio_free(BL_EN_PIN);
#endif

	pwm_gpio = iomux_mode_to_gpio(PWM_MODE);
	gpio_request(pwm_gpio, NULL);
	gpio_direction_output(pwm_gpio, GPIO_LOW);
	if(s_lcd_en2 != INVALID_GPIO) {
		msleep(50);
		gpio_direction_output(s_lcd_en2, GPIO_HIGH);
	}
	return ret;
}

static int rk29_backlight_pwm_suspend(void)
{
	int ret = 0, pwm_gpio;

	s_bl_suspend = 1;
	pwm_gpio = iomux_mode_to_gpio(PWM_MODE);
	if (gpio_request(pwm_gpio, NULL)) {
		printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
		return -1;
	}
	gpio_direction_output(pwm_gpio, GPIO_LOW);
#ifdef  LCD_DISP_ON_PIN
	gpio_direction_output(BL_EN_PIN, 0);
	gpio_set_value(BL_EN_PIN, !BL_EN_VALUE);
#endif
	if(s_lcd_en2 != INVALID_GPIO) {
		msleep(50);
		gpio_direction_output(s_lcd_en2, GPIO_HIGH);
	}
	return ret;
}

static int rk29_backlight_pwm_resume(void)
{
	int pwm_gpio = iomux_mode_to_gpio(PWM_MODE);

	if(s_lcd_en2 != INVALID_GPIO) {
		gpio_direction_output(s_lcd_en2, GPIO_LOW);
		msleep(50);
	}
	gpio_free(pwm_gpio);
	iomux_set(PWM_MODE);
#ifdef  LCD_DISP_ON_PIN
	msleep(150);
	gpio_direction_output(BL_EN_PIN, 1);
	gpio_set_value(BL_EN_PIN, BL_EN_VALUE);
#endif
	s_bl_suspend = 0;
	return 0;
}

static struct rk29_bl_info rk29_bl_info = {
	.io_init = rk29_backlight_io_init,
	.io_deinit = rk29_backlight_io_deinit,
	.pwm_suspend = rk29_backlight_pwm_suspend,
	.pwm_resume = rk29_backlight_pwm_resume,
};

static struct platform_device rk29_device_backlight = {
	.name	= "rk29_backlight",
	.id 	= -1,
	.dev	= {
		.platform_data  = &rk29_bl_info,
	}
};

#endif

/*MMA8452 gsensor*/
#if defined (CONFIG_GS_MMA8452)
#define MMA8452_INT_PIN   RK30_PIN0_PB7

static int mma8452_init_platform_hw(void)
{
	return 0;
}

static struct sensor_platform_data mma8452_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 30,
        .init_platform_hw = mma8452_init_platform_hw,
        .orientation = {-1, 0, 0, 0, 0, -1, 0, 1, 0},
};
#endif
#if defined (CONFIG_GS_LIS3DH)
#define LIS3DH_INT_PIN   RK30_PIN0_PB7

static int lis3dh_init_platform_hw(void)
{

        return 0;
}

static struct sensor_platform_data lis3dh_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 30,
        .init_platform_hw = lis3dh_init_platform_hw,
	.orientation = {-1, 0, 0, 0, 1, 0, 0, 0, -1},
};
#endif

#if defined (CONFIG_COMPASS_AK8963)
static struct sensor_platform_data akm8963_info =
{
       .type = SENSOR_TYPE_COMPASS,
       .irq_enable = 1,
       .poll_delay_ms = 30,
       .m_layout = 
       {
               {
                       {0, 1, 0},
                       {1, 0, 0},
                       {0, 0, -1},
               },

               {
                       {1, 0, 0},
                       {0, 1, 0},
                       {0, 0, 1},
               },

               {
                       {0, -1, 0},
                       {-1, 0, 0},
                       {0, 0, -1},
               },

               {
                       {1, 0, 0},
                       {0, 1, 0},
                       {0, 0, 1},
               },
       }
};

#endif

#if defined(CONFIG_LS_PHOTORESISTOR)
static struct sensor_platform_data light_photoresistor_info = {
	.type = SENSOR_TYPE_LIGHT,
	.irq_enable = 0,
        .address = 2   ,
	.poll_delay_ms = 200,
};
#endif

#if defined (CONFIG_COMPASS_AK8975)
static struct sensor_platform_data akm8975_info =
{
	.type = SENSOR_TYPE_COMPASS,
	.irq_enable = 1,
	.poll_delay_ms = 30,
	.m_layout = 
	{
		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},

		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},

		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},

		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},
	}
};

#endif

#if defined(CONFIG_MT6229)
static int mt6229_io_init(void)
{
      #if 0
	rk30_mux_api_set(GPIO2B6_LCDC1DATA14_SMCADDR18_TSSYNC_NAME, GPIO2B_GPIO2B6);
      k30_mux_api_set(GPIO4D2_SMCDATA10_TRACEDATA10_NAME, GPIO4D_GPIO4D2);
	rk30_mux_api_set(GPIO2B7_LCDC1DATA15_SMCADDR19_HSADCDATA7_NAME, GPIO2B_GPIO2B7);
	rk30_mux_api_set(GPIO2C0_LCDCDATA16_GPSCLK_HSADCCLKOUT_NAME, GPIO2C_GPIO2C0);
	rk30_mux_api_set(GPIO2C1_LCDC1DATA17_SMCBLSN0_HSADCDATA6_NAME, GPIO2C_GPIO2C1);
	rk30_mux_api_set(GPIO2C1_LCDC1DATA17_SMCBLSN0_HSADCDATA6_NAME, GPIO2C_GPIO2C1);
      #endif
	 return 0;
}

static int mt6229_io_deinit(void)
{
	
	return 0;
}
 
struct rk29_mt6229_data rk29_mt6229_info = {
	.io_init = mt6229_io_init,
  	.io_deinit = mt6229_io_deinit,
	.modem_power_en = RK30_PIN0_PC6,
	.bp_power = RK30_PIN2_PD5,
	.modem_usb_en = RK30_PIN0_PC7,
	.modem_uart_en = RK30_PIN2_PD4,
	.bp_wakeup_ap = RK30_PIN0_PC5,
	.ap_ready = RK30_PIN0_PC4,

};
struct platform_device rk29_device_mt6229 = {	
        .name = "mt6229",	
    	.id = -1,	
	.dev		= {
		.platform_data = &rk29_mt6229_info,
	}    	
    };
#endif

#if defined(CONFIG_GYRO_L3G4200D)

#include <linux/l3g4200d.h>
#define L3G4200D_INT_PIN  RK30_PIN0_PB4

static int l3g4200d_init_platform_hw(void)
{
	return 0;
}

static struct sensor_platform_data l3g4200d_info = {
	.type = SENSOR_TYPE_GYROSCOPE,
	.irq_enable = 1,
	.poll_delay_ms = 0,
	.orientation = {1, 0, 0 , 0 , -1, 0, 0, 0, -1},
	.init_platform_hw = l3g4200d_init_platform_hw,
	.x_min = 40,//x_min,y_min,z_min = (0-100) according to hardware
	.y_min = 40,
	.z_min = 20,
};

#endif

#ifdef CONFIG_LS_CM3217
static struct sensor_platform_data cm3217_info = {
	.type = SENSOR_TYPE_LIGHT,
	.irq_enable = 0,
	.poll_delay_ms = 500,
};

#endif

#ifdef CONFIG_FB_ROCKCHIP

#define LCD_CS_PIN         INVALID_GPIO
#define LCD_CS_VALUE       GPIO_HIGH

static int s_lcd_en = RK30_PIN0_PB0;
#define LCD_EN_PIN         s_lcd_en
#define LCD_EN_VALUE       GPIO_LOW

static void yftech_power_ctrl(bool on)
{
#if defined (CONFIG_KP_AXP22)
	extern void yftech_axp_power_dc1sw(bool on);
	if (pmic_is_axp228()) {
		const char * ldos_name[] = {
			"axp22_ldoio0",		// VCC_TP
			NULL
		};
		const char ** p_name = ldos_name;

		while (*p_name) {
			struct regulator *regulator;

			regulator = regulator_get(NULL, *p_name);
			if (!IS_ERR_OR_NULL(regulator)) {
				if (regulator_is_enabled(regulator) != on) {
					int ret = on ? regulator_enable(regulator) : regulator_disable(regulator);
				}
				regulator_put(regulator);
			} else {
				printk(KERN_ERR "regulator_get(%s) error\n", *p_name);
			}
			++p_name;
		}

		yftech_axp_power_dc1sw(on);
	}
#endif
}

static int rk_fb_io_init(struct rk29_fb_setting_info *fb_setting)
{
	int ret = 0;

	if(LCD_CS_PIN !=INVALID_GPIO)
	{
		ret = gpio_request(LCD_CS_PIN, NULL);
		if (ret != 0)
		{
			gpio_free(LCD_CS_PIN);
			printk(KERN_ERR "request lcd cs pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(LCD_CS_PIN, LCD_CS_VALUE);
		}
	}

	if(LCD_EN_PIN !=INVALID_GPIO)
	{
		ret = gpio_request(LCD_EN_PIN, NULL);
		if (ret != 0)
		{
			gpio_free(LCD_EN_PIN);
			printk(KERN_ERR "request lcd en pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(LCD_EN_PIN, LCD_EN_VALUE);
		}
	}
	return 0;
}
static int rk_fb_io_disable(void)
{
	if(LCD_CS_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_CS_PIN, !LCD_CS_VALUE);
	}
	if(LCD_EN_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_EN_PIN, !LCD_EN_VALUE);
	}
	yftech_power_ctrl(false);
	return 0;
}
static int rk_fb_io_enable(void)
{
	if(LCD_CS_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_CS_PIN, LCD_CS_VALUE);
	}
	if(LCD_EN_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_EN_PIN, LCD_EN_VALUE);
	}
	yftech_power_ctrl(true);
	return 0;
}

#if defined(CONFIG_LCDC0_RK3188) || defined(CONFIG_LCDC0_RK3066B)
struct rk29fb_info lcdc0_screen_info = {
	.prop           = EXTEND,       //extend display device
       .lcd_info  = NULL,
       .set_screen_info = hdmi_init_lcdc,

};
#endif

#if defined(CONFIG_LCDC1_RK3188) || defined(CONFIG_LCDC1_RK3066B)
#define LCD_RST_PIN			RK30_PIN0_PA7
#define MIPI_RST_PIN        RK30_PIN0_PC3
static int rk29_lcd_io_init(void)
{
/*	int ret = gpio_request(LCD_RST_PIN, NULL);
	if (ret != 0) {
		gpio_free(LCD_RST_PIN);
		printk("%s: request LCD_RST_PIN error\n", __func__);
		return -EIO;
	}*/
	gpio_request(MIPI_RST_PIN, NULL);
	gpio_direction_output(LCD_RST_PIN, 0);
	msleep(5);
	gpio_direction_output(LCD_RST_PIN, 1);
	msleep(5);
	return 0;
}

static int rk29_lcd_io_deinit(void)
{
//	gpio_free(LCD_RST_PIN);
	gpio_free(MIPI_RST_PIN);
	return 0;
}

static struct rk29lcd_info rk29_lcd_info = {
    .reset_pin = MIPI_RST_PIN,
    .io_init   = rk29_lcd_io_init,
    .io_deinit = rk29_lcd_io_deinit,
};

struct rk29fb_info lcdc1_screen_info = {
	.prop	   = PRMRY,		//primary display device
	.io_init   = rk_fb_io_init,
	.io_disable = rk_fb_io_disable,
	.io_enable = rk_fb_io_enable,
	.set_screen_info = set_lcd_info,
	
};
#endif

int lcd_switch_lcdc(void) {
	return env_get_u32("lcd_switch_lcdc", 1);
}

#ifdef CONFIG_DP_ANX6345

	#define DVDD33_EN_PIN 		RK30_PIN0_PB0
	#define DVDD33_EN_VALUE 	GPIO_LOW

	#define DVDD18_EN_PIN 		RK30_PIN3_PD4
	#define DVDD18_EN_VALUE 	GPIO_HIGH

	#define EDP_RST_PIN 		RK30_PIN0_PA7
	static int rk_edp_power_ctl(void)
	{
		int ret;
		ret = gpio_request(DVDD33_EN_PIN, "dvdd33_en_pin");
		if (ret != 0)
		{
			gpio_free(DVDD33_EN_PIN);
			printk(KERN_ERR "request dvdd33 en pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(DVDD33_EN_PIN, DVDD33_EN_VALUE);
		}
		msleep(5);

		ret = gpio_request(DVDD18_EN_PIN, "dvdd18_en_pin");
		if (ret != 0)
		{
			gpio_free(DVDD18_EN_PIN);
			printk(KERN_ERR "request dvdd18 en pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(DVDD18_EN_PIN, DVDD18_EN_VALUE);
		}

		ret = gpio_request(EDP_RST_PIN, "edp_rst_pin");
		if (ret != 0)
		{
			gpio_free(EDP_RST_PIN);
			printk(KERN_ERR "request rst pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(EDP_RST_PIN, GPIO_LOW);
			msleep(50);
			gpio_direction_output(EDP_RST_PIN, GPIO_HIGH);
			msleep(50);
		}
		return 0;

	}
	static struct anx6345_platform_data anx6345_platform_data = {
		.power_ctl 	= rk_edp_power_ctl,
		.dvdd33_en_pin 	= DVDD33_EN_PIN,
		.dvdd33_en_val 	= DVDD33_EN_VALUE,
		.dvdd18_en_pin 	= DVDD18_EN_PIN,
		.dvdd18_en_val 	= DVDD18_EN_VALUE,
		.edp_rst_pin   	= EDP_RST_PIN,
	};
	static struct i2c_board_info __initdata i2c_anx6345_info = {
		.type          = "anx6345",
		.addr          = 0x39,
		.flags         = 0,
		.platform_data = &anx6345_platform_data,
	};
#endif

static struct resource resource_fb[] = {
	[0] = {
		.name  = "fb0 buf",
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = "ipp buf",  //for rotate
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.name  = "fb2 buf",
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device device_fb = {
	.name		= "rk-fb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resource_fb),
	.resource	= resource_fb,
};
#endif

#if defined(CONFIG_ARCH_RK3188)
static struct resource resource_mali[] = {
	[0] = {
	.name  = "ump buf",
	.start = 0,
	.end   = 0,
	.flags = IORESOURCE_MEM,
	},

};

static struct platform_device device_mali= {
	.name		= "mali400_ump",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resource_mali),
	.resource	= resource_mali,
};
#endif

#if defined(CONFIG_LCDC0_RK3188) || defined(CONFIG_LCDC0_RK3066B)
static struct resource resource_lcdc0[] = {
	[0] = {
		.name  = "lcdc0 reg",
		.start = RK30_LCDC0_PHYS,
		.end   = RK30_LCDC0_PHYS + RK30_LCDC0_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	
	[1] = {
		.name  = "lcdc0 irq",
		.start = IRQ_LCDC0,
		.end   = IRQ_LCDC0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device device_lcdc0 = {
	.name		  = "rk30-lcdc",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(resource_lcdc0),
	.resource	  = resource_lcdc0,
	.dev 		= {
		.platform_data = &lcdc0_screen_info,
	},
};
#endif
#if defined(CONFIG_LCDC1_RK3188) || defined(CONFIG_LCDC1_RK3066B)
static struct resource resource_lcdc1[] = {
	[0] = {
		.name  = "lcdc1 reg",
		.start = RK30_LCDC1_PHYS,
		.end   = RK30_LCDC1_PHYS + RK30_LCDC1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = "lcdc1 irq",
		.start = IRQ_LCDC1,
		.end   = IRQ_LCDC1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device device_lcdc1 = {
	.name		  = "rk30-lcdc",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(resource_lcdc1),
	.resource	  = resource_lcdc1,
	.dev 		= {
		.platform_data = &lcdc1_screen_info,
	},
};
#endif

#if defined(CONFIG_MFD_RK610)
#define RK610_RST_PIN 			RK30_PIN3_PB2
static int rk610_power_on_init(void)
{
	int ret;
	if(RK610_RST_PIN != INVALID_GPIO)
	{
		ret = gpio_request(RK610_RST_PIN, "rk610 reset");
		if (ret)
		{
			printk(KERN_ERR "rk610_control_probe request gpio fail\n");
		}
		else 
		{
			gpio_direction_output(RK610_RST_PIN, GPIO_LOW);
			msleep(10);
			gpio_direction_output(RK610_RST_PIN, GPIO_HIGH);
			msleep(10);
		}
	}

	return 0;
	
}

static struct rk610_ctl_platform_data rk610_ctl_pdata = {
	.rk610_power_on_init = rk610_power_on_init,
};
#endif

#if defined(CONFIG_MFD_RK616)
#define RK616_SCL_RATE			(100*1000)   //i2c scl rate
static struct rk616_platform_data rk616_pdata = {
	.power_init = rk610_power_on_init,
	.scl_rate   = RK616_SCL_RATE,
	.lcd0_func = INPUT,             //port lcd0 as input
	.lcd1_func = INPUT,             //port lcd1 as input
	.lvds_ch_nr = 1,		//the number of used lvds channel  
	.hdmi_irq = INVALID_GPIO,
	.spk_ctl_gpio = RK30_PIN2_PD7,
	.hp_ctl_gpio = RK30_PIN2_PD7,
};
#endif

#ifdef CONFIG_SND_SOC_RK610
static int rk610_codec_io_init(void)
{
//if need iomux.
//Must not gpio_request
	return 0;
}

static struct rk610_codec_platform_data rk610_codec_pdata = {
	.spk_ctl_io = RK30_PIN2_PD7,
	.io_init = rk610_codec_io_init,
	.boot_depop = 1,
};
#endif

#ifdef CONFIG_RK_HDMI
#define RK_HDMI_RST_PIN 			RK30_PIN3_PB2
static int rk_hdmi_power_init(void)
{
	int ret;

	if(RK_HDMI_RST_PIN != INVALID_GPIO)
	{
		if (gpio_request(RK_HDMI_RST_PIN, NULL)) {
			printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
			return -1;
		}
		gpio_direction_output(RK_HDMI_RST_PIN, GPIO_LOW);
		gpio_set_value(RK_HDMI_RST_PIN, GPIO_LOW);
		msleep(100);
		gpio_set_value(RK_HDMI_RST_PIN, GPIO_HIGH);
		msleep(50);
	}
	return 0;
}
static struct rk_hdmi_platform_data rk_hdmi_pdata = {
	//.io_init = rk_hdmi_power_init,
};
#endif
#ifdef CONFIG_ION
#define ION_RESERVE_SIZE        (80 * SZ_1M)
static struct ion_platform_data rk30_ion_pdata = {
	.nr = 1,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = ION_NOR_HEAP_ID,
			.name = "norheap",
			.size = ION_RESERVE_SIZE,
		}
	},
};

static struct platform_device device_ion = {
	.name = "ion-rockchip",
	.id = 0,
	.dev = {
		.platform_data = &rk30_ion_pdata,
	},
};
#endif


#ifdef CONFIG_SSD2828_RGB2MIPI
#include "../../../drivers/video/rockchip/transmitter/mipi_dsi.h"

struct ssd2828_t ssd2828_platdata = {
	.id = 0x2828,
	.reset = {
		.reset_pin = RK30_PIN0_PA7,
		.effect_value = GPIO_LOW,
	},
	.vddio = {
		.enable_pin = RK30_PIN0_PB0,
		.effect_value = GPIO_LOW,
	},

	.vdd_mipi = {
		.enable_pin = INVALID_GPIO,
		.effect_value = GPIO_LOW,
	},
	.shut = {                     //SHUT PIN
		.enable_pin = RK30_PIN3_PD4,
		.effect_value = GPIO_LOW,
	},
	.spi = {
		.cs = RK30_PIN0_PD7,
		.sck = RK30_PIN0_PD6,
		.miso = RK30_PIN0_PD4,
		.mosi = RK30_PIN0_PD5,
	},
};

//board
static struct platform_device device_ssd2828 = {
        .name   = "ssd2828",
        .id     = -1,
        .dev = {
                .platform_data = &ssd2828_platdata,
        },
};

#endif
/**************************************************************************************************
 * SDMMC devices,  include the module of SD,MMC,and sdio.noted by xbw at 2012-03-05
**************************************************************************************************/
#ifdef CONFIG_SDMMC_RK29
#include "board-rk3188-ds1006h-sdmmc-config.c"
#include "../plat-rk/rk-sdmmc-ops.c"
#include "../plat-rk/rk-sdmmc-wifi.c"
#endif //endif ---#ifdef CONFIG_SDMMC_RK29

#ifdef CONFIG_SDMMC0_RK29
static int rk29_sdmmc0_cfg_gpio(void)
{
#ifdef CONFIG_SDMMC_RK29_OLD
	iomux_set(MMC0_CMD);
	iomux_set(MMC0_CLKOUT);
	iomux_set(MMC0_D0);
	iomux_set(MMC0_D1);
	iomux_set(MMC0_D2);
	iomux_set(MMC0_D3);

	iomux_set_gpio_mode(iomux_mode_to_gpio(MMC0_DETN));

	gpio_request(RK30_PIN3_PA7, "sdmmc-power");
	gpio_direction_output(RK30_PIN3_PA7, GPIO_LOW);

#else
	rk29_sdmmc_set_iomux(0, 0xFFFF);

    #if defined(CONFIG_SDMMC0_RK29_SDCARD_DET_FROM_GPIO)
        #if SDMMC_USE_NEW_IOMUX_API
        iomux_set_gpio_mode(iomux_gpio_to_mode(RK29SDK_SD_CARD_DETECT_N));
        #else
        rk30_mux_api_set(RK29SDK_SD_CARD_DETECT_PIN_NAME, RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO);
        #endif
    #else
        #if SDMMC_USE_NEW_IOMUX_API       
        iomux_set(MMC0_DETN);
        #else
        rk30_mux_api_set(RK29SDK_SD_CARD_DETECT_PIN_NAME, RK29SDK_SD_CARD_DETECT_IOMUX_FMUX);
        #endif
    #endif	

#if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
	gpio_request(SDMMC0_WRITE_PROTECT_PIN, "sdmmc-wp");
	gpio_direction_input(SDMMC0_WRITE_PROTECT_PIN);
#endif

#endif

	return 0;
}

#define CONFIG_SDMMC0_USE_DMA
struct rk29_sdmmc_platform_data default_sdmmc0_data = {
	.host_ocr_avail =
	    (MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
	     MMC_VDD_33_34 | MMC_VDD_34_35 | MMC_VDD_35_36),
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
	.io_init = rk29_sdmmc0_cfg_gpio,

#if !defined(CONFIG_SDMMC_RK29_OLD)
	.set_iomux = rk29_sdmmc_set_iomux,
#endif

	.dma_name = "sd_mmc",
#ifdef CONFIG_SDMMC0_USE_DMA
	.use_dma = 1,
#else
	.use_dma = 0,
#endif

#if defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC) && defined(CONFIG_USE_SDMMC0_FOR_WIFI_DEVELOP_BOARD)
    .status = rk29sdk_wifi_mmc0_status,
    .register_status_notify = rk29sdk_wifi_mmc0_status_register,
#endif

#if defined(RK29SDK_SD_CARD_PWR_EN) || (INVALID_GPIO != RK29SDK_SD_CARD_PWR_EN)
    .power_en = RK29SDK_SD_CARD_PWR_EN,
    .power_en_level = RK29SDK_SD_CARD_PWR_EN_LEVEL,
#else
    .power_en = INVALID_GPIO,
    .power_en_level = GPIO_LOW,
#endif    
	.enable_sd_wakeup = 0,

#if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
	.write_prt = SDMMC0_WRITE_PROTECT_PIN,
	.write_prt_enalbe_level = SDMMC0_WRITE_PROTECT_ENABLE_VALUE;
#else
	.write_prt = INVALID_GPIO,
#endif

    .det_pin_info = {    
    #if defined(RK29SDK_SD_CARD_DETECT_N) || (INVALID_GPIO != RK29SDK_SD_CARD_DETECT_N)  
        .io             = RK29SDK_SD_CARD_DETECT_N, //INVALID_GPIO,
        .enable         = RK29SDK_SD_CARD_INSERT_LEVEL,
        #ifdef RK29SDK_SD_CARD_DETECT_PIN_NAME
        .iomux          = {
            .name       = RK29SDK_SD_CARD_DETECT_PIN_NAME,
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO
            .fgpio      = RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO,
            #endif
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FMUX
            .fmux       = RK29SDK_SD_CARD_DETECT_IOMUX_FMUX,
            #endif
        },
        #endif
    #else
        .io             = INVALID_GPIO,
        .enable         = GPIO_LOW,
    #endif    
    }, 

};
#endif // CONFIG_SDMMC0_RK29

#ifdef CONFIG_SDMMC1_RK29
#define CONFIG_SDMMC1_USE_DMA
static int rk29_sdmmc1_cfg_gpio(void)
{
#if defined(CONFIG_SDMMC_RK29_OLD)
	iomux_set(MMC1_CMD);
	iomux_set(MMC1_CLKOUT);
	iomux_set(MMC1_D0);
	iomux_set(MMC1_D1);
	iomux_set(MMC1_D2);
	iomux_set(MMC1_D3);
#else

#if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
	gpio_request(SDMMC1_WRITE_PROTECT_PIN, "sdio-wp");
	gpio_direction_input(SDMMC1_WRITE_PROTECT_PIN);
#endif

#endif

	return 0;
}

struct rk29_sdmmc_platform_data default_sdmmc1_data = {
	.host_ocr_avail =
	    (MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
	     MMC_VDD_33_34),

#if !defined(CONFIG_USE_SDMMC1_FOR_WIFI_DEVELOP_BOARD)
	.host_caps = (MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ |
		      MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
#else
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
#endif

	.io_init = rk29_sdmmc1_cfg_gpio,

#if !defined(CONFIG_SDMMC_RK29_OLD)
	.set_iomux = rk29_sdmmc_set_iomux,
#endif

	.dma_name = "sdio",
#ifdef CONFIG_SDMMC1_USE_DMA
	.use_dma = 1,
#else
	.use_dma = 0,
#endif

#if defined(CONFIG_WIFI_CONTROL_FUNC) || defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC)
    .status = rk29sdk_wifi_status,
    .register_status_notify = rk29sdk_wifi_status_register,
#endif

#if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
	.write_prt = SDMMC1_WRITE_PROTECT_PIN,
	    .write_prt_enalbe_level = SDMMC1_WRITE_PROTECT_ENABLE_VALUE;
#else
	.write_prt = INVALID_GPIO,
#endif

    #if defined(CONFIG_RK29_SDIO_IRQ_FROM_GPIO)
        .sdio_INT_gpio = RK29SDK_WIFI_SDIO_CARD_INT,
    #endif

    .det_pin_info = {    
#if defined(CONFIG_USE_SDMMC1_FOR_WIFI_DEVELOP_BOARD)
     #if defined(RK29SDK_SD_CARD_DETECT_N) || (INVALID_GPIO != RK29SDK_SD_CARD_DETECT_N)  
        .io             = RK29SDK_SD_CARD_DETECT_N,
     #else
         .io             = INVALID_GPIO,
     #endif   

        .enable         = RK29SDK_SD_CARD_INSERT_LEVEL,
        #ifdef RK29SDK_SD_CARD_DETECT_PIN_NAME
        .iomux          = {
            .name       = RK29SDK_SD_CARD_DETECT_PIN_NAME,
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO
            .fgpio      = RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO,
            #endif
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FMUX
            .fmux       = RK29SDK_SD_CARD_DETECT_IOMUX_FMUX,
            #endif
        },
        #endif
 #else
        .io             = INVALID_GPIO,
        .enable         = GPIO_LOW,
#endif
    },
   
	.enable_sd_wakeup = 0,
};
#endif //endif--#ifdef CONFIG_SDMMC1_RK29

/**************************************************************************************************
 * the end of setting for SDMMC devices
**************************************************************************************************/

#ifdef CONFIG_BATTERY_RK30_ADC_FAC
static struct rk30_adc_battery_platform_data rk30_adc_battery_platdata = {
        .dc_det_pin      = RK30_PIN0_PB2,
        .batt_low_pin    = INVALID_GPIO, 
        .charge_set_pin  = INVALID_GPIO,
        .charge_ok_pin   = RK30_PIN0_PA6,
	 .usb_det_pin = INVALID_GPIO,
        .dc_det_level    = GPIO_LOW,
        .charge_ok_level = GPIO_HIGH,

	.reference_voltage = 1800, // the rK2928 is 3300;RK3066 and rk29 are 2500;rk3066B is 1800;
       .pull_up_res = 200,     //divider resistance ,  pull-up resistor
       .pull_down_res = 120, //divider resistance , pull-down resistor

	.is_reboot_charging = 1,
        .save_capacity   = 1 ,
        .low_voltage_protection = 3600,    
};

static struct platform_device rk30_device_adc_battery = {
        .name   = "rk30-battery",
        .id     = -1,
        .dev = {
                .platform_data = &rk30_adc_battery_platdata,
        },
};
#endif
#ifdef CONFIG_RK30_PWM_REGULATOR
static int pwm_voltage_map[] = {
	800000,825000,850000, 875000,900000, 925000 ,950000, 975000,1000000, 1025000, 1050000, 1075000, 1100000, 1125000, 1150000, 1175000, 1200000, 1225000, 1250000, 1275000, 1300000, 1325000, 1350000,1375000
};

static struct regulator_consumer_supply pwm_dcdc1_consumers[] = {
	{
		.supply = "vdd_cpu",
	}
};

struct regulator_init_data pwm_regulator_init_dcdc[1] =
{
	{
		.constraints = {
			.name = "PWM_DCDC1",
			.min_uV = 600000,
			.max_uV = 1800000,	//0.6-1.8V
			.apply_uV = true,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(pwm_dcdc1_consumers),
		.consumer_supplies = pwm_dcdc1_consumers,
	},
};

static struct pwm_platform_data pwm_regulator_info[1] = {
	{
		.pwm_id = 1,
		.pwm_gpio = RK30_PIN3_PD4,
		.pwm_iomux_pwm = PWM1,
		.pwm_iomux_gpio = GPIO3_D4,
		.pwm_voltage = 1100000,
		.suspend_voltage = 1000000,
		.min_uV = 800000,
		.max_uV	= 1375000,
		.coefficient = 575,	//57.5%
		.pwm_voltage_map = pwm_voltage_map,
		.init_data	= &pwm_regulator_init_dcdc[0],
	},
};

struct platform_device pwm_regulator_device[1] = {
	{
		.name = "pwm-voltage-regulator",
		.id = 0,
		.dev		= {
			.platform_data = &pwm_regulator_info[0],
		}
	},
};
#endif

#ifdef CONFIG_RFKILL_RK
// bluetooth rfkill device, its driver in net/rfkill/rfkill-rk.c
static struct rfkill_rk_platform_data rfkill_rk_platdata = {
    .type               = RFKILL_TYPE_BLUETOOTH,

    .poweron_gpio       = { // BT_REG_ON
        .io             = INVALID_GPIO, //RK30_PIN3_PC7,
        .enable         = GPIO_HIGH,
        .iomux          = {
            .name       = "bt_poweron",
            .fgpio      = GPIO3_C7,
        },
    },

    .reset_gpio         = { // BT_RST
        .io             = RK30_PIN3_PD1, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_LOW,
        .iomux          = {
            .name       = "bt_reset",
            .fgpio      = GPIO3_D1,
       },
   }, 

    .wake_gpio          = { // BT_WAKE, use to control bt's sleep and wakeup
        .io             = RK30_PIN3_PC6, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_HIGH,
        .iomux          = {
            .name       = "bt_wake",
            .fgpio      = GPIO3_C6,
        },
    },

    .wake_host_irq      = { // BT_HOST_WAKE, for bt wakeup host when it is in deep sleep
        .gpio           = {
            .io         = RK30_PIN0_PA5, // set io to INVALID_GPIO for disable it
            .enable     = GPIO_LOW,      // set GPIO_LOW for falling, set 0 for rising
            .iomux      = {
                .name   = NULL,
            },
        },
    },

    .rts_gpio           = { // UART_RTS, enable or disable BT's data coming
        .io             = RK30_PIN1_PA3, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_LOW,
        .iomux          = {
            .name       = "bt_rts",
            .fgpio      = GPIO1_A3,
            .fmux       = UART0_RTSN,
        },
    },
};

static struct platform_device device_rfkill_rk = {
    .name   = "rfkill_rk",
    .id     = -1,
    .dev    = {
        .platform_data = &rfkill_rk_platdata,
    },
};
#endif

#if defined(CONFIG_GPS_RK)
int rk_gps_io_init(void)
{
	printk("%s \n", __FUNCTION__);
	
	rk30_mux_api_set(GPIO1B5_UART3RTSN_NAME, GPIO1B_GPIO1B5);//VCC_EN
	gpio_request(RK30_PIN1_PB5, NULL);
	gpio_direction_output(RK30_PIN1_PB5, GPIO_LOW);

	rk30_mux_api_set(GPIO1B4_UART3CTSN_GPSRFCLK_NAME, GPIO1B_GPSRFCLK);//GPS_CLK
	rk30_mux_api_set(GPIO1B2_UART3SIN_GPSMAG_NAME, GPIO1B_GPSMAG);//GPS_MAG
	rk30_mux_api_set(GPIO1B3_UART3SOUT_GPSSIG_NAME, GPIO1B_GPSSIG);//GPS_SIGN

	rk30_mux_api_set(GPIO1A6_UART1CTSN_SPI0CLK_NAME, GPIO1A_GPIO1A6);//SPI_CLK
	gpio_request(RK30_PIN1_PA6, NULL);
	gpio_direction_output(RK30_PIN1_PA6, GPIO_LOW);

	rk30_mux_api_set(GPIO1A5_UART1SOUT_SPI0TXD_NAME, GPIO1A_GPIO1A5);//SPI_MOSI
	gpio_request(RK30_PIN1_PA5, NULL);
	gpio_direction_output(RK30_PIN1_PA5, GPIO_LOW);	

	rk30_mux_api_set(GPIO1A7_UART1RTSN_SPI0CSN0_NAME, GPIO1A_GPIO1A7);//SPI_CS
	gpio_request(RK30_PIN1_PA7, NULL);
	gpio_direction_output(RK30_PIN1_PA7, GPIO_LOW);		
	return 0;
}
int rk_gps_power_up(void)
{
	printk("%s \n", __FUNCTION__);

	return 0;
}

int rk_gps_power_down(void)
{
	printk("%s \n", __FUNCTION__);

	return 0;
}

int rk_gps_reset_set(int level)
{
	return 0;
}
int rk_enable_hclk_gps(void)
{
	printk("%s \n", __FUNCTION__);
	clk_enable(clk_get(NULL, "hclk_gps"));
	return 0;
}
int rk_disable_hclk_gps(void)
{
	printk("%s \n", __FUNCTION__);
	clk_disable(clk_get(NULL, "hclk_gps"));
	return 0;
}
struct rk_gps_data rk_gps_info = {
	.io_init = rk_gps_io_init,
	.power_up = rk_gps_power_up,
	.power_down = rk_gps_power_down,
	.reset = rk_gps_reset_set,
	.enable_hclk_gps = rk_enable_hclk_gps,
	.disable_hclk_gps = rk_disable_hclk_gps,
	.GpsSign = RK30_PIN1_PB3,
	.GpsMag = RK30_PIN1_PB2,        //GPIO index
	.GpsClk = RK30_PIN1_PB4,        //GPIO index
	.GpsVCCEn = RK30_PIN1_PB5,     //GPIO index
	.GpsSpi_CSO = RK30_PIN1_PA4,    //GPIO index
	.GpsSpiClk = RK30_PIN1_PA5,     //GPIO index
	.GpsSpiMOSI = RK30_PIN1_PA7,	  //GPIO index
	.GpsIrq = IRQ_GPS,
	.GpsSpiEn = 0,
	.GpsAdcCh = 2,
	.u32GpsPhyAddr = RK30_GPS_PHYS,
	.u32GpsPhySize = RK30_GPS_SIZE,
};

struct platform_device rk_device_gps = {
	.name = "gps_hv5820b",
	.id = -1,
	.dev		= {
	.platform_data = &rk_gps_info,
		}
	};
#endif

#if defined(CONFIG_MT5931_MT6622)
static struct mt6622_platform_data mt6622_platdata = {
		    .power_gpio         = { // BT_REG_ON
		      #if DS1006H_V1_2_SUPPORT
                    .io             = RK30_PIN3_PC7, // set io to INVALID_GPIO for disable it
		      #else
		    	.io             = RK30_PIN3_PD5, // set io to INVALID_GPIO for disable it
		    	#endif
			    .enable         = GPIO_HIGH,
			    .iomux          = {
				    .name       = NULL,
				},
		    },

		    .reset_gpio         = { // BT_RST
		        #if DS1006H_V1_2_SUPPORT
                    .io             = RK30_PIN3_PD1,
		        #else
		        .io             = RK30_PIN0_PD7,
		        #endif
		        .enable         = GPIO_HIGH,
		        .iomux          = {
		            .name       = NULL,
		        },
		    },

		    .irq_gpio           = {
		          #if DS1006H_V1_2_SUPPORT
                       .io             = RK30_PIN0_PA5,
		          #else
			    .io             = RK30_PIN3_PD2,
			    #endif
			    .enable         = GPIO_HIGH,
			    .iomux          = {
				    .name       = NULL,
				},
		    }
};

static struct platform_device device_mt6622 = {
		    .name   = "mt6622",
			.id     = -1,
			.dev    = {
			       .platform_data = &mt6622_platdata,
			},
};	
#endif

static struct platform_device *devices[] __initdata = {

#ifdef CONFIG_ION
	&device_ion,
#endif
#ifdef CONFIG_WIFI_CONTROL_FUNC
	&rk29sdk_wifi_device,
#endif

#if defined(CONFIG_MT6620)
	    &mt3326_device_gps,
#endif

#ifdef CONFIG_BATTERY_RK30_ADC_FAC
 	&rk30_device_adc_battery,
#endif
#ifdef CONFIG_RFKILL_RK
	&device_rfkill_rk,
#endif
#ifdef CONFIG_GPS_RK
	&rk_device_gps,
#endif
#ifdef CONFIG_MT5931_MT6622
	&device_mt6622,
#endif
#if defined(CONFIG_MT6229)
	&rk29_device_mt6229,
#endif
#if defined(CONFIG_ARCH_RK3188)
	&device_mali,
#endif

};

int rk_platform_add_backlight_device(void)
{
	struct platform_device *bl = NULL; //backlight
	
#ifdef CONFIG_BACKLIGHT_RK29_BL
	bl = &rk29_device_backlight;
#endif

	if(bl)
		platform_device_register(bl);
		
	return 0;
}

static unsigned s_lcd_ok = 0;
static int rk_platform_add_display_devices(void)
{
	struct platform_device *fb = NULL;  //fb
	struct platform_device *lcdc0 = NULL; //lcdc0
	struct platform_device *lcdc1 = NULL; //lcdc1
	struct platform_device *bl = NULL; //backlight
#ifdef CONFIG_FB_ROCKCHIP
	fb = &device_fb;
#endif

#if defined(CONFIG_LCDC0_RK3188) || defined(CONFIG_LCDC0_RK3066B)
	lcdc0 = &device_lcdc0;
#endif

#if defined(CONFIG_LCDC1_RK3188) || defined(CONFIG_LCDC1_RK3066B)
	lcdc1 = &device_lcdc1;
#endif

#ifdef CONFIG_BACKLIGHT_RK29_BL
	bl = &rk29_device_backlight;
#endif

		if(s_lcd_ok == '0') __rk_platform_add_display_devices(fb,lcdc0,lcdc1,bl);
#if defined(CONFIG_MFD_RK616)
	else if(s_lcd_ok == '6') __rk_platform_add_display_devices(fb,lcdc0,lcdc1,NULL);
#endif

	return 0;
	
}

// i2c
#ifdef CONFIG_I2C0_RK30
static struct i2c_board_info __initdata i2c0_info[] = {
#if defined (CONFIG_GS_MMA8452)
	{
		.type	        = "gs_mma8452",
		.addr	        = 0x1d,
		.flags	        = 0,
		.irq	        = MMA8452_INT_PIN,
		.platform_data = &mma8452_info,
	},
#endif

#if defined (CONFIG_GS_LIS3DH)
	{
		.type	        = "gs_lis3dh",
		.addr	        = 0x19,   //0x19(SA0-->VCC), 0x18(SA0-->GND)
		.flags	        = 0,
		.irq	        = LIS3DH_INT_PIN,
		.platform_data = &lis3dh_info,
	},
#endif
#if defined (CONFIG_COMPASS_AK8963)
	{
		.type          = "ak8963",
		.addr          = 0x0d,
		.flags         = 0,
		.irq           = RK30_PIN3_PD7,	
		.platform_data = &akm8963_info,
	},
#endif

#if defined (CONFIG_LS_PHOTORESISTOR)
	{
		.type           = "ls_photoresistor",
		.addr           = 0x5e,            
		.flags          = 0,
		.irq            = INVALID_GPIO,	
		.platform_data = &light_photoresistor_info,
	},
#endif

#if defined (CONFIG_COMPASS_AK8975)
	{
		.type          = "ak8975",
		.addr          = 0x0d,
		.flags         = 0,
		.irq           = RK30_PIN3_PD7,	
		.platform_data = &akm8975_info,
		.irq           = RK30_PIN3_PD7,	
		.platform_data = &akm8975_info,
	},
#endif
#if defined (CONFIG_GYRO_L3G4200D)
	{
		.type          = "l3g4200d_gryo",
		.addr          = 0x69,
		.flags         = 0,
		.irq           = L3G4200D_INT_PIN,
		.platform_data = &l3g4200d_info,
	},
#endif
#if defined (CONFIG_SND_SOC_RK1000)
	{
		.type          = "rk1000_i2c_codec",
		.addr          = 0x60,
		.flags         = 0,
	},
	{
		.type          = "rk1000_control",
		.addr          = 0x40,
		.flags         = 0,
	},
#endif
};
#endif

int __sramdata g_pmic_type =  0;
#ifdef CONFIG_I2C1_RK30
#ifdef CONFIG_REGULATOR_ACT8846
#define PMU_POWER_SLEEP RK30_PIN0_PA1
#define PMU_VSEL RK30_PIN3_PD3
#define ACT8846_HOST_IRQ                RK30_PIN0_PB3

static struct pmu_info  act8846_dcdc_info[] = {
	{
		.name          = "act_dcdc1",   //ddr
		.min_uv          = 1200000,
		.max_uv         = 1200000,
		.suspend_vol  =   1200000,
	},
	{
		.name          = "vdd_core",    //logic
		.min_uv          = 1200000,
		.max_uv         = 1200000,
		#ifdef CONFIG_ACT8846_SUPPORT_RESET
		.suspend_vol  =  1200000,
		#else
		.suspend_vol  =  900000,
		#endif
	},
	{
		.name          = "vdd_cpu",   //arm
		.min_uv          = 1200000,
		.max_uv         = 1200000,
		#ifdef CONFIG_ACT8846_SUPPORT_RESET
		.suspend_vol  =  1200000,
		#else
		.suspend_vol  =  900000,
		#endif
	},
	{
		.name          = "act_dcdc4",   //vccio
		.min_uv          = 3000000,
		.max_uv         = 3000000,
		#ifdef CONFIG_ACT8846_SUPPORT_RESET
		.suspend_vol  =  3000000,
		#else
		.suspend_vol  =  2800000,
		#endif
	},
	
};
static  struct pmu_info  act8846_ldo_info[] = {
	{
		.name          = "act_ldo1",   //vdd10
		.min_uv          = 1000000,
		.max_uv         = 1000000,
	},
	{
		.name          = "act_ldo2",    //vdd12
		.min_uv          = 1200000,
		.max_uv         = 1200000,
	},
	{
		.name          = "act_ldo3",   //vcc18_cif
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "act_ldo4",   //vcca33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "act_ldo5",   //vcctp
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "act_ldo6",   //vcc_jetta
		.min_uv          = 2800000,
		.max_uv         = 2800000,
	},
	{
		.name          = "act_ldo7",   //vcc18
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "act_ldo8",   //vcc28_cif
		.min_uv          = 2800000,
		.max_uv         = 2800000,
	},
 };

#include "../mach-rk30/board-pmu-act8846.c"
#endif

#if defined (CONFIG_KP_AXP22) ///add by wangjian
#include "../../../drivers/power/axp_power/axp22-board.c"

static struct pmu_info  axp228_dcdc_info[] = {
	{
		.name          = "axp22_dcdc1",   //vcc_io 3.0 
		.min_uv          = 3000000,
		.max_uv         = 3000000,
	},
	{
		.name          = "vdd_cpu",//"axp22_dcdc2"// avdd_com 1.1v 
		.min_uv          = 1100000,
		.max_uv          = 1100000,
	},
	{
		.name          = "vdd_core",//"axp22_dcdc3"//vdd_log 1.1v 
		.min_uv          = 1100000,
		.max_uv         = 1100000,
	},
	//{
	//	.name          = "axp_dcdc4",   //not used
	//	.min_uv          = 3000000,
	//	.max_uv         = 3000000,
	//},
	{
		.name          = "axp22_dcdc5",   //vcc_ddr 1.5v
		.min_uv          = 1500000,
		.max_uv         = 1500000,
	},
	
};
static  struct pmu_info  axp228_ldo_info[] = {
	{
		.name          = "axp22_aldo1",   //vcca_33 aldo1
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "axp22_aldo2",    //vcc_card aldo2
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "axp22_aldo3",   //vcc_18 aldo3
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "axp22_dldo1",   //vccio_wl  dldo1
		.min_uv          = 2800000,
		.max_uv         = 2800000,
	},
	{
		.name          = "act_ldo8",//"axp22_dldo2",   //vcc28_cif dldo2
		.min_uv          = 2800000,
		.max_uv         = 2800000,
	},
	{
		.name          = "axp22_dldo3",   //csi_avdd dldo3
		.min_uv          = 2800000,
		.max_uv         = 2800000,
	},
	{
		.name          = "axp22_dldo4",   //vcc_jettaa dldo4
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "axp22_eldo1",   //vdd_1v2 eldo1
		.min_uv        = 1500000,// = 1200000,
		.max_uv        = 1500000,//= 1200000,
	},
	{
		.name          = "axp22_eldo2",   //vdd_jetta eldo2
		.min_uv          = 1200000,
		.max_uv         = 1200000,
	},
	{
		.name          = "act_ldo3",//"axp22_eldo3",   //vcc18_cif eldo3
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "axp22_dc5ldo",   //vdd_10 dc5ldo
		.min_uv          = 1000000,
		.max_uv         = 1000000,
	},
	{
		.name          = "axp22_ldoio0",   //VCC_TP gpio0
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
 };

#include "../mach-rk30/board-pmu-axp228.c"
#endif

#ifdef CONFIG_MFD_WM831X_I2C
#define PMU_POWER_SLEEP 		RK30_PIN0_PA1 

static struct pmu_info  wm8326_dcdc_info[] = {
	{
		.name          = "vdd_core",   //logic
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		.suspend_vol  =  950000,
	},
	{
		.name          = "vdd_cpu",    //arm
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		.suspend_vol  =  950000,
	},
	{
		.name          = "dcdc3",   //ddr
		.min_uv          = 1150000,
		.max_uv         = 1150000,
		.suspend_vol  =  1150000,
	},
	#ifdef CONFIG_MACH_RK3066_SDK
	{
		.name          = "dcdc4",   //vcc_io
		.min_uv          = 3300000,
		.max_uv         = 3300000,
		.suspend_vol  =  3000000,
	},
	#else
	{
		.name          = "dcdc4",   //vcc_io
		.min_uv          = 3000000,
		.max_uv         = 3000000,
		.suspend_vol  =  2800000,
	},
	#endif
};

static struct pmu_info  wm8326_ldo_info[] = {
	{
		.name          = "ldo1",   //vcc18_cif
		.min_uv          = 1800000,
		.max_uv         = 1800000,
		.suspend_vol  =  1800000,
	},
	{
		.name          = "ldo2",    //vccio_wl
		.min_uv          = 1800000,
		.max_uv         = 1800000,
		.suspend_vol  =  1800000,
	},
	{
		.name          = "ldo3",   //
		.min_uv          = 1100000,
		.max_uv         = 1100000,
		.suspend_vol  =  1100000,
	},
	{
		.name          = "ldo4",   //vdd11
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		.suspend_vol  =  1000000,
	},
	{
		.name          = "ldo5",   //vcc25
		.min_uv          = 1800000,
		.max_uv         = 1800000,
		.suspend_vol  =  1800000,
	},
	{
		.name          = "ldo6",   //vcc33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
		.suspend_vol  =  3300000,
	},
	{
		.name          = "ldo7",   //vcc28_cif
		.min_uv          = 2800000,
		.max_uv         = 2800000,
		.suspend_vol  =  2800000,
	},
	{
		.name          = "ldo8",   //vcca33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
		.suspend_vol  =  3300000,
	},
	{
		.name          = "ldo9",   //vcc_tp
		.min_uv          = 3300000,
		.max_uv         = 3300000,
		.suspend_vol  =  3300000,
	},
	{
		.name          = "ldo10",   //flash_io
		.min_uv          = 1800000,
		.max_uv         = 1800000,
		.suspend_vol  =  1800000,
	},
};

#include "../mach-rk30/board-pmu-wm8326.c"
#endif

#ifdef CONFIG_MFD_TPS65910
#if defined(CONFIG_ARCH_RK3188) || defined(CONFIG_SOC_RK3168)
#define TPS65910_HOST_IRQ        RK30_PIN0_PB3
#else
#define TPS65910_HOST_IRQ        RK30_PIN6_PA4
#endif

#define PMU_POWER_SLEEP RK30_PIN0_PA1

static struct pmu_info  tps65910_dcdc_info[] = {
	{
		.name          = "vdd_core",   //logic
		.min_uv          = 1200000,
		.max_uv         = 1200000,
	},
	{
		.name          = "vdd_cpu",    //ddr
		.min_uv          = 1200000,
		.max_uv         = 1200000,
	},
	{
		.name          = "vio",   //vcc_io
		.min_uv          = 3000000,
		.max_uv         = 3000000,
	},
	
};
static  struct pmu_info  tps65910_ldo_info[] = {
	/*
	{
		.name          = "vpll",   //vdd10
		.min_uv          = 1000000,
		.max_uv         = 1000000,
	},*/
	{
		.name          = "act_ldo3",    //vcc18_cif
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "vdig2",   //vdd_jetta
		.min_uv          = 1100000,
		.max_uv         = 1100000,
	},
	{
		.name          = "act_ldo8",   //vcc28_cif
		.min_uv          = 2800000,
		.max_uv         = 2800000,
	},
	{
		.name          = "vaux2",   //vcca33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "vaux33",   //vcc_tp
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "vmmc",   //vcc30
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "vdac",   //vcc18
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
 };

#include "../mach-rk30/board-pmu-tps65910.c"
#endif

static struct i2c_board_info __initdata i2c1_info[] = {
#if defined (CONFIG_REGULATOR_ACT8846)
	{
		.type    		= "act8846",
		.addr           = 0x5a, 
		.flags			= 0,
		.irq            = ACT8846_HOST_IRQ,
		.platform_data=&act8846_data,
	},
#endif
#if defined (CONFIG_RTC_HYM8563)
	{
		.type                   = "rtc_hym8563",
		.addr           = 0x51,
		.flags                  = 0,
		.irq            = RK30_PIN0_PB5,
	},
#endif
#if defined (CONFIG_MFD_WM831X_I2C)
	{
		.type          = "wm8326",
		.addr          = 0x34,
		.flags         = 0,
		.irq           = RK30_PIN0_PB3,
		.platform_data = &wm831x_platdata,
	},
#endif
#if defined (CONFIG_MFD_TPS65910)
	{
        .type           = "tps65910",
        .addr           = TPS65910_I2C_ID0,
        .flags          = 0,
        .irq            = TPS65910_HOST_IRQ,
    	.platform_data = &tps65910_data,
	},
#endif
};
#endif

void __sramfunc board_pmu_suspend(void)
{      
        #if defined (CONFIG_REGULATOR_ACT8846)
       if(pmic_is_act8846())
       board_pmu_act8846_suspend(); 
       #endif
	#if defined (CONFIG_MFD_WM831X_I2C)
       if(pmic_is_wm8326())
       board_pmu_wm8326_suspend();
	#endif
	#if defined (CONFIG_MFD_TPS65910)
       if(pmic_is_tps65910())
       board_pmu_tps65910_suspend(); 
    #endif   
}

void __sramfunc board_pmu_resume(void)
{      
        #if defined (CONFIG_REGULATOR_ACT8846)
       if(pmic_is_act8846())
       board_pmu_act8846_resume(); 
       #endif
	#if defined (CONFIG_MFD_WM831X_I2C)
       if(pmic_is_wm8326())
       board_pmu_wm8326_resume();
	#endif
	#if defined (CONFIG_MFD_TPS65910)
       if(pmic_is_tps65910())
       board_pmu_tps65910_resume(); 
	#endif
}

 int __sramdata gpio3d6_iomux,gpio3d6_do,gpio3d6_dir,gpio3d6_en;

#define grf_readl(offset)	readl_relaxed(RK30_GRF_BASE + offset)
#define grf_writel(v, offset)	do { writel_relaxed(v, RK30_GRF_BASE + offset); dsb(); } while (0)
 
void __sramfunc rk30_pwm_logic_suspend_voltage(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR

//	int gpio0d7_iomux,gpio0d7_do,gpio0d7_dir,gpio0d7_en;
	sram_udelay(10000);
	gpio3d6_iomux = grf_readl(GRF_GPIO3D_IOMUX);
	gpio3d6_do = grf_readl(GRF_GPIO3H_DO);
	gpio3d6_dir = grf_readl(GRF_GPIO3H_DIR);
	gpio3d6_en = grf_readl(GRF_GPIO3H_EN);

	grf_writel((1<<28), GRF_GPIO3D_IOMUX);
	grf_writel((1<<30)|(1<<14), GRF_GPIO3H_DIR);
	grf_writel((1<<30)|(1<<14), GRF_GPIO3H_DO);
	grf_writel((1<<30)|(1<<14), GRF_GPIO3H_EN);
#endif 
}
void __sramfunc rk30_pwm_logic_resume_voltage(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	grf_writel((1<<28)|gpio3d6_iomux, GRF_GPIO3D_IOMUX);
	grf_writel((1<<30)|gpio3d6_en, GRF_GPIO3H_EN);
	grf_writel((1<<30)|gpio3d6_dir, GRF_GPIO3H_DIR);
	grf_writel((1<<30)|gpio3d6_do, GRF_GPIO3H_DO);
	sram_udelay(10000);

#endif

}
extern void pwm_suspend_voltage(void);
extern void pwm_resume_voltage(void);
void  rk30_pwm_suspend_voltage_set(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	pwm_suspend_voltage();
#endif
}
void  rk30_pwm_resume_voltage_set(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	pwm_resume_voltage();
#endif
}


#ifdef CONFIG_I2C2_RK30
static struct i2c_board_info __initdata i2c2_info[] = {
#if defined (CONFIG_MACH_RK_YF)
	{
		.type          = "Goodix-TS",
		.addr          = 0x5d,
		.flags         = 0,
		//.irq           = TOUCH_INT_PIN,
		.platform_data = &goodix_info,
	},
#endif
#if defined (CONFIG_CT36X_TS)
	{
		.type	       = CT36X_NAME,
		.addr          = 0x01,
		.flags         = 0,
		.platform_data = &ct36x_info,
	},
#endif
#if defined (CONFIG_LS_CM3217)
	{
		.type          = "lightsensor",
		.addr          = 0x10,
		.flags         = 0,
		.platform_data = &cm3217_info,
	},
#endif
#if defined (CONFIG_LS_US5151)
        {    
                .type           = "us5151",
                .addr           = 0x10,
                .flags          = 0, 
        },   
#endif

#if defined(CONFIG_HDMI_CAT66121)
	{
		.type		= "cat66121_hdmi",
		.addr		= 0x4c,
		.flags		= 0,
		.irq		= RK30_PIN2_PD6,
		.platform_data 	= &rk_hdmi_pdata,
	},
#endif
};
#endif

#ifdef CONFIG_I2C3_RK30
static struct i2c_board_info __initdata i2c3_info[] = {
};
#endif

#ifdef CONFIG_I2C4_RK30
static struct i2c_board_info __initdata i2c4_info[] = {
#ifdef CONFIG_MFD_RK610
		{
			.type			= "rk610_ctl",
			.addr			= 0x40,
			.flags			= 0,
			.platform_data		= &rk610_ctl_pdata,
		},
#ifdef CONFIG_RK610_TVOUT
		{
			.type			= "rk610_tvout",
			.addr			= 0x42,
			.flags			= 0,
		},
#endif
#ifdef CONFIG_HDMI_RK610
		{
			.type			= "rk610_hdmi",
			.addr			= 0x46,
			.flags			= 0,
			.irq			= INVALID_GPIO,
		},
#endif
#ifdef CONFIG_SND_SOC_RK610
		{//RK610_CODEC addr  from 0x60 to 0x80 (0x60~0x80)
			.type			= "rk610_i2c_codec",
			.addr			= 0x60,
			.flags			= 0,
			.platform_data		= &rk610_codec_pdata,			
		},
#endif
#endif
#if defined (CONFIG_MFD_RK616)
	{
		.type	       = "rk616",
		.addr	       = 0x50,
		.flags	       = 0,
		.platform_data = &rk616_pdata,
	},
#endif

#if defined (CONFIG_SND_SOC_RT5631)
        {
                .type                   = "rt5631",
                .addr                   = 0x1a,
                .flags                  = 0,
        },
#endif

};
#endif

#ifdef CONFIG_I2C_GPIO_RK30
#define I2C_SDA_PIN     INVALID_GPIO// RK30_PIN2_PD6   //set sda_pin here
#define I2C_SCL_PIN     INVALID_GPIO//RK30_PIN2_PD7   //set scl_pin here
static int rk30_i2c_io_init(void)
{
        //set iomux (gpio) here
        //rk30_mux_api_set(GPIO2D7_I2C1SCL_NAME, GPIO2D_GPIO2D7);
        //rk30_mux_api_set(GPIO2D6_I2C1SDA_NAME, GPIO2D_GPIO2D6);

        return 0;
}
struct i2c_gpio_platform_data default_i2c_gpio_data = {
       .sda_pin = I2C_SDA_PIN,
       .scl_pin = I2C_SCL_PIN,
       .udelay = 5, // clk = 500/udelay = 100Khz
       .timeout = 100,//msecs_to_jiffies(100),
       .bus_num    = 5,
       .io_init = rk30_i2c_io_init,
};
static struct i2c_board_info __initdata i2c_gpio_info[] = {
};
#endif

static void __init rk30_i2c_register_board_info(void)
{
#ifdef CONFIG_I2C0_RK30
	i2c_register_board_info(0, i2c0_info, ARRAY_SIZE(i2c0_info));
#endif
#ifdef CONFIG_I2C1_RK30
	i2c_register_board_info(1, i2c1_info, ARRAY_SIZE(i2c1_info));
#endif
#if defined (CONFIG_KP_AXP22)
	if (env_get_u32("axp22_supproted", 0)) {
		axp22_board_init();
	}
#endif
#ifdef CONFIG_I2C2_RK30
	i2c_register_board_info(2, i2c2_info, ARRAY_SIZE(i2c2_info));
#endif
#ifdef CONFIG_I2C3_RK30
	i2c_register_board_info(3, i2c3_info, ARRAY_SIZE(i2c3_info));
#endif
#ifdef CONFIG_I2C4_RK30
	i2c_register_board_info(4, i2c4_info, ARRAY_SIZE(i2c4_info));
#endif
#ifdef CONFIG_I2C_GPIO_RK30
	i2c_register_board_info(5, i2c_gpio_info, ARRAY_SIZE(i2c_gpio_info));
#endif
}
//end of i2c

#define POWER_ON_PIN RK30_PIN0_PA0   //power_hold
#define GPIO_5V_DRV		(env_get_u32("vcc_5v_ctrl", RK30_PIN0_PA3))    //5v for otg host && hdmi
#define CHARGE_PIN s_charge_gpio     //charge level pin
#define POWER_IND_PIN s_power_ind_gpio   //power indicator
static int s_power_ind_gpio = INVALID_GPIO;
static int s_charge_gpio = INVALID_GPIO;
static int s_charge_pol = 0;
int charge_mode = 0;

static void rk30_charge_mode(int on)
{
	charge_mode = on;
	if(CHARGE_PIN != INVALID_GPIO) {
		on = (s_charge_pol == 0) ^ (on != 0);
		gpio_direction_output(CHARGE_PIN, on ? 1 : 0);
	}
}

int lcd_supported(char * name);
static void rk30_pm_restart(char mode, const char *cmd)
{
	printk(KERN_ERR "rk30_pm_restart start...\n");
	gpio_direction_output(LCD_EN_PIN, !LCD_EN_VALUE);
	if(s_lcd_en2 != INVALID_GPIO) {
		gpio_direction_output(s_lcd_en2, GPIO_HIGH);
	}
	if(lcd_supported("anx6345")) {
		gpio_direction_output(EDP_RST_PIN, GPIO_LOW);
		gpio_direction_output(DVDD18_EN_PIN, DVDD18_EN_VALUE);
		gpio_direction_output(DVDD33_EN_PIN, DVDD33_EN_VALUE);
	}
	arm_machine_restart(mode, cmd);
}

static void rk30_pm_power_off(void)
{
	printk(KERN_ERR "rk30_pm_power_off start...\n");
	gpio_direction_output(GPIO_5V_DRV, GPIO_LOW);
	gpio_request(LCD_EN_PIN, NULL);
	gpio_direction_output(LCD_EN_PIN, !LCD_EN_VALUE);
	if(lcd_supported("anx6345")) {
		gpio_direction_output(EDP_RST_PIN, GPIO_LOW);
		gpio_direction_output(DVDD18_EN_PIN, DVDD18_EN_VALUE);
		gpio_direction_output(DVDD33_EN_PIN, DVDD33_EN_VALUE);
	}
	if(POWER_IND_PIN != INVALID_GPIO) {
		gpio_direction_output(POWER_IND_PIN, GPIO_HIGH);
	}
#if defined(CONFIG_MFD_WM831X)
	wm831x_set_bits(Wm831x,WM831X_GPIO_LEVEL,0x0001,0x0000);  //set sys_pwr 0
	wm831x_device_shutdown(Wm831x);//wm8326 shutdown
#endif
#if defined (CONFIG_KP_AXP22)
	if (pmic_is_axp228()) {
		void yftech_axp_power_off(void);
		yftech_axp_power_off();
		return;
	}
#endif
#if defined(CONFIG_REGULATOR_ACT8846)
       if (pmic_is_act8846()) {
               printk("enter dcdet===========\n");
               if(gpio_get_value (RK30_PIN0_PB2) == GPIO_LOW)
               {
                       printk("enter restart===========\n");
                       arm_pm_restart(0, "charge");
               }
		/** code here may cause tablet cannot boot when shutdown without charger pluged in
		  * and then plug in charger. -- Cody Xie
               else
		{
			act8846_device_shutdown();
		}
		  */
       }
#endif
	#if defined(CONFIG_MFD_TPS65910)
	if(g_pmic_type == PMIC_TYPE_TPS65910)
	{
		tps65910_device_shutdown();//tps65910 shutdown
	}
	#endif
	gpio_direction_output(POWER_ON_PIN, GPIO_LOW);
	while (1);
}


int pmu_data_read(int index)
{
#if defined (CONFIG_REGULATOR_ACT8846)
	if(pmic_is_act8846()) {
		return act8846_data_read((u8)index);
	}
#endif
#if defined (CONFIG_REGULATOR_TPS65910)
	if(pmic_is_tps65910()) {
		return tps65910_data_read((u8)index);
	}
#endif
#if defined (CONFIG_KP_AXP22)
	if (pmic_is_axp228()) {
		extern int rk_get_system_battery_capacity(void);
		int cap = rk_get_system_battery_capacity();
		return (cap > 1) ? cap : 1;
	}
#endif
	return -ENODEV;
}

int pmu_data_write(int index, u8 value)
{
#if defined (CONFIG_REGULATOR_ACT8846)
	if(pmic_is_act8846()) {
		return act8846_data_write((u8)index, value);
	}
#endif
#if defined (CONFIG_REGULATOR_TPS65910)
	if(pmic_is_tps65910()) {
		return tps65910_data_write((u8)index, value);
	}
#endif
#if defined (CONFIG_KP_AXP22)
	if (pmic_is_axp228()) {
		return 0;
	}
#endif
	return -ENODEV;
}

static unsigned s_acc_dir = -1;
static char * s_acc_name;
static int s_acc_off[3];
static int s_acc_units;
//for calibration
static int s_acc_cal = -1;
static int s_acc_val[3];
static int s_acc_count;

int acc_supported(char * name)
{
	int supported = 0;
	if(s_acc_dir == -1) {
		const char * accs = env_get_str("acc_supproted", 0);
		supported = !accs || strstr(accs, name);
	}
	return supported;
}

void acc_register(char * name, int units)
{
	if(s_acc_dir == -1) {
		char dir_name[32] = "acc_dir_";
		strcat(dir_name, name);
		if(env_cpy_u32s(dir_name, &s_acc_dir, 1) != 1) {
			s_acc_dir = env_get_u32("acc_dir", -1);
		}
		if(s_acc_dir == -1) {
			s_acc_dir = 0;
		}
		s_acc_name = name;
		s_acc_units = units;
		printk("acc %s registered,dir %d ,units %d\n", name, s_acc_dir, s_acc_units);
	}
}

void acc_report(struct input_dev *input, int x, int y, int z)
{
	int t;
	if(s_acc_cal < 0) return;
	switch(s_acc_dir) {
		case 1:
			t = x;
			x = -y;
			y = t;
			break;
		case 2:
			x = -x;
			y = -y;
			break;
		case 3:
			t = x;
			x = y;
			y = -t;
			break;
		case 4:
			x = -x;
			z = -z;
			break;
		case 5:
			t = x;
			x = y;
			y = t;
			z = -z;
			break;
		case 6:
			y = -y;
			z = -z;
			break;
		case 7:
			t = x;
			x = -y;
			y = -t;
			z = -z;
			break;
	}
	if(s_acc_cal) {
		s_acc_val[0] += x;
		s_acc_val[1] += y;
		s_acc_val[2] += z;
		if(s_acc_cal == 1) {
			s_acc_val[0] /= s_acc_count;
			s_acc_val[1] /= s_acc_count;
			s_acc_val[2] /= s_acc_count;
			//always update new off
			s_acc_off[0] = -s_acc_val[0];
			s_acc_off[1] = -s_acc_val[1];
			s_acc_off[2] = s_acc_units - s_acc_val[2];
			printk("acc new offset %d %d %d\n", s_acc_off[0], s_acc_off[1], s_acc_off[2]);
			if(sys_data_write(SYS_DATA_MARK, sizeof(s_acc_off), s_acc_off) == sizeof(s_acc_off)) {
				s_acc_count = 0;
			}
			else {
				printk("udpate new offset fail\n");
				s_acc_count = -1;
			}
		}
		s_acc_cal--;
	}
	x += s_acc_off[0];
	y += s_acc_off[1];
	z += s_acc_off[2];
	input_report_abs(input, ABS_X, x);
	input_report_abs(input, ABS_Y, y);
	input_report_abs(input, ABS_Z, z);
	input_sync(input);
}

#include <linux/proc_fs.h>

static int charger_read(char *page, char **start, off_t off,int count, int *eof, void *data)
{
	int value = pmu_data_read(PMU_BAT_CAP);
	sprintf(page, "%d\n", ((value <= 0 || (value & 0x80)) ? 1 : 0));
	return 2;
}

static int charger_write(struct file *file, const char *buffer,unsigned long count, void *data)
{
	if(count && buffer) {
		int value = *buffer == '1';
		printk("set charge mode %d\n", value);
		rk30_charge_mode(value);
#if defined (CONFIG_KP_AXP22)
		if (pmic_is_axp228()) {
			extern void yftech_charger_mode(bool enable);
			yftech_charger_mode(!!value);
		}
#endif
	}
	return count;
}

static int acc_info_read(char *page, char **start, off_t off,int count, int *eof, void *data)
{
	if(s_acc_name) {
		if(s_acc_cal == -1) {
			int buf[3];
			int len = sys_data_read(SYS_DATA_MARK, sizeof(buf), buf);
			if(len == sizeof(buf)) {
				memcpy(s_acc_off, buf, sizeof(buf));
				printk("acc offset %d %d %d\n", s_acc_off[0], s_acc_off[1], s_acc_off[2]);
			}
			s_acc_cal = 0;
		}
		count = sprintf(page, "name:%s\nunits:%d\ndir:%d\noffset:%d %d %d\n",
		s_acc_name, s_acc_units, s_acc_dir,
		s_acc_off[0], s_acc_off[1], s_acc_off[2]);
	}
	else {
		count = -1;
	}
	return count;
}

static int acc_info_write(struct file *file, const char *buffer,unsigned long count, void *data)
{
	if(count && buffer && s_acc_dir != -1) {
		s_acc_dir = *buffer - '0';
		printk("set acc direction to %d\n", s_acc_dir);
	}
	else {
		count = -1;
	}
	return count;
}

static int acc_cal_read(char *page, char **start, off_t off,int count, int *eof, void *data)
{
	return sprintf(page, "%d\n", s_acc_cal ? s_acc_cal : s_acc_count);
}

static int acc_cal_write(struct file *file, const char *buffer,unsigned long count, void *data)
{
	int ret = -1;
	if(count && buffer && s_acc_cal == 0) {
		int temp = simple_strtol(buffer, NULL, 0);
		if(temp > 0) {
			printk("acc calibrating %d\n", temp);
			s_acc_val[0] = s_acc_val[1] = s_acc_val[2] = 0;
			s_acc_cal = s_acc_count = temp;
			ret = count;
		}
	}
	return ret;
}

static int backlight_read(char *page, char **start, off_t off,int count, int *eof, void *data)
{
	//int value = gpio_get_value(CHARGE_PIN);
	int value = gpio_get_value(BL_EN_PIN);
	sprintf(page, "%d\n", value ? 1 : 0);
	return 2;
}

static int backlight_write(struct file *file, const char *buffer,unsigned long count, void *data)
{
	if(count && buffer && !s_bl_suspend) {
		int value = *buffer == '1';
		if(value) {
			gpio_set_value(BL_EN_PIN, BL_EN_VALUE);
		}
		else {
			gpio_set_value(BL_EN_PIN, !BL_EN_VALUE);
		}
	}
	return count;
}


static void __init misc_setup(void)
{
	rk29_bl_info.min_brightness = env_get_u32("lcd_bl_min", 65);
	rk29_bl_info.max_brightness = env_get_u32("lcd_bl_max", 235);
	rk29_bl_info.brightness_mode = env_get_u32("lcd_bl_mode", BRIGHTNESS_MODE_LINE);
	rk29_bl_info.pre_div = env_get_u32("lcd_pwm_clock", 20000);
	rk29_bl_info.bl_ref = env_get_u32("lcd_pwm_pol", PWM_EFFECT_VALUE);
	rk29_bl_info.pwm_id = env_get_u32("lcd_pwm_id", PWM_ID);
}

//android init
static char * s_initrc_buffer = NULL;
static int s_initrc_length = 0;
static int s_buffer_length = 0;

static ssize_t init_rc_read(struct file *file, char __user *buf,
		      size_t len, loff_t * offset)
{
	printk("read init rc\n");
	return simple_read_from_buffer(buf, len, offset,s_initrc_buffer,s_initrc_length);
}

//should support read_file(can't use seq_xxx)
static const struct file_operations init_rc_proc_fops = {
	.owner = THIS_MODULE,
	.read = init_rc_read,
};

static int __init on_init_rc(char * name,char * value,void * param)
{
	int value_length = strlen(value);
	if(s_initrc_length + 1 + value_length > s_buffer_length) {
		s_initrc_buffer = krealloc(s_initrc_buffer, s_buffer_length+4096, GFP_KERNEL);
		if(s_initrc_buffer == NULL) {
			printk("memory out for init_rc");
			return -ENOMEM;
		}
		s_buffer_length += 4096;
	}
	memcpy(s_initrc_buffer + s_initrc_length, value, value_length);
	s_initrc_length += value_length;
	s_initrc_buffer[s_initrc_length++] = '\n';
	return 0;
}

static void __init proc_setup(void)
{
	struct proc_dir_entry * entry;
	entry = create_proc_entry("charger", S_IRUGO|S_IWUGO, 0);
	if (entry) {
		entry->read_proc = charger_read;
		entry->write_proc = charger_write;
	} else {
		printk("create proc charger failed");
	}

	entry = create_proc_entry("acc_info", S_IRUGO|S_IWUGO, 0);
	if (entry) {
		entry->read_proc = acc_info_read;
		entry->write_proc = acc_info_write;
	} else {
		printk("create proc acc_info failed");
	}
	entry = create_proc_entry("acc_cal", S_IRUGO|S_IWUGO, 0);
	if (entry) {
		entry->read_proc = acc_cal_read;
		entry->write_proc = acc_cal_write;
	} else {
		printk("create proc acc_cal failed");
	}
	entry = proc_create("init.rc", 0, NULL, &init_rc_proc_fops);
	if (entry) {
		env_enum_str("init",NULL,on_init_rc);
		entry->size = s_initrc_length;
	} else {
		printk("create proc init.rc failed");
	}
	entry = create_proc_entry("backlight", S_IRUGO|S_IWUGO, 0);
	if (entry) {
		entry->read_proc = backlight_read;
		entry->write_proc = backlight_write;
	} else {
		printk("create proc backlight failed");
	}
}

int lcd_supported(char * name)
{
	const char * lcds = env_get_str("lcd_supproted", "rk61x");
	if(!strcmp(lcds, name)) {
		return 1;
	}
	if(!memcmp(lcds, name, 4) && !strcmp(lcds, "rk61x")) {
		return name[4] == s_lcd_ok;
	}
	if(!memcmp(name, "rkhdmi61", 8)) {
		return name[8] == s_lcd_ok;
	}
	return 0;
}

void codec610_set_spk(bool on);
void codec616_set_spk(bool on);
void codec_set_spk(bool on)
{
	if(s_lcd_ok == '0') codec610_set_spk(on);
#if defined(CONFIG_MFD_RK616)
	else if(s_lcd_ok == '6') codec616_set_spk(on);
#endif
}
static void __init machine_rk30_board_init(void)
{
	u32 usb_detect;
	//avs_init();
	gpio_request(POWER_ON_PIN, "poweronpin");
	gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);

	gpio_request(GPIO_5V_DRV, "5vdrv");
	gpio_direction_output(GPIO_5V_DRV, GPIO_HIGH);

	CHARGE_PIN = env_get_u32("chargepin", CHARGE_PIN);
	s_charge_pol = env_get_u32("power_charge_pol", 0);
	if(CHARGE_PIN != INVALID_GPIO) {
		gpio_request(CHARGE_PIN, "chargepin");
		rk30_charge_mode(0);
	}

	POWER_IND_PIN = env_get_u32("power_indicator_gpio", POWER_IND_PIN);
	if(POWER_IND_PIN != INVALID_GPIO) {
		gpio_request(POWER_IND_PIN, "powerindpin");
		gpio_direction_output(POWER_IND_PIN, GPIO_LOW);
	}

#if defined(CONFIG_MACH_RK_YF)
	s_touch_reset = env_get_u32("ctp_reset_gpio", RK30_PIN0_PB6);
	s_touch_int = env_get_u32("ctp_int_gpio", RK30_PIN1_PB7);
	goodix_info.irq_pin = TOUCH_INT_PIN;
	goodix_info.rest_pin = TOUCH_RESET_PIN;
	i2c2_info[0].irq = TOUCH_INT_PIN;
#endif
#if defined(CONFIG_MFD_RK616)
	rk616_pdata.spk_ctl_gpio = env_get_u32("rk616_speak_ctl", RK30_PIN2_PD7);
	rk616_pdata.hp_ctl_gpio = env_get_u32("rk616_hp_ctl", RK30_PIN2_PD7);
#endif
#ifdef CONFIG_SSD2828_RGB2MIPI
	
	lcdc1_screen_info.lcd_info = &rk29_lcd_info;
	platform_device_register(&device_ssd2828);

#endif

#ifdef CONFIG_DP_ANX6345
	if(lcd_supported("anx6345")) {
		s_lcd_en = INVALID_GPIO;
		i2c_register_board_info(2, &i2c_anx6345_info, 1);
	}
#endif
	s_lcd_en2 = env_get_u32("lcd_en2_pgio", s_lcd_en2);
	if(s_lcd_en2 == s_lcd_en) {
		s_lcd_en = INVALID_GPIO;
	}
	pm_power_off = rk30_pm_power_off;
	arm_pm_restart = rk30_pm_restart;
	
        gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);


	rk30_i2c_register_board_info();
	spi_register_board_info(board_spi_devices, ARRAY_SIZE(board_spi_devices));
	platform_add_devices(devices, ARRAY_SIZE(devices));

	usb_detect = env_get_u32("power_usb_detect", INVALID_GPIO);
	if(usb_detect != INVALID_GPIO) {
		board_usb_detect_init(usb_detect);
	}

	default_sdmmc0_data.det_pin_info.enable = env_get_u32("mmc_cd_polarity", default_sdmmc0_data.det_pin_info.enable);

#ifdef CONFIG_WIFI_CONTROL_FUNC
	rk29sdk_wifi_bt_gpio_control_init();
#endif

#if defined(CONFIG_MT6620)
	    clk_set_rate(clk_get_sys("rk_serial.1", "uart"), 48*1000000);
#endif

#if defined(CONFIG_MT5931_MT6622)
		clk_set_rate(clk_get_sys("rk_serial.0", "uart"), 24*1000000);
#endif		
	misc_setup();
	proc_setup();
}

extern void yf_camera_init(void);
#define HD_SCREEN_SIZE 1920UL*1200UL*4*3
static void __init rk30_reserve(void)
{
#if defined(CONFIG_ARCH_RK3188)
	/*if lcd resolution great than or equal to 1920*1200,reserve the ump memory */
	if(!(get_fb_size() < ALIGN(HD_SCREEN_SIZE,SZ_1M)))
	{
		int ump_mem_phy_size=384UL*1024UL*1024UL;
		resource_mali[0].start = board_mem_reserve_add("ump buf", ump_mem_phy_size); 
		resource_mali[0].end = resource_mali[0].start + ump_mem_phy_size -1;
	}
#endif
#ifdef CONFIG_ION
	rk30_ion_pdata.heaps[0].base = board_mem_reserve_add("ion", ION_RESERVE_SIZE);
#endif

#ifdef CONFIG_FB_ROCKCHIP
	resource_fb[0].start = board_mem_reserve_add("fb0 buf", get_fb_size());
	resource_fb[0].end = resource_fb[0].start + get_fb_size()- 1;
#if 0
	resource_fb[1].start = board_mem_reserve_add("ipp buf", RK30_FB0_MEM_SIZE);
	resource_fb[1].end = resource_fb[1].start + RK30_FB0_MEM_SIZE - 1;
#endif

if(env_get_u32("lcd_ipp_rotation", 0)) {
	resource_fb[2].start = board_mem_reserve_add("fb2 buf",get_fb_size());
	resource_fb[2].end = resource_fb[2].start + get_fb_size() - 1;
}
#endif


#ifdef CONFIG_VIDEO_RK29
	yf_camera_init();
	rk30_camera_request_reserve_mem();
#endif
	
#ifdef CONFIG_GPS_RK
	//it must be more than 8MB
	rk_gps_info.u32MemoryPhyAddr = board_mem_reserve_add("gps", SZ_8M);
#endif
	board_mem_reserved();
}
/******************************** arm dvfs frequency volt table **********************************/
/**
 * dvfs_cpu_logic_table: table for arm and logic dvfs 
 * @frequency	: arm frequency
 * @cpu_volt	: arm voltage depend on frequency
 */
#if defined(CONFIG_ARCH_RK3188)
#if 0
//sdk
static struct cpufreq_frequency_table dvfs_arm_table_volt_level0[] = {
        {.frequency = 312 * 1000,       .index = 850 * 1000},
        {.frequency = 504 * 1000,       .index = 900 * 1000},
        {.frequency = 816 * 1000,       .index = 950 * 1000},
        {.frequency = 1008 * 1000,      .index = 1025 * 1000},
        {.frequency = 1200 * 1000,      .index = 1100 * 1000},
        {.frequency = 1416 * 1000,      .index = 1200 * 1000},
        {.frequency = 1608 * 1000,      .index = 1300 * 1000},
        {.frequency = CPUFREQ_TABLE_END},
};
//default
static struct cpufreq_frequency_table dvfs_arm_table_volt_level1[] = {
        {.frequency = 312 * 1000,       .index = 875 * 1000},
        {.frequency = 504 * 1000,       .index = 925 * 1000},
        {.frequency = 816 * 1000,       .index = 975 * 1000},
        {.frequency = 1008 * 1000,      .index = 1075 * 1000},
        {.frequency = 1200 * 1000,      .index = 1150 * 1000},
        {.frequency = 1416 * 1000,      .index = 1250 * 1000},
        {.frequency = 1608 * 1000,      .index = 1350 * 1000},
        {.frequency = CPUFREQ_TABLE_END},
};
#endif
// ds1006h 10'
static struct cpufreq_frequency_table dvfs_arm_table_volt_level2[] = {
        {.frequency = 312 * 1000,       .index = 925 * 1000},
        {.frequency = 504 * 1000,       .index = 950 * 1000},
        {.frequency = 816 * 1000,       .index = 1000 * 1000},
        {.frequency = 1008 * 1000,      .index = 1075 * 1000},
        {.frequency = 1200 * 1000,      .index = 1200 * 1000},
        {.frequency = 1416 * 1000,      .index = 1250 * 1000},
        {.frequency = 1608 * 1000,      .index = 1350 * 1000},
        {.frequency = CPUFREQ_TABLE_END},
};
//if you board is good for volt quality,select dvfs_arm_table_volt_level0
#define dvfs_arm_table dvfs_arm_table_volt_level2

/******************************** gpu dvfs frequency volt table **********************************/
//sdk
#if 0
static struct cpufreq_frequency_table dvfs_gpu_table_volt_level0[] = {	
        {.frequency = 133 * 1000,       .index = 975 * 1000},//the mininum rate is limited 133M for rk3188
	{.frequency = 200 * 1000,       .index = 975 * 1000},
	{.frequency = 266 * 1000,       .index = 1000 * 1000},
	{.frequency = 300 * 1000,       .index = 1050 * 1000},
	{.frequency = 400 * 1000,       .index = 1100 * 1000},
	{.frequency = 600 * 1000,       .index = 1200 * 1000},
        {.frequency = CPUFREQ_TABLE_END},
};
#endif
//ds1006h 10'
static struct cpufreq_frequency_table dvfs_gpu_table_volt_level1[] = {	
       {.frequency = 133 * 1000,       .index = 975 * 1000},
	{.frequency = 200 * 1000,       .index = 1000 * 1000},
	{.frequency = 266 * 1000,       .index = 1025 * 1000},
	{.frequency = 300 * 1000,       .index = 1050 * 1000},
	{.frequency = 400 * 1000,       .index = 1100 * 1000},
	{.frequency = 600 * 1000,       .index = 1250 * 1000},
        {.frequency = CPUFREQ_TABLE_END},
};

#define dvfs_gpu_table dvfs_gpu_table_volt_level1

static struct cpufreq_frequency_table dvfs_ddr_table_t[] = {
	{.frequency = 200 * 1000 + DDR_FREQ_SUSPEND,    .index = 950 * 1000},
	{.frequency = 460 * 1000 + DDR_FREQ_NORMAL,     .index = 1150 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

/******************************** ddr dvfs frequency volt table **********************************/
static struct cpufreq_frequency_table dvfs_ddr_table_volt_level0[] = {
	{.frequency = 200 * 1000 + DDR_FREQ_SUSPEND,    .index = 950 * 1000},
	//{.frequency = 384 * 1000 + DDR_FREQ_VIDEO,      .index = 1100 * 1000},
	{.frequency = 396 * 1000 + DDR_FREQ_NORMAL,     .index = 1100 * 1000},
	{.frequency = 528 * 1000 + DDR_FREQ_NORMAL,     .index = 1200 * 1000},
	{.frequency = 528 * 1000,     .index = 1200 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

#define dvfs_ddr_table dvfs_ddr_table_volt_level0

#else
/**
 * dvfs_cpu_logic_table: table for arm and logic dvfs
 * @frequency	: arm frequency
 * @cpu_volt	: arm voltage depend on frequency
 * @logic_volt	: logic voltage arm requests depend on frequency
 * comments	: min arm/logic voltage
 */
#ifdef CONFIG_DVFS_WITH_UOC
//chenxing uoc
static struct cpufreq_frequency_table dvfs_arm_table[] = {
	{.frequency = 312 * 1000,       .index = 950 * 1000},
	{.frequency = 504 * 1000,       .index = 1000 * 1000},
	{.frequency = 816 * 1000,       .index = 1050 * 1000},
	{.frequency = 1008 * 1000,      .index = 1125 * 1000},
	{.frequency = 1200 * 1000,      .index = 1200 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_gpu_table[] = {
	{.frequency = 100 * 1000,	.index = 1000 * 1000},
	{.frequency = 200 * 1000,	.index = 1000 * 1000},
	{.frequency = 266 * 1000,	.index = 1050 * 1000},
	//{.frequency = 300 * 1000,	.index = 1050 * 1000},
	{.frequency = 400 * 1000,	.index = 1125 * 1000},
        {.frequency = 600 * 1000,       .index = 1250 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_ddr_table[] = {
	{.frequency = 200 * 1000 + DDR_FREQ_SUSPEND,    .index = 1000 * 1000},
	{.frequency = 300 * 1000 + DDR_FREQ_VIDEO,      .index = 1050 * 1000},
	{.frequency = 400 * 1000 + DDR_FREQ_NORMAL,     .index = 1100 * 1000},
	{.frequency = 400 * 1000,     .index = 1100 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};
#else
//chenliang
static struct cpufreq_frequency_table dvfs_arm_table[] = {
	{.frequency = 312 * 1000,       .index = 950 * 1000},
	{.frequency = 504 * 1000,       .index = 1000 * 1000},
	{.frequency = 816 * 1000,       .index = 1050 * 1000},
	{.frequency = 1008 * 1000,      .index = 1125 * 1000},
	{.frequency = 1200 * 1000,      .index = 1200 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_gpu_table[] = {
	{.frequency = 100 * 1000,	.index = 1000 * 1000},
	{.frequency = 200 * 1000,	.index = 1000 * 1000},
	{.frequency = 266 * 1000,	.index = 1050 * 1000},
//	{.frequency = 300 * 1000,	.index = 1050 * 1000},
	{.frequency = 400 * 1000,	.index = 1125 * 1000},
        {.frequency = 600 * 1000,       .index = 1250 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_ddr_table[] = {
	{.frequency = 200 * 1000 + DDR_FREQ_SUSPEND,    .index = 1000 * 1000},
	{.frequency = 280 * 1000 + DDR_FREQ_VIDEO,      .index = 1050 * 1000},
	{.frequency = 300 * 1000 + DDR_FREQ_NORMAL,     .index = 1075 * 1000},
	{.frequency = 400 * 1000 + DDR_FREQ_NORMAL,     .index = 1150 * 1000},
	{.frequency = 400 * 1000,     .index = 1150 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};
#endif
#endif
/******************************** arm dvfs frequency volt table end **********************************/


#include <mach/ddr.h>
void __init board_ddr_init(void)
{
	int mode;
	int freq = env_get_u32("ddr_max_freq", DDR_FREQ) * 1000 + DDR_FREQ_NORMAL;
	int voltage = 0;
	struct cpufreq_frequency_table * table = dvfs_ddr_table;
	while(table->frequency != CPUFREQ_TABLE_END) {
		mode = table->frequency % 1000;
		if(mode == DDR_FREQ_NORMAL) {
			if(table->frequency >= freq) {
				table->frequency = freq;
				if(voltage == 0) {
					voltage = table->index;
				}
				else {
					table->index = voltage;
				}
				printk("update ddr table %d %d\n", freq, voltage);
			}
		}
		table++;
	}
}
//#define DVFS_CPU_TABLE_SIZE	(ARRAY_SIZE(dvfs_cpu_logic_table))
//static struct cpufreq_frequency_table cpu_dvfs_table[DVFS_CPU_TABLE_SIZE];
//static struct cpufreq_frequency_table dep_cpu2core_table[DVFS_CPU_TABLE_SIZE];

void __init board_clock_init(void)
{
	rk30_clock_data_init(periph_pll_default, codec_pll_default, RK30_CLOCKS_DEFAULT_FLAGS);
	//dvfs_set_arm_logic_volt(dvfs_cpu_logic_table, cpu_dvfs_table, dep_cpu2core_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "cpu"), dvfs_arm_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "gpu"), dvfs_gpu_table);

#if defined(CONFIG_ARCH_RK3188)
	if (rk_pll_flag())
		dvfs_set_freq_volt_table(clk_get(NULL, "ddr"), dvfs_ddr_table_t);
	else
#endif
	{
		board_ddr_init();
		dvfs_set_freq_volt_table(clk_get(NULL, "ddr"), dvfs_ddr_table);
	}
}

static unsigned s_ctp_ok = 0;
int ctp_supported(char * name)
{
	int supported = 0;
	if(!s_ctp_ok) {
		const char * ctps = env_get_str("ctp_supproted", 0);
		supported = !ctps || strstr(ctps, name);
	}
	return supported;
}

void ctp_register(char * name)
{
	if(!s_ctp_ok) {
		s_ctp_ok = 1;
		printk("ctp %s registered\n", name);
	}
}

int rk61x_detect(struct i2c_client *client)
{
	int count = 2;
	char data;
	char * name = client->name;
	if(s_lcd_ok) {
		return -1;
	}
	if(!strcmp(name, "rk610_ctl")) {
		while(count) {
			if(i2c_master_reg8_recv(client, 0, &data, 1, 100*1000) >= 0) break;
			count--;
			msleep(10);
		}
		printk("%s probe count %d\n", name, count);
	}
	if(count == 0) {
		return -1;
	}
	if(!strcmp(name, "rk610_ctl")) {
		s_lcd_ok = '0';
	}
	else {
		s_lcd_ok = '6';
		if(!lcd_supported("rk61x")) {
#if defined(CONFIG_MFD_RK616)
			rk616_pdata.lcd1_func = UNUSED;
#endif
		}
	}
	if(lcd_switch_lcdc() && (s_lcd_ok == '0' || !lcd_supported("rk61x"))) {
		device_lcdc1.dev.platform_data = &lcdc0_screen_info;
		device_lcdc0.dev.platform_data = &lcdc1_screen_info;
	}
	rk_platform_add_display_devices();
	return 0;
}

void rk30_setgpio_resume_board(void)
{
	if(POWER_IND_PIN != INVALID_GPIO) {
		gpio_direction_output(POWER_IND_PIN, GPIO_LOW);
	}
	//gpio_direction_output(GPIO_5V_DRV, GPIO_HIGH);
	rk30_charge_mode(0);
}

void rk30_setgpio_suspend_board(void)
{
	//gpio_direction_output(GPIO_5V_DRV, GPIO_LOW);
	rk30_charge_mode(1);
	if(POWER_IND_PIN != INVALID_GPIO) {
		gpio_direction_output(POWER_IND_PIN, GPIO_HIGH);
	}
}
void __init yftech_fixup(struct machine_desc *desc, struct tag *tags,
			char **cmdline, struct meminfo *mi)
{
	env_fixup();
	rk30_fixup(desc, tags, cmdline, mi);
}

MACHINE_START(RK30, "RK30board")
	.boot_params	= PLAT_PHYS_OFFSET + 0x800,
	.fixup		= yftech_fixup,
	.reserve	= &rk30_reserve,
	.map_io		= rk30_map_io,
	.init_irq	= rk30_init_irq,
	.timer		= &rk30_timer,
	.init_machine	= machine_rk30_board_init,
MACHINE_END
