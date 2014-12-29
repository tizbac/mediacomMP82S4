#ifndef _LCD_YF__
#define _LCD_YF__

#include <linux/delay.h>
#include <mach/gpio.h>
#include <mach/yfmach.h>
#include "../transmitter/mipi_dsi.h"

extern int lcd_supported(char * name);

#define OUT_TYPE        SCREEN_RGB
#define OUT_FACE        OUT_P888

#define OUT_CLK         66666666
#define LCDC_ACLK       500000000
/* Timing */
#define H_PW            32
#define H_BP            80
#define H_VD            env_get_u32("lcd_h_vd", 1280)
#define H_FP            48

#define V_PW            6
#define V_BP            14
#define V_VD            env_get_u32("lcd_v_vd", 800)
#define V_FP            3

#define LCD_WIDTH       320
#define LCD_HEIGHT      180

#define DCLK_POL        1
#define SWAP_RB         0

#ifdef CONFIG_SSD2828_RGB2MIPI
#define mipi_dsi_init(data) 				dsi_set_regs(data, ARRAY_SIZE(data))
#define mipi_dsi_send_dcs_packet(data) 		dsi_send_dcs_packet(data, ARRAY_SIZE(data))
#define mipi_dsi_post_init(data)			dsi_set_regs(data, ARRAY_SIZE(data))
#define data_lane  4
static struct rk29lcd_info *gLcd_info = NULL;
int lcd_init(void);
int lcd_standby(u8 enable);

static unsigned int pre_initialize[] = {
	0x00B10000,
	0x00B20000,
	0x00B30000,
	0x00B40000,
	0x00B50000,
	0x00B60000 | (VPF_24BPP) | (VM_BM << 2),     // burst mode 24bits

	0x00de0000 | (data_lane -1),    //4 lanes
	0x00d60004,

	0x00B90000,
	0x00ba8016,   //pll
	0x00Bb0008,
	0x00B90001,
	0x00c40001,
};

static unsigned int post_initialize[] = {
	0x00B90000,
	
//	0x00ba8006,   //pll
//	0x00Bb0002,	
	0x00B7030b,
	
	0x00B90001,
	0x00B80000,
	0x00BC0000,
	0x00c00100,      //software reset ssd2828
};

static unsigned char mipi_exit_sleep_mode[] = {0x11};
static unsigned char mipi_set_diaplay_on[] = {0x29};
static unsigned char mipi_enter_sleep_mode[] = {0x10};
static unsigned char mipi_set_diaplay_off[] = {0x28};

int lcd_init(void)
{
	dsi_probe_current_chip();
	gpio_direction_output(gLcd_info->reset_pin, 0);
	msleep(10);
	gpio_set_value(gLcd_info->reset_pin, 1);
	msleep(6);
	mipi_dsi_init(pre_initialize);
	msleep(1);
	mipi_dsi_send_dcs_packet(mipi_exit_sleep_mode);
	msleep(100);
	mipi_dsi_send_dcs_packet(mipi_set_diaplay_on);
	msleep(1);
	mipi_dsi_post_init(post_initialize);
    return 0;
}

int lcd_standby(u8 enable)
{
	if(enable) {
		printk("lcd_standby...\n");

		mipi_dsi_send_dcs_packet(mipi_set_diaplay_off);
		msleep(2);
		mipi_dsi_send_dcs_packet(mipi_enter_sleep_mode);
		msleep(100);

		dsi_power_off();
		gpio_set_value(gLcd_info->reset_pin, 0);
	} else {
		dsi_power_up();
		lcd_init();
	}
    return 0;
}
#endif

#ifdef CONFIG_RK616_MIPI_DSI
int rk_lcd_init(void) {
	u8 dcs[16] = {0};
	if(dsi_is_active() != 1)
		return -1;

	dsi_enable_hs_clk(1);

	dcs[0] = LPDT;
	dcs[1] = dcs_exit_sleep_mode;
	dsi_send_dcs_packet(dcs, 2);
	msleep(1);
	dcs[0] = LPDT;
	dcs[1] = dcs_set_display_on;
	dsi_send_dcs_packet(dcs, 2);
	msleep(10);
	//dsi_enable_command_mode(0);
	dsi_enable_video_mode(1);
	return 0;
}

int rk_lcd_standby(u8 enable) {

	u8 dcs[16] = {0};
	if(dsi_is_active() != 1)
		return -1;

	if(enable) {
		/*below is changeable*/
		dcs[0] = LPDT;
		dcs[1] = dcs_set_display_off;
		dsi_send_dcs_packet(dcs, 2);
		msleep(1);
		dcs[0] = LPDT;
		dcs[1] = dcs_enter_sleep_mode;
		dsi_send_dcs_packet(dcs, 2);
		msleep(1);
	} else {
		rk_lcd_init();
	}
	return 0;
}

#endif
#define RK_USE_SCREEN_ID

static void set_lcd_info_by_id(struct rk29fb_screen *screen, struct rk29lcd_info *lcd_info )
{
	int rk610 = lcd_supported("rk61x");
	int ssd2828 = lcd_supported("ssd2828");
	/* screen type & face */
	screen->type = env_get_u32("lcd_out_type", rk610 ? SCREEN_LVDS: OUT_TYPE);
	screen->face = env_get_u32("lcd_out_face", OUT_FACE);
	screen->lvds_format = env_get_u32("lcd_out_format", LVDS_8BIT_2);

	/* Screen size */
	screen->x_res = H_VD;
	screen->y_res = V_VD;

	screen->width = env_get_u32("lcd_width", LCD_WIDTH);
	screen->height = env_get_u32("lcd_height", LCD_HEIGHT);

	/* Timing */
	screen->lcdc_aclk = env_get_u32("lcd_aclk", LCDC_ACLK);
	screen->pixclock = env_get_u32("lcd_clk", OUT_CLK);
	screen->left_margin = env_get_u32("lcd_h_bp", H_BP);
	screen->right_margin = env_get_u32("lcd_h_fp", H_FP);
	screen->hsync_len = env_get_u32("lcd_h_pw", H_PW);
	screen->upper_margin = env_get_u32("lcd_v_bp", V_BP);
	screen->lower_margin = env_get_u32("lcd_v_fp", V_FP);
	screen->vsync_len = env_get_u32("lcd_v_pw", V_PW);

	/* Pin polarity */
	screen->pin_hsync = env_get_u32("lcd_h_pol", 0);
	screen->pin_vsync = env_get_u32("lcd_v_pol", 0);
	screen->pin_den = env_get_u32("lcd_den_pol", 0);
	screen->pin_dclk = env_get_u32("lcd_dclk_pol", DCLK_POL);

	/* Swap rule */
	screen->swap_rb = env_get_u32("lcd_swap_rb", SWAP_RB);
	screen->swap_rg = env_get_u32("lcd_swap_rg", 0);
	screen->swap_gb = env_get_u32("lcd_swap_gb", 0);
	screen->swap_delta = 0;
	screen->swap_dumy = 0;

	if(ssd2828) {
#ifdef CONFIG_SSD2828_RGB2MIPI
		int vpw, hpw, vbp, hbp, vfp, hfp;
		screen->init = lcd_init;
		screen->standby = lcd_standby;
		if(lcd_info) {
			gLcd_info = lcd_info;
			gLcd_info->io_init();
		}
		vpw = screen->vsync_len;
		hpw = screen->hsync_len;
		vbp = screen->upper_margin;
		hbp = screen->left_margin;
		vfp = screen->lower_margin;
		hfp = screen->right_margin;
		pre_initialize[0] = 0x00B10000 | ((vpw & 0Xff) << 8) | (hpw & 0Xff);
		pre_initialize[1] = 0x00B20000 | (((vbp+vpw) & 0Xff) << 8) | ((hbp+hpw) & 0Xff);
		pre_initialize[2] = 0x00B30000 | ((vfp & 0Xff) << 8) | (hfp & 0Xff);
		pre_initialize[3] = 0x00B40000 | screen->x_res;
		pre_initialize[4] = 0x00B50000 | screen->y_res;
#endif
	}
#ifdef CONFIG_RK616_MIPI_DSI
	else if(screen->type == SCREEN_MIPI) {
		screen->dsi_lane = env_get_u32("lcd_dis_lane", 4);
		screen->hs_tx_clk = env_get_u32("lcd_hs_tclk", 528*1000000);
		screen->init = rk_lcd_init;
		screen->standby = rk_lcd_standby;
		if(lcd_info)
			gLcd_info = lcd_info;
	}
#endif
	else {
		screen->init = NULL;
		screen->standby = NULL;
	}
}
#endif
