#ifdef CONFIG_VIDEO_RK29

#include <plat/rk_camera.h>

static char s_camera_name[RK_CAM_NUM][32];
static struct rkcamera_platform_data new_camera[RK_CAM_NUM+1];

static unsigned __init cam_get_u32(char * name, int index, unsigned def)
{
	char ename[64];
	sprintf(ename, "%s%d", name, index);
	return env_get_u32(ename, def);
}

static unsigned __init cam_get_power_seq(const char * name, int index)
{
	unsigned def = strcmp(name, "hm5065") ? sensor_PWRSEQ_DEFAULT : hm5065_PWRSEQ;
	return cam_get_u32("cam_power_seq", index, def);
}

static unsigned __init cam_get_power_info(const char * name, int index)
{
	unsigned def = 1;
	if(!strcmp(name, "ov5640")) def |= 0x01000000; //ldo_18 to 1.5v
	else if(!strcmp(name, "hm5065")) def |= 0x10000000; //ldo_28 to 1.8v
	else if(!strcmp(name, "hm2057")) def |= 0x10000000; //ldo_28 to 1.8v
	//else if(!strcmp(name, "")) def |= 0x00000010; //direct resume
	return cam_get_u32("cam_power_info", index, def);
}

#define WH_IN_RES(w,h) ((w>>4) << 8) | (h >> 4)
static unsigned __init cam_get_resolution(const char * name, int index)
{
	unsigned result;
	unsigned def = 0;
	while(*name >= 'A') name++;
	//just work for yuanfeng sensors
	if(*name == '0') def = 0x30000;
	else if(*name == '5') def = 0x500000;
	else if(*name == '2') def = 0x200000;
	result = cam_get_u32("cam_resolution", index, def);
	printk("camera resolution %s 0x%x\r\n", name, result);
	if(result != def && !(result & 0xFFFF)) {
		if(result == 0x500000) result |= WH_IN_RES(2592, 1944);
		else if(result == 0x200000) result |= WH_IN_RES(1600, 1200);
		else if(result == 0x30000) result |= WH_IN_RES(640, 480);
		else if(result == 0x300000) result |= WH_IN_RES(2048, 1536);
		else if(result == 0x130000) result |= WH_IN_RES(1280, 1024);
		else if(result == 0x100000) result |= WH_IN_RES(1280, 720);
	}
	return result;
}

void __init yf_camera_init(void)
{
	int i, j;
	unsigned power_info;
	const char * cam_name;
	char cfg_name[] = "cam_name0";
	struct rk29camera_gpio_res * gpios;
	rk_camera_device_register_info_t * infos;
	struct rkcamera_platform_data * camera = new_camera;
	struct rkcamera_platform_data end = new_camera_device_end;
	for(i=0; i<2; i++) {
		for(j=i; j<RK_CAM_NUM; j+=2) {
			cfg_name[sizeof(cfg_name) - 2] = '0' + j;
			cam_name = env_get_str(cfg_name, 0);
			if(cam_name == 0) continue;

			sprintf(s_camera_name[j], "%s%d_%s", i ? "front" : "back", j>>1, cam_name);

			gpios = &camera->io;
			gpios->gpio_reset = cam_get_u32("cam_reset", j, INVALID_GPIO);
			gpios->gpio_power = cam_get_u32("cam_power", j, INVALID_GPIO);
			gpios->gpio_powerdown = cam_get_u32("cam_pd", j, INVALID_GPIO);
			gpios->gpio_flash = cam_get_u32("cam_flash", j, INVALID_GPIO);
			gpios->gpio_flag = cam_get_u32("cam_flag", j, 0);

			infos = &camera->dev;
			infos->link_info.module_name = cam_name;
			infos->link_info.bus_id = RK_CAM_PLATFORM_DEV_ID_0;
			infos->link_info.i2c_adapter_id = 3;

			infos->device_info.name = "soc-camera-pdrv";
			infos->device_info.dev.init_name = s_camera_name[j];

			strcpy(infos->i2c_cam_info.type , cam_name);
			infos->i2c_cam_info.addr = (unsigned short) cam_get_u32("cam_addr", j, 0);

			camera->orientation = cam_get_u32("cam_orient", j, 0);
			camera->resolution = cam_get_resolution(cam_name, j);
			camera->mirror = cam_get_u32("cam_mirror", j, 0);
			camera->i2c_rate = cam_get_u32("cam_i2c_rate", j, 100000);
			camera->flash = cam_get_u32("cam_flash", j, 0);
			camera->mclk_rate = cam_get_u32("cam_mclk_rate", j, 24);

			camera->powerup_sequence = cam_get_power_seq(cam_name, i);
			power_info = cam_get_power_info(cam_name, i);

			camera->pwdn_info = power_info & 0x0000FFFF;
			gpios->gpio_flag |= power_info & 0xFFFF0000;

			camera++;
		}
	}
	memcpy(camera, &end, sizeof(*camera));
}

#endif  //#ifdef CONFIG_VIDEO_RK29

/*---------------- Camera Sensor Configuration Macro End------------------------*/
#include "../../../drivers/media/video/rk30_camera.c"
/*---------------- Camera Sensor Macro Define End  ---------*/

#define PMEM_CAM_SIZE PMEM_CAM_NECESSARY
/*****************************************************************************************
 * camera  devices
 * author: ddl@rock-chips.com
 *****************************************************************************************/
#ifdef CONFIG_VIDEO_RK29
#define CONFIG_SENSOR_POWER_IOCTL_USR	   1 //define this refer to your board layout
#define CONFIG_SENSOR_RESET_IOCTL_USR	   0
#define CONFIG_SENSOR_POWERDOWN_IOCTL_USR	   0
#define CONFIG_SENSOR_FLASH_IOCTL_USR	   0

#if CONFIG_SENSOR_POWER_IOCTL_USR
static int sensor_power_usr_cb (struct rk29camera_gpio_res *res, int on)
{
	struct regulator *ldo_18,*ldo_28,*avdd;
	avdd = pmic_is_axp228() ? regulator_get(NULL, "axp22_dldo3") : NULL;
	ldo_28 = regulator_get(NULL, "act_ldo8");	// vcc28_cif
	ldo_18 = regulator_get(NULL, "act_ldo3");	// vcc18_cif
	if (ldo_28 == NULL || IS_ERR(ldo_28) || ldo_18 == NULL || IS_ERR(ldo_18)){
		printk("get cif ldo failed!\n");
		return -1;
	}
	if(on == 0){
		if (!IS_ERR_OR_NULL(avdd)) {
			while(regulator_is_enabled(avdd)) regulator_disable(avdd);
			regulator_put(avdd);
		}
		while(regulator_is_enabled(ldo_28)) regulator_disable(ldo_28);
		regulator_put(ldo_28);
		while(regulator_is_enabled(ldo_18)) regulator_disable(ldo_18);
		regulator_put(ldo_18);
	}
	else{
		int vol = res->gpio_flag & 0x10000000 ? 1800000 : 2800000;
		regulator_set_voltage(ldo_28, vol, vol);
		regulator_enable(ldo_28);
		regulator_put(ldo_28);
        if (!IS_ERR_OR_NULL(avdd)) {
			regulator_set_voltage(avdd, 2800000, 2800000);
			regulator_enable(avdd);
			regulator_put(avdd);
		}
        msleep(30);
		vol = res->gpio_flag & 0x01000000 ? 1500000 : 1800000;
		regulator_set_voltage(ldo_18, vol, vol);
		regulator_enable(ldo_18);
		regulator_put(ldo_18);
		msleep(30);
	}
	return 0;
}
#endif

#if CONFIG_SENSOR_RESET_IOCTL_USR
static int sensor_reset_usr_cb (struct rk29camera_gpio_res *res,int on)
{
	#error "CONFIG_SENSOR_RESET_IOCTL_USR is 1, sensor_reset_usr_cb function must be writed!!";
}
#endif

#if CONFIG_SENSOR_POWERDOWN_IOCTL_USR
static int sensor_powerdown_usr_cb (struct rk29camera_gpio_res *res,int on)
{
	#error "CONFIG_SENSOR_POWERDOWN_IOCTL_USR is 1, sensor_powerdown_usr_cb function must be writed!!";
}
#endif

#if CONFIG_SENSOR_FLASH_IOCTL_USR
static int sensor_flash_usr_cb (struct rk29camera_gpio_res *res,int on)
{
	#error "CONFIG_SENSOR_FLASH_IOCTL_USR is 1, sensor_flash_usr_cb function must be writed!!";
}
#endif

static struct rk29camera_platform_ioctl_cb	sensor_ioctl_cb = {
	#if CONFIG_SENSOR_POWER_IOCTL_USR
	.sensor_power_cb = sensor_power_usr_cb,
	#else
	.sensor_power_cb = NULL,
	#endif

	#if CONFIG_SENSOR_RESET_IOCTL_USR
	.sensor_reset_cb = sensor_reset_usr_cb,
	#else
	.sensor_reset_cb = NULL,
	#endif

	#if CONFIG_SENSOR_POWERDOWN_IOCTL_USR
	.sensor_powerdown_cb = sensor_powerdown_usr_cb,
	#else
	.sensor_powerdown_cb = NULL,
	#endif

	#if CONFIG_SENSOR_FLASH_IOCTL_USR
	.sensor_flash_cb = sensor_flash_usr_cb,
	#else
	.sensor_flash_cb = NULL,
	#endif
};


static rk_sensor_user_init_data_s rk_init_data_sensor[RK_CAM_NUM] ;
#include "../../../drivers/media/video/rk30_camera.c"

#endif /* CONFIG_VIDEO_RK29 */
