#if defined(CONFIG_KP_AXP22)

#include "../../../drivers/power/axp_power/axp-regu.h"
#include "../../../drivers/power/axp_power/axp-mfd.h"
#endif
#include <mach/sram.h>
#include <linux/platform_device.h>

#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <mach/yfmach.h>

#include <linux/init.h>
#include <linux/device.h>
#include <linux/regulator/machine.h>
#include <linux/i2c.h>
#include <mach/irqs.h>
#include <linux/power_supply.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include "../../../drivers/power/axp_power/axp-cfg.h"
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif


#if defined(CONFIG_KP_AXP22)

struct axp228_consumer_data {
	struct mutex lock;
	struct regulator *regulator;
	int enabled;
	int min_uV;
	int max_uV;
	int min_uA;
	int max_uA;
	unsigned int mode;
};

static int axp228_set_init(void);//(struct platform_device *axp228_dev);
/*enum axp_regls{

	vcc_ldo1,//0 , rtc
	vcc_ldo2,//1, aldo1
	vcc_ldo3,//2, aldo2
	vcc_ldo4,//3, aldo3
	vcc_ldo5,//4, dldo1
	vcc_ldo6,//5, dldo2 act_ldo8
	vcc_ldo7,//6, dldo3
	vcc_ldo8,//7, dldo4
	vcc_ldo9,//8, eldo1
	vcc_ldo10,//9, eldo2
	vcc_ldo11,//10, eldo3 act_ldo3
	vcc_ldo12,//11,dc5ldo
	
	vcc_DCDC1,//12,
	vcc_DCDC2,//13
	vcc_DCDC3,//14
	vcc_DCDC4,//15
	vcc_DCDC5,//16
	vcc_ldoio0,//17 gpio0
	vcc_ldoio1,//18
};*/
static struct platform_device axp228[]={
	{
			.name = "reg-22-cs-rtc",
			.id = 0,
			.dev		= {
				.platform_data = "axp22_rtc",
			}
 	},{
			.name = "reg-22-cs-aldo1",
			.id = 1,
			.dev		= {
				.platform_data = "axp22_aldo1",
			}
 	},{
			.name = "reg-22-cs-aldo2",
			.id = 2,
			.dev		= {
				.platform_data = "axp22_aldo2",
			}
 	},{
			.name = "reg-22-cs-aldo3",
			.id = 3,
			.dev		= {
				.platform_data = "axp22_aldo3",
			}
	},{
			.name = "reg-22-cs-dldo1",
			.id = 4,
			.dev		= {
				.platform_data = "axp22_dldo1",
			}
 	},{
			.name = "reg-22-cs-dldo2",
			.id = 5,
			.dev		= {
				.platform_data = "act_ldo8",
			}
 	},{
			.name = "reg-22-cs-dldo3",
			.id = 6,
			.dev		= {
				.platform_data = "axp22_dldo3",
			}
	},{
			.name = "reg-22-cs-dldo4",
			.id = 7,
			.dev		= {
				.platform_data = "axp22_dldo4",
			}
	},{
			.name = "reg-22-cs-eldo1",
			.id = 8,
			.dev		= {
				.platform_data = "axp22_eldo1",
			}
	},{
			.name = "reg-22-cs-eldo2",
			.id = 9,
			.dev		= {
				.platform_data = "axp22_eldo2",
			}
	},{
			.name = "reg-22-cs-eldo3",
			.id = 10,
			.dev		= {
				.platform_data = "act_ldo3",
			}
	},{
			.name = "reg-22-cs-dc5ldo",
			.id = 11,
			.dev		= {
				.platform_data = "axp22_dc5ldo",
			}
	},{
			.name = "reg-22-cs-dcdc1",
			.id = 12,
			.dev		= {
				.platform_data =  "axp22_dcdc1",
			}
	},{
			.name = "reg-22-cs-dcdc2",
			.id = 13,
			.dev		= {
				.platform_data = "axp22_dcdc2",
			}
	},{
			.name = "reg-22-cs-dcdc3",
			.id = 14,
			.dev		= {
				.platform_data = "axp22_dcdc3",
			}
	},{
			.name = "reg-22-cs-dcdc4",
			.id = 15,
			.dev		= {
				.platform_data = "axp22_dcdc4",
			}
 	},{
			.name = "reg-22-cs-dcdc5",
			.id = 16,
			.dev		= {
				.platform_data = "axp22_dcdc5",
			}
	},{
			.name = "reg-22-cs-gpio0ldo",
			.id = 17,
			.dev		= {
				.platform_data = "axp22_ldoio0",
			}
 	},{
			.name = "reg-22-cs-gpio1ldo",
			.id = 18,
			.dev		= {
				.platform_data = "axp22_ldoio1",
			}
	},
};



 static int __init axp228_init(void)
{
	int j,ret;
	for (j = 0; j < ARRAY_SIZE(axp228); j++){
 		ret =  platform_device_register(&axp228[j]);
 		
  		if (ret)
				goto creat_devices_failed;
	}axp228_set_init();
	return ret;

creat_devices_failed:
	while (j--)
		platform_device_register(&axp228[j]);
	return ret;

}
subsys_initcall_sync(axp228_init);
//module_init(axp228_init);

static void __exit axp228_exit(void)
{
	int j;
	for (j = ARRAY_SIZE(axp228) - 1; j >= 0; j--){
		platform_device_unregister(&axp228[j]);
	}
}
module_exit(axp228_exit);


//static struct axp228_consumer_data *s_axp228 ;
/*static int axp228_data_read(u8 index)
{
	
	return 1;//not set yet
}

static int axp228_data_write(u8 index, u8 value)
{
	
	return 1;//not set yet
}*/
static int   axp228_set_init()//(struct platform_device *axp228_dev)
{
	struct regulator *dcdc;
	struct regulator *ldo;
	int i;
	char * found;
	const char * disabled = env_get_str("power_disabled_reg", "act_ldo8,act_ldo3,");
    printk("%s,line=%d\n", __func__,__LINE__);
	
	#ifndef CONFIG_RK_CONFIG
	g_pmic_type = PMIC_TYPE_AXP228;
	#endif
	printk("%s:g_pmic_type=%d\n",__func__,g_pmic_type);
	//s_axp228 = dev_get_drvdata(axp228_dev);
	
	printk("************************start set voltage herer***********************************\n");
	
	/*for (i = 0; i < ARRAY_SIZE(axp228_dcdc_info); i++)
	{printk("axp228_dcdc_info[i].name==========%s\n",axp228_dcdc_info[i].name);
		//if(axp228_dcdc_info[i].name==(&axp228_dev).dev.platform_data){
		dcdc =regulator_get(NULL, axp228_dcdc_info[i].name);//printk("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa  dcdc supply_name =%d\n",dcdc.min_uV);
		if(dcdc==NULL) break;
		
		if(axp228_dcdc_info[i].min_uv > regulator_get_voltage(dcdc)||regulator_get_voltage(dcdc)>axp228_dcdc_info[i].max_uv)
		regulator_set_voltage(dcdc, axp228_dcdc_info[i].min_uv, axp228_dcdc_info[i].max_uv);
		printk("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n");
		regulator_enable(dcdc);
		printk("ccccccccccccccccccccccccccccccccccccc\n");
	    printk("%s  %s =%dmV end\n", __func__,axp228_dcdc_info[i].name, regulator_get_voltage(dcdc));
	    regulator_put(dcdc);printk("dddddddddddddddddddddddddddddddddd\n");
	    //f(axp228_dcdc_info[i].name==("axp22_dcdc1")){axp_write(&axp->dev,0x12,0xff);}
	    
	    udelay(100);//}
	}*/ 
	
	for(i = 0; i < ARRAY_SIZE(axp228_ldo_info); i++)
	{
                
	        ldo =regulator_get(NULL, axp228_ldo_info[i].name);
	        regulator_set_voltage(ldo, axp228_ldo_info[i].min_uv, axp228_ldo_info[i].max_uv);
	        regulator_enable(ldo);
	        printk("%s  %s =%dmV end\n", __func__,axp228_ldo_info[i].name, regulator_get_voltage(ldo));
		found = strstr(disabled, axp228_ldo_info[i].name);
		if(found && found[strlen(axp228_ldo_info[i].name)] == ',') {
			regulator_disable(ldo);
		}
	        regulator_put(ldo);
	}
	return 1;
}



static void update_voltage_constraints(struct axp228_consumer_data *data)
{
	int ret;

	if (data->min_uV && data->max_uV
	    && data->min_uV <= data->max_uV) {
		ret = regulator_set_voltage(data->regulator,
					    data->min_uV, data->max_uV);
		if (ret != 0) {
			printk(KERN_ERR "regulator_set_voltage() failed: %d\n",
			       ret);
			return;
		}
	}

	if (data->min_uV && data->max_uV && !data->enabled) {
		ret = regulator_enable(data->regulator);
		if (ret == 0)
			data->enabled = 1;
		else
			printk(KERN_ERR "regulator_enable() failed: %d\n",
				ret);
	}

	if (!(data->min_uV && data->max_uV) && data->enabled) {
		ret = regulator_disable(data->regulator);
		if (ret == 0)
			data->enabled = 0;
		else
			printk(KERN_ERR "regulator_disable() failed: %d\n",
				ret);
	}
}

static void update_current_limit_constraints(struct axp228_consumer_data
						*data)
{
	int ret;

	if (data->max_uA
	    && data->min_uA <= data->max_uA) {
		ret = regulator_set_current_limit(data->regulator,
					data->min_uA, data->max_uA);
		if (ret != 0) {
			pr_err("regulator_set_current_limit() failed: %d\n",
			       ret);
			return;
		}
	}

	if (data->max_uA && !data->enabled) {
		ret = regulator_enable(data->regulator);
		if (ret == 0)
			data->enabled = 1;
		else
			printk(KERN_ERR "regulator_enable() failed: %d\n",
				ret);
	}

	if (!(data->min_uA && data->max_uA) && data->enabled) {
		ret = regulator_disable(data->regulator);
		if (ret == 0)
			data->enabled = 0;
		else
			printk(KERN_ERR "regulator_disable() failed: %d\n",
				ret);
	}
}

static ssize_t show_min_uV(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct axp228_consumer_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->min_uV);
}

static ssize_t set_min_uV(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct axp228_consumer_data *data = dev_get_drvdata(dev);
	long val;

	if (strict_strtol(buf, 10, &val) != 0)
		return count;

	mutex_lock(&data->lock);

	data->min_uV = val;
	update_voltage_constraints(data);

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t show_max_uV(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct axp228_consumer_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->max_uV);
}

static ssize_t set_max_uV(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct axp228_consumer_data *data = dev_get_drvdata(dev);
	long val;

	if (strict_strtol(buf, 10, &val) != 0)
		return count;

	mutex_lock(&data->lock);

	data->max_uV = val;
	update_voltage_constraints(data);

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t show_min_uA(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct axp228_consumer_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->min_uA);
}

static ssize_t set_min_uA(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct axp228_consumer_data *data = dev_get_drvdata(dev);
	long val;

	if (strict_strtol(buf, 10, &val) != 0)
		return count;

	mutex_lock(&data->lock);

	data->min_uA = val;
	update_current_limit_constraints(data);

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t show_max_uA(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct axp228_consumer_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->max_uA);
}

static ssize_t set_max_uA(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct axp228_consumer_data *data = dev_get_drvdata(dev);
	long val;

	if (strict_strtol(buf, 10, &val) != 0)
		return count;

	mutex_lock(&data->lock);

	data->max_uA = val;
	update_current_limit_constraints(data);

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t show_mode(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct axp228_consumer_data *data = dev_get_drvdata(dev);

	switch (data->mode) {
	case REGULATOR_MODE_FAST:
		return sprintf(buf, "fast\n");
	case REGULATOR_MODE_NORMAL:
		return sprintf(buf, "normal\n");
	case REGULATOR_MODE_IDLE:
		return sprintf(buf, "idle\n");
	case REGULATOR_MODE_STANDBY:
		return sprintf(buf, "standby\n");
	default:
		return sprintf(buf, "unknown\n");
	}
}

static ssize_t set_mode(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct axp228_consumer_data *data = dev_get_drvdata(dev);
	unsigned int mode;
	int ret;

	if (strncmp(buf, "fast", strlen("fast")) == 0)
		mode = REGULATOR_MODE_FAST;
	else if (strncmp(buf, "normal", strlen("normal")) == 0)
		mode = REGULATOR_MODE_NORMAL;
	else if (strncmp(buf, "idle", strlen("idle")) == 0)
		mode = REGULATOR_MODE_IDLE;
	else if (strncmp(buf, "standby", strlen("standby")) == 0)
		mode = REGULATOR_MODE_STANDBY;
	else {
		dev_err(dev, "Configuring invalid mode\n");
		return count;
	}

	mutex_lock(&data->lock);
	ret = regulator_set_mode(data->regulator, mode);
	if (ret == 0)
		data->mode = mode;
	else
		dev_err(dev, "Failed to configure mode: %d\n", ret);
	mutex_unlock(&data->lock);

	return count;
}

static DEVICE_ATTR(min_microvolts, 0644, show_min_uV, set_min_uV);
static DEVICE_ATTR(max_microvolts, 0644, show_max_uV, set_max_uV);
static DEVICE_ATTR(min_microamps, 0644, show_min_uA, set_min_uA);
static DEVICE_ATTR(max_microamps, 0644, show_max_uA, set_max_uA);
static DEVICE_ATTR(mode, 0644, show_mode, set_mode);

struct device_attribute *attributes_axp228[] = {
	&dev_attr_min_microvolts,
	&dev_attr_max_microvolts,
	&dev_attr_min_microamps,
	&dev_attr_max_microamps,
	&dev_attr_mode,
};

static int regulator_axp228_consumer_probe(struct platform_device *pdev)
{
	char *reg_id = pdev->dev.platform_data;
	struct axp228_consumer_data *drvdata;
	int ret, i;

	drvdata = kzalloc(sizeof(struct axp228_consumer_data), GFP_KERNEL);
	if (drvdata == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	mutex_init(&drvdata->lock);

	drvdata->regulator = regulator_get(NULL, reg_id);
	if (IS_ERR(drvdata->regulator)) {
		ret = PTR_ERR(drvdata->regulator);
		goto err;
	}
	
	//for (i = 0; i < ARRAY_SIZE(attributes_axp228); i++) {
		//ret = device_create_file(&pdev->dev, attributes_axp228[i]);
		//if (ret != 0)
		//	goto err;
	//}

	drvdata->mode = regulator_get_mode(drvdata->regulator);

	platform_set_drvdata(pdev, drvdata);

	return 0;

err:
	for (i = 0; i < ARRAY_SIZE(attributes_axp228); i++)
		device_remove_file(&pdev->dev, attributes_axp228[i]);
	kfree(drvdata);
//	printk("axp22 regulator  axp228 get drvdata->regulator = %d err\n",drvdata->regulator);
	return ret;
}

static int regulator_axp228_consumer_remove(struct platform_device *pdev)
{
	struct axp228_consumer_data *drvdata = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes_axp228); i++)
		device_remove_file(&pdev->dev, attributes_axp228[i]);
	if (drvdata->enabled)
		regulator_disable(drvdata->regulator);
	regulator_put(drvdata->regulator);

	kfree(drvdata);

	return 0;
}

static struct platform_driver regulator_axp228_consumer_driver[] = {
	{
		.probe		= regulator_axp228_consumer_probe,
		.remove		= regulator_axp228_consumer_remove,
		.driver		= {
			.name		= "reg-22-cs-rtc",
		},
	},{
		.probe		= regulator_axp228_consumer_probe,
		.remove		= regulator_axp228_consumer_remove,
		.driver		= {
			.name		= "reg-22-cs-aldo1",
		},
	},{
		.probe		= regulator_axp228_consumer_probe,
		.remove		= regulator_axp228_consumer_remove,
		.driver		= {
			.name		= "reg-22-cs-aldo2",
		},
	},{
		.probe		= regulator_axp228_consumer_probe,
		.remove		= regulator_axp228_consumer_remove,
		.driver		= {
			.name		= "reg-22-cs-aldo3",
		},
	},{
		.probe		= regulator_axp228_consumer_probe,
		.remove		= regulator_axp228_consumer_remove,
		.driver		= {
			.name		= "reg-22-cs-dldo1",
		},
	},{
		.probe		= regulator_axp228_consumer_probe,
		.remove		= regulator_axp228_consumer_remove,
		.driver		= {
			.name		= "reg-22-cs-dldo2",
		},
	},{
		.probe		= regulator_axp228_consumer_probe,
		.remove		= regulator_axp228_consumer_remove,
		.driver		= {
			.name		= "reg-22-cs-dldo3",
		},
	},{
		.probe		= regulator_axp228_consumer_probe,
		.remove		= regulator_axp228_consumer_remove,
		.driver		= {
			.name		= "reg-22-cs-dldo4",
		},
	},{
		.probe		= regulator_axp228_consumer_probe,
		.remove		= regulator_axp228_consumer_remove,
		.driver		= {
			.name		= "reg-22-cs-eldo1",
		},
	},{
		.probe		= regulator_axp228_consumer_probe,
		.remove		= regulator_axp228_consumer_remove,
		.driver		= {
			.name		= "reg-22-cs-eldo2",
		},
	},{
		.probe		= regulator_axp228_consumer_probe,
		.remove		= regulator_axp228_consumer_remove,
		.driver		= {
			.name		= "reg-22-cs-eldo3",
		},
	},{
		.probe		= regulator_axp228_consumer_probe,
		.remove		= regulator_axp228_consumer_remove,
		.driver		= {
			.name		= "reg-22-cs-dcdc1",
		},
	},{
		.probe		= regulator_axp228_consumer_probe,
		.remove		= regulator_axp228_consumer_remove,
		.driver		= {
			.name		= "reg-22-cs-dcdc2",
		},
	},{
		.probe		= regulator_axp228_consumer_probe,
		.remove		= regulator_axp228_consumer_remove,
		.driver		= {
			.name		= "reg-22-cs-dcdc3",
		},
	},{
		.probe		= regulator_axp228_consumer_probe,
		.remove		= regulator_axp228_consumer_remove,
		.driver		= {
			.name		= "reg-22-cs-dcdc4",
		},
	},{
		.probe		= regulator_axp228_consumer_probe,
		.remove		= regulator_axp228_consumer_remove,
		.driver		= {
			.name		= "reg-22-cs-dcdc5",
		},
	},{
		.probe		= regulator_axp228_consumer_probe,
		.remove		= regulator_axp228_consumer_remove,
		.driver		= {
			.name		= "reg-22-cs-gpio0ldo",
		},
	},{
		.probe		= regulator_axp228_consumer_probe,
		.remove		= regulator_axp228_consumer_remove,
		.driver		= {
			.name		= "reg-22-cs-gpio1ldo",
		},
	},
	
};


static int __init regulator_axp228_consumer_init(void)
{
	int j,ret;
	for (j = 0; j < ARRAY_SIZE(regulator_axp228_consumer_driver); j++){ 
		ret =  platform_driver_register(&regulator_axp228_consumer_driver[j]);
		if (ret)
			goto creat_drivers_failed;
	}
	return ret;
		
creat_drivers_failed:
	while (j--)
		platform_driver_unregister(&regulator_axp228_consumer_driver[j]);
	return ret;
}
module_init(regulator_axp228_consumer_init);

static void __exit regulator_axp228_consumer_exit(void)
{
	int j;
	for (j = ARRAY_SIZE(regulator_axp228_consumer_driver) - 1; j >= 0; j--){ 
			platform_driver_unregister(&regulator_axp228_consumer_driver[j]);
	}
}
module_exit(regulator_axp228_consumer_exit);



#endif

