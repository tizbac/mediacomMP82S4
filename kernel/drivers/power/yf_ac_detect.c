
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>

#include <mach/board.h>
#include <mach/yfmach.h>

#define DEVICE_NAME		"ac-detect"
static int ac_level, ac_gpio, irq;
static bool irq_hang;

static int ac_detect_suspend(struct platform_device *dev, pm_message_t state)
{
	irq_hang = false;
	enable_irq(irq);
	enable_irq_wake(irq);

	return 0;
}

static int ac_detect_resume(struct platform_device *dev)
{
	if (irq_hang) {
		rk28_send_wakeup_key();
	} else {
		disable_irq_nosync(irq);
		disable_irq_wake(irq);
	}

	return 0;
}

static irqreturn_t ac_detect_irq_handler(int irq, void *dev_id)
{
	disable_irq_nosync(irq);
	disable_irq_wake(irq);
	irq_hang = true;
	return IRQ_HANDLED;
}

static int ac_detect_probe(struct platform_device *dev)
{
	int ret, flags;

	ac_level = env_get_u32("power_charge_level", 1);
	ac_gpio = env_get_u32("power_ac_gpio", INVALID_GPIO);
	if (ac_gpio == INVALID_GPIO)
		return -ENODEV;

	irq = gpio_to_irq(ac_gpio);

	// 仅使用cw2013时需要该驱动
	flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
	ret = request_irq(irq, ac_detect_irq_handler, flags, "ac_detect", NULL);
	if (ret < 0) {
		printk("%s: request_irq(%d) failed\n", __func__, irq);
		return -ENODEV;
	}
	disable_irq_nosync(irq);

	return 0;
}

static int __devexit ac_detect_remove(struct platform_device *dev)
{
	return 0;
}

static struct platform_driver ac_detect_driver = {
	.probe		= ac_detect_probe,
	.remove		= __devexit_p(ac_detect_remove),
	.suspend	= ac_detect_suspend,
	.resume		= ac_detect_resume,
	.driver		= {
		.name	= DEVICE_NAME,
		.owner	= THIS_MODULE,
	},
};

static struct platform_device ac_detect_device = {
	.name = DEVICE_NAME,
};

static int __init ac_detect_init(void)
{
	platform_device_register(&ac_detect_device);
	return platform_driver_register(&ac_detect_driver);
}

static void __exit ac_detect_exit(void)
{
	platform_driver_unregister(&ac_detect_driver);
}

module_init(ac_detect_init);
module_exit(ac_detect_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("wuzhibo@yftech.com");
MODULE_DESCRIPTION("AC inside wakeup");
