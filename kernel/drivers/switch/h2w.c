#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

static struct switch_dev * ph2wdev;

#define CHECK_TICK			msecs_to_jiffies(500)
#define DELAY_WORK_TIMES	(10)

#define EARPHONE_SWITCH_NAME	"h2w"
#define EARPHONE_PIN			RK30_PIN0_PB4

static unsigned char earphone_in = 1, delay_work_time;
static struct delayed_work earphone_work;
extern void earphone_mute_spk(int off);
extern void earphone_mute_spk_616(int off);

static void do_earphone_work(struct work_struct *work)
{
	static unsigned char earphone_check_filter;
	unsigned char val;

	val = gpio_get_value(EARPHONE_PIN);
	if (earphone_in != val) {
		earphone_in = val;

		if (ph2wdev) {
			printk("earphone %s\n", !earphone_in ? "inside":"remove");
			switch_set_state(ph2wdev, !earphone_in);	// 0: inside
		}
	} else {
		earphone_check_filter = 0;
		++delay_work_time;
		if (delay_work_time == 3) {
			earphone_mute_spk(0);
			earphone_mute_spk_616(0);
		}
		if (delay_work_time > DELAY_WORK_TIMES) {
			return;
		}
	}

	schedule_delayed_work(&earphone_work, CHECK_TICK);
}

static irqreturn_t earphone_isr(int irq, void *dev_id)
{
	earphone_mute_spk(1);
	earphone_mute_spk_616(1);

	delay_work_time = 0;
	schedule_delayed_work(&earphone_work, 0);

	return IRQ_HANDLED;
}

static ssize_t earphone_print_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", !earphone_in);
}

static int earphone_switch_init(void)
{
	int ret, irq;

	ph2wdev = kzalloc(sizeof(struct switch_dev), GFP_KERNEL);
	if (!ph2wdev)
		return -ENOMEM;

	ph2wdev->name = EARPHONE_SWITCH_NAME;
	ph2wdev->print_state = earphone_print_state;
	switch_dev_register(ph2wdev);

	INIT_DELAYED_WORK(&earphone_work, do_earphone_work);

	ret = gpio_request(EARPHONE_PIN, "earphone");
	if (ret < 0) {
		pr_err("earphone: failed to request GPIO %d\n", EARPHONE_PIN);
		return ret;
	}
	irq = gpio_to_irq(EARPHONE_PIN);
	ret = request_irq(irq, earphone_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "earphone", NULL);
	if (ret) {
		pr_err("earphone: Unable to claim irq %d\n", irq);
		gpio_free(EARPHONE_PIN);
		return ret;
	}
	schedule_delayed_work(&earphone_work, CHECK_TICK);

	return 0;
}

static int h2w_suspend(struct platform_device * pdev, pm_message_t state)
{
	return 0;
}

static int h2w_resume(struct platform_device * pdev)
{
	delay_work_time = 0;
	schedule_delayed_work(&earphone_work, CHECK_TICK);

	return 0;
}

static int h2w_switch_probe(struct platform_device *pdev)
{
	earphone_switch_init();
	return 0;
}

static int __devexit h2w_switch_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver h2w_switch_driver = {
	.probe		= h2w_switch_probe,
	.remove		= __devexit_p(h2w_switch_remove),
	.suspend    = h2w_suspend,
	.resume     = h2w_resume,
	.driver		= {
		.name	= "yftech_h2w",
		.owner	= THIS_MODULE,
	},
};

struct platform_device h2w_switch_device = {
	.name = "yftech_h2w",
	.id = -1,
};

static int __init modinit(void)
{
	platform_driver_register(&h2w_switch_driver);
	platform_device_register(&h2w_switch_device);

	return 0;
}
module_init(modinit);

static void __exit modexit(void)
{
	platform_driver_unregister(&h2w_switch_driver);
}
module_exit(modexit);

MODULE_DESCRIPTION("rk h2w driver");
MODULE_AUTHOR("rk@rock-chips.com");
MODULE_LICENSE("GPL");
