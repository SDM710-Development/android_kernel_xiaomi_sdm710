/*
 * platform indepent driver interface
 *
 * Coypritht (c) 2017 Goodix
 */
#define pr_fmt(fmt)		KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

#include "goodix_fp.h"

int gf_parse_dts(struct gf_dev *gf_dev)
{
	int rc;

#ifdef CONFIG_FINGERPRINT_GOODIX_FP_POWER_CTRL
	/* get pwr resource */
	rc = of_get_named_gpio(gf_dev->dev->of_node, "fp-gpio-pwr", 0);
	if (gpio_is_valid(rc)) {
		gf_dev->pwr_gpio = rc;

		rc = devm_gpio_request(gf_dev->dev, gf_dev->pwr_gpio,
				       "goodix_pwr");
		if (rc < 0) {
			dev_err(gf_dev->dev, "failed to request PWR GPIO\n");
			return rc;
		}
	} else {
		gf_dev->pwr_gpio = -1; /* do not use pwr gpio */
	}
#endif

	/* get reset resource */
	rc = of_get_named_gpio(gf_dev->dev->of_node, "goodix,gpio-reset", 0);
	if (!gpio_is_valid(rc)) {
		dev_err(gf_dev->dev, "RESET GPIO is invalid\n");
		return rc;
	}
	gf_dev->reset_gpio = rc;

	/* get irq resourece */
	rc = of_get_named_gpio(gf_dev->dev->of_node, "goodix,gpio-irq", 0);
	if (!gpio_is_valid(rc)) {
		dev_info(gf_dev->dev, "IRQ GPIO is invalid\n");
		return rc;
	}
	gf_dev->irq_gpio = rc;

	return 0;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		dev_info(gf_dev->dev, "remove irq_gpio success\n");
	}

	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		dev_info(gf_dev->dev, "remove reset_gpio success\n");
	}
}

int gf_set_power(struct gf_dev *gf_dev, bool enable)
{
	int rc = 0;

#ifdef CONFIG_FINGERPRINT_GOODIX_FP_POWER_CTRL
	if (gpio_is_valid(gf_dev->pwr_gpio)) {
		rc = gpio_direction_output(gf_dev->pwr_gpio, enable ? 1 : 0);
		dev_info(gf_dev->dev, "set_power(%s) %s\n",
			 enable ? "on" : "off", !rc ? "succeeded" : "failed");
	}
	msleep(10);
#endif

	return rc;
}

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if (gf_dev == NULL) {
		dev_info(gf_dev->dev, "Input buff is NULL.\n");
		return -EPERM;
	}
	gpio_direction_output(gf_dev->reset_gpio, 0);
	mdelay(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	dev_info(gf_dev->dev, "%s success\n", __func__);
	return 0;
}

#if defined(CONFIG_FINGERPRINT_GOODIX_FP_PLATFORM)

static int gf_probe_platform(struct platform_device *pdev)
{
	return gf_probe_common(&pdev->dev);
}

static int gf_remove_platform(struct platform_device *pdev)
{
	return gf_remove_common(&pdev->dev);
}

static struct platform_driver gf_platform_driver = {
	.driver = {
		.name = GF_DEV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = gf_probe_platform,
	.remove = gf_remove_platform,
};

int gf_register_platform_driver(struct of_device_id *match_table)
{
	int rc;

	gf_platform_driver.driver.of_match_table = match_table;

	rc = platform_driver_register(&gf_platform_driver);
	if (rc < 0)
		pr_err("failed to register platform driver\n");

	return rc;
}

void gf_unregister_platform_driver(void)
{
	platform_driver_unregister(&gf_platform_driver);
}

#endif
