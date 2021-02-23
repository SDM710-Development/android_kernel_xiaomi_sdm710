/*
 * platform indepent driver interface
 *
 * Coypritht (c) 2017 Goodix
 */
#define DEBUG
#define pr_fmt(fmt)		"gf_platform: " fmt

#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "goodix_fp.h"

#if defined(CONFIG_FINGERPRINT_GOODIX_FP_SPI)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(CONFIG_FINGERPRINT_GOODIX_FP_PLATFORM)
#include <linux/platform_device.h>
#endif

#define FAIL (-1)

int gf_parse_dts(struct gf_dev *gf_dev)
{
#ifdef GF_PW_CTL
	int rc = 0;
	/* get pwr resource */
	gf_dev->pwr_gpio =
	    of_get_named_gpio(gf_dev->dev->of_node, "fp-gpio-pwr", 0);
	if (!gpio_is_valid(gf_dev->pwr_gpio)) {
		pr_info("PWR GPIO is invalid.\n");
		return FAIL;
	}
	rc = gpio_request(gf_dev->pwr_gpio, "goodix_pwr");
	if (rc) {
		dev_err(gf_dev->dev,
			"Failed to request PWR GPIO. rc = %d\n", rc);
		return FAIL;
	}
#endif

	/* get reset resource */
	gf_dev->reset_gpio =
	    of_get_named_gpio(gf_dev->dev->of_node, "goodix,gpio-reset", 0);
	if (!gpio_is_valid(gf_dev->reset_gpio)) {
		pr_info("RESET GPIO is invalid.\n");
		return -EPERM;
	}

	/* get irq resourece */
	gf_dev->irq_gpio =
	    of_get_named_gpio(gf_dev->dev->of_node, "goodix,gpio-irq", 0);
	pr_info("gf::irq_gpio:%d\n", gf_dev->irq_gpio);
	if (!gpio_is_valid(gf_dev->irq_gpio)) {
		pr_info("IRQ GPIO is invalid.\n");
		return -EPERM;
	}

	return 0;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
	pr_info("[info] %s\n", __func__);
	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
#ifdef GF_PW_CTL
	if (gpio_is_valid(gf_dev->pwr_gpio)) {
		gpio_free(gf_dev->pwr_gpio);
		pr_info("remove pwr_gpio success\n");
	}
#endif
}

int gf_set_power(struct gf_dev *gf_dev, bool enable)
{
	int rc = 0;

#ifdef GF_PW_CTL
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
		pr_info("Input buff is NULL.\n");
		return -EPERM;
	}
	gpio_direction_output(gf_dev->reset_gpio, 0);
	mdelay(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	pr_info("%s success\n", __func__);
	return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -EPERM;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}
