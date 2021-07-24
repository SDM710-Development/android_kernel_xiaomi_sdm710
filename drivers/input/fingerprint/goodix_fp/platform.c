/*
 * platform indepent driver interface
 *
 * Coypritht (c) 2017 Goodix
 */
#define pr_fmt(fmt)		KBUILD_MODNAME ": " fmt

#include <linux/err.h>
#include <linux/platform_device.h>

#include "goodix_fp.h"

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
