/*
 * SPI specific code
 *
 * Coypritht (c) 2017 Goodix
 * Copyright (C) 2021 Ivan Vecera <ivan@cera.cz>
 */
#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include "goodix_fp.h"

static int gf_probe_spi(struct spi_device *spidev)
{
	return gf_probe_common(&spidev->dev);
}

static int gf_remove_spi(struct spi_device *spidev)
{
	return gf_remove_common(&spidev->dev);
}

static struct spi_driver gf_spi_driver = {
	.driver = {
		.name = GF_DEV_NAME "_spi",
		.owner = THIS_MODULE,
	},
	.probe = gf_probe_spi,
	.remove = gf_remove_spi,
};

int gf_register_spi_driver(struct of_device_id *match_table)
{
	int rc;

	gf_spi_driver.driver.of_match_table = match_table;

	rc = spi_register_driver(&gf_spi_driver);
	if (rc < 0)
		pr_err("failed to register SPI driver\n");

	return rc;
}

void gf_unregister_spi_driver(void)
{
	spi_unregister_driver(&gf_spi_driver);
}
