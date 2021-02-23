/*
 * Driver definition for sensor driver
 *
 * Coypright (c) 2017 Goodix
 */
#ifndef __GF_SPI_H
#define __GF_SPI_H

#include <linux/types.h>
#include <linux/notifier.h>

#define GF_DEV_NAME	"goodix_fp"

enum FP_MODE {
	GF_IMAGE_MODE = 0,
	GF_KEY_MODE,
	GF_SLEEP_MODE,
	GF_FF_MODE,
	GF_DEBUG_MODE = 0x56
};

#define GF_NAV_INPUT_UP			KEY_UP
#define GF_NAV_INPUT_DOWN		KEY_DOWN
#define GF_NAV_INPUT_LEFT		KEY_LEFT
#define GF_NAV_INPUT_RIGHT		KEY_RIGHT
#define GF_NAV_INPUT_CLICK		KEY_VOLUMEDOWN
#define GF_NAV_INPUT_DOUBLE_CLICK	KEY_VOLUMEUP
#define GF_NAV_INPUT_LONG_PRESS		KEY_SEARCH
#define GF_NAV_INPUT_HEAVY		KEY_CHAT

#define GF_KEY_INPUT_HOME		KEY_HOME
#define GF_KEY_INPUT_MENU		KEY_MENU
#define GF_KEY_INPUT_BACK		KEY_BACK
#define GF_KEY_INPUT_POWER		KEY_POWER
#define GF_KEY_INPUT_CAMERA		KEY_CAMERA

typedef enum gf_nav_event {
	GF_NAV_NONE = 0,
	GF_NAV_FINGER_UP,
	GF_NAV_FINGER_DOWN,
	GF_NAV_UP,
	GF_NAV_DOWN,
	GF_NAV_LEFT,
	GF_NAV_RIGHT,
	GF_NAV_CLICK,
	GF_NAV_HEAVY,
	GF_NAV_LONG_PRESS,
	GF_NAV_DOUBLE_CLICK,
} gf_nav_event_t;

typedef enum gf_key_event {
	GF_KEY_NONE = 0,
	GF_KEY_HOME,
	GF_KEY_POWER,
	GF_KEY_MENU,
	GF_KEY_BACK,
	GF_KEY_CAMERA,
} gf_key_event_t;

struct gf_key {
	enum gf_key_event key;
	uint32_t value;		/* key down = 1, key up = 0 */
};

struct gf_key_map {
	unsigned int type;
	unsigned int code;
};

struct gf_ioc_chip_info {
	unsigned char vendor_id;
	unsigned char mode;
	unsigned char operation;
	unsigned char reserved[5];
};

#define GF_IOC_MAGIC		'g'	/*define magic number */
#define GF_IOC_INIT		_IOR(GF_IOC_MAGIC, 0, uint8_t)
#define GF_IOC_EXIT		_IO(GF_IOC_MAGIC, 1)
#define GF_IOC_RESET		_IO(GF_IOC_MAGIC, 2)
#define GF_IOC_ENABLE_IRQ	_IO(GF_IOC_MAGIC, 3)
#define GF_IOC_DISABLE_IRQ	_IO(GF_IOC_MAGIC, 4)
#define GF_IOC_ENABLE_SPI_CLK	_IOW(GF_IOC_MAGIC, 5, uint32_t)
#define GF_IOC_DISABLE_SPI_CLK	_IO(GF_IOC_MAGIC, 6)
#define GF_IOC_ENABLE_POWER	_IO(GF_IOC_MAGIC, 7)
#define GF_IOC_DISABLE_POWER	_IO(GF_IOC_MAGIC, 8)
#define GF_IOC_INPUT_KEY_EVENT	_IOW(GF_IOC_MAGIC, 9, struct gf_key)
#define GF_IOC_ENTER_SLEEP_MODE	_IO(GF_IOC_MAGIC, 10)
#define GF_IOC_GET_FW_INFO	_IOR(GF_IOC_MAGIC, 11, uint8_t)
#define GF_IOC_REMOVE		_IO(GF_IOC_MAGIC, 12)
#define GF_IOC_CHIP_INFO	_IOW(GF_IOC_MAGIC, 13, struct gf_ioc_chip_info)
#define GF_IOC_NAV_EVENT	_IOW(GF_IOC_MAGIC, 14, gf_nav_event_t)

/*#define AP_CONTROL_CLK       1*/
#define GF_NETLINK_ENABLE	1
#define GF_NET_EVENT_IRQ	1
#define GF_NET_EVENT_FB_BLACK	2
#define GF_NET_EVENT_FB_UNBLACK	3

struct gf_dev {
	dev_t devt;
	struct list_head device_entry;
	struct device *dev;
	struct clk *core_clk;
	struct clk *iface_clk;

	struct input_dev *input;
	/* buffer is NULL unless this device is open (users > 0) */
	unsigned users;
	signed irq_gpio;
	signed reset_gpio;
	signed pwr_gpio;
	int irq;
	int irq_enabled;
	int clk_enabled;
	struct fasync_struct *async;
	struct notifier_block notifier;
	bool avail;
	char fb_black;
	char wait_finger_down;
	struct work_struct work;
#ifdef CONFIG_FINGERPRINT_FP_VREG_CONTROL
	struct regulator *vreg;
#endif
};

int gf_probe_common(struct device *dev);
int gf_remove_common(struct device *dev);

int gf_parse_dts(struct gf_dev *gf_dev);
void gf_cleanup(struct gf_dev *gf_dev);

#ifdef CONFIG_FINGERPRINT_GOODIX_FP_SPI

int gf_register_spi_driver(struct of_device_id *match_table);
void gf_unregister_spi_driver(void);

#else

static inline
int gf_register_spi_driver(struct of_device_id *match_table)
{
	return 0;
}

static inline
void gf_unregister_spi_driver(void)
{
}

#endif

#ifdef CONFIG_FINGERPRINT_GOODIX_FP_PLATFORM

int gf_register_platform_driver(struct of_device_id *match_table);
void gf_unregister_platform_driver(void);

#else

static inline
int gf_register_platform_driver(struct of_device_id *match_table)
{
	return 0;
}

static inline
void gf_unregister_platform_driver(void)
{
}

#endif

void gf_sendnlmsg(char *message);
int gf_netlink_init(void);
void gf_netlink_exit(void);

#endif /*__GF_SPI_H*/
