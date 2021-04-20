############ SoC Modules ############

# for pinctrl WCD driver
ifdef CONFIG_PINCTRL_WCD
	PINCTRL_WCD_OBJS += pinctrl-wcd.o
endif

# for pinctrl LPI driver
ifdef CONFIG_PINCTRL_LPI
	PINCTRL_LPI_OBJS += pinctrl-lpi.o
endif

# for soundwire driver
ifdef CONFIG_SOUNDWIRE_WCD_CTRL
	SWR_CTRL_OBJS += swr-wcd-ctrl.o
endif

ifdef CONFIG_SOUNDWIRE
	SWR_OBJS += regmap-swr.o
	SWR_OBJS += soundwire.o
endif

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include

# Currently, for versions of gcc which support it, the kernel Makefile
# is disabling the maybe-uninitialized warning.  Re-enable it for the
# AUDIO driver.  Note that we must use EXTRA_CFLAGS here so that it
# will override the kernel settings.
ifeq ($(call cc-option-yn, -Wmaybe-uninitialized),y)
EXTRA_CFLAGS += -Wmaybe-uninitialized
endif
#EXTRA_CFLAGS += -Wmissing-prototypes

ifeq ($(call cc-option-yn, -Wheader-guard),y)
EXTRA_CFLAGS += -Wheader-guard
endif

# Module information used by KBuild framework
obj-$(CONFIG_PINCTRL_WCD) += pinctrl_wcd_dlkm.o
pinctrl_wcd_dlkm-y := $(PINCTRL_WCD_OBJS)

obj-$(CONFIG_PINCTRL_LPI) += pinctrl_lpi_dlkm.o
pinctrl_lpi_dlkm-y := $(PINCTRL_LPI_OBJS)

obj-$(CONFIG_SOUNDWIRE) += swr_dlkm.o
swr_dlkm-y := $(SWR_OBJS)

obj-$(CONFIG_SOUNDWIRE_WCD_CTRL) += swr_ctrl_dlkm.o
swr_ctrl_dlkm-y := $(SWR_CTRL_OBJS)
