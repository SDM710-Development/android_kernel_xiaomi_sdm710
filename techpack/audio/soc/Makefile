############ SoC Modules ############

# pinctrl WCD driver
obj-$(CONFIG_PINCTRL_WCD) += audio_pinctrl_wcd.o
audio_pinctrl_wcd-y := pinctrl-wcd.o

# pinctrl LPI driver
obj-$(CONFIG_PINCTRL_LPI) += audio_pinctrl_lpi.o
audio_pinctrl_lpi-y := pinctrl-lpi.o

# soundwire driver
obj-$(CONFIG_SOUNDWIRE_WCD_CTRL) += audio_swr_ctrl.o
audio_swr_ctrl-y += swr-wcd-ctrl.o

obj-$(CONFIG_SOUNDWIRE) += audio_swr.o
audio_swr-y += regmap-swr.o soundwire.o

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include
