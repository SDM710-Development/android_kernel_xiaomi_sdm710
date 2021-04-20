# TAS2557 Codec
ifdef CONFIG_SND_SOC_TAS2557
	TAS2557_OBJS += tas2557-core.o
	TAS2557_OBJS += tas2557-misc.o
	TAS2557_OBJS += tas2557-codec.o
	TAS2557_OBJS += tas2557-regmap.o
	TAS2557_OBJS += tiload.o
	CFLAGS_tas2557-misc.o += $(call cc-option, -Wno-shift-overflow)
endif

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include

obj-$(CONFIG_SND_SOC_TAS2557)    += snd-soc-tas2557_dlkm.o
snd-soc-tas2557_dlkm-y := $(TAS2557_OBJS)
