############ TAS2562 ############

# for TAS2562 Codec
ifdef CONFIG_SND_SOC_TAS2562
	TAS2562_OBJS += tas2562-codec.o
	TAS2562_OBJS += tas2562-regmap.o
	TAS2562_OBJS += tas2562-misc.o
endif

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include

# Module information used by KBuild framework
obj-$(CONFIG_SND_SOC_TAS2562) += tas2562_dlkm.o
tas2562_dlkm-y := $(TAS2562_OBJS)
