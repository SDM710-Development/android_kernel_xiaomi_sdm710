############ TAS2562 ############

# TAS2562 Codec
obj-$(CONFIG_SND_SOC_TAS2562) += audio_tas2562.o
audio_tas2562-y := tas2562-codec.o tas2562-misc.o tas2562-regmap.o

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include
