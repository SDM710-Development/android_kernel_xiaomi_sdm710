# TAS2557 Codec
obj-$(CONFIG_SND_SOC_TAS2557) += audio_tas2557.o
audio_tas2557-y := tas2557-codec.o tas2557-core.o tas2557-misc.o \
		   tas2557-regmap.o tiload.o

CFLAGS_tas2557-misc.o += $(call cc-option, -Wno-shift-overflow)

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include
