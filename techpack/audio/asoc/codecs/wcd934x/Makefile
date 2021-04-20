############ WCD934X ############

# WCD934X Codec
obj-$(CONFIG_SND_SOC_WCD934X) += audio_wcd934x.o
audio_wcd934x-y := wcd934x.o wcd934x-dsp-cntl.o
audio_wcd934x-$(CONFIG_SND_SOC_WCD934X_MBHC) += wcd934x-mbhc.o
audio_wcd934x-$(CONFIG_SND_SOC_WCD934X_DSD) += wcd934x-dsd.o

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include
