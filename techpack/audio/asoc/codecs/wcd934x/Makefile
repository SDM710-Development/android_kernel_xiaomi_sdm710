############ WCD934X ############

# for WCD934X Codec
ifdef CONFIG_SND_SOC_WCD934X
	WCD934X_OBJS += wcd934x.o
	WCD934X_OBJS += wcd934x-dsp-cntl.o
	WCD934X_OBJS += wcd934x-mbhc.o
	WCD934X_OBJS += wcd934x-dsd.o
endif

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include

# Module information used by KBuild framework
obj-$(CONFIG_SND_SOC_WCD934X) += wcd934x_dlkm.o
wcd934x_dlkm-y := $(WCD934X_OBJS)
