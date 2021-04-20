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
obj-$(CONFIG_SND_SOC_WCD934X) += wcd934x_dlkm.o
wcd934x_dlkm-y := $(WCD934X_OBJS)
