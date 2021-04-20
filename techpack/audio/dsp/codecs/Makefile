############ Native Enc/Dec ############

ifdef CONFIG_MSM_QDSP6V2_CODECS
	NATIVE_OBJS += q6audio_v2.o q6audio_v2_aio.o
	NATIVE_OBJS += audio_utils_aio.o
	NATIVE_OBJS += audio_utils.o
	NATIVE_OBJS += audio_native.o
	NATIVE_OBJS += aac_in.o
	NATIVE_OBJS += amrnb_in.o
	NATIVE_OBJS += amrwb_in.o
	NATIVE_OBJS += audio_aac.o
	NATIVE_OBJS += audio_alac.o
	NATIVE_OBJS += audio_amrnb.o
	NATIVE_OBJS += audio_amrwb.o
	NATIVE_OBJS += audio_amrwbplus.o
	NATIVE_OBJS += audio_ape.o
	NATIVE_OBJS += audio_evrc.o
	NATIVE_OBJS += audio_g711alaw.o
	NATIVE_OBJS += audio_g711mlaw.o
	NATIVE_OBJS += audio_hwacc_effects.o
	NATIVE_OBJS += audio_mp3.o
	NATIVE_OBJS += audio_multi_aac.o
	NATIVE_OBJS += audio_qcelp.o
	NATIVE_OBJS += audio_wma.o
	NATIVE_OBJS += audio_wmapro.o
	NATIVE_OBJS += evrc_in.o
	NATIVE_OBJS += g711alaw_in.o
	NATIVE_OBJS += g711mlaw_in.o
	NATIVE_OBJS += qcelp_in.o
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
obj-$(CONFIG_MSM_QDSP6V2_CODECS) += native_dlkm.o
native_dlkm-y := $(NATIVE_OBJS)
