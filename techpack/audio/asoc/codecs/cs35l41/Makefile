# These are configurable via Kconfig for kernel-based builds
# Need to explicitly configure for Android-based builds
AUDIO_BLD_DIR := $(ANDROID_BUILD_TOP)/kernel/msm-4.9
AUDIO_ROOT := $(AUDIO_BLD_DIR)/techpack/audio

# As per target team, build is done as follows:
# Defconfig : build with default flags
# Slub      : defconfig  + CONFIG_SLUB_DEBUG := y +
#	      CONFIG_SLUB_DEBUG_ON := y + CONFIG_PAGE_POISONING := y
# Perf      : Using appropriate msmXXXX-perf_defconfig
#
# Shipment builds (user variants) should not have any debug feature
# enabled. This is identified using 'TARGET_BUILD_VARIANT'. Slub builds
# are identified using the CONFIG_SLUB_DEBUG_ON configuration. Since
# there is no other way to identify defconfig builds, QTI internal
# representation of perf builds (identified using the string 'perf'),
# is used to identify if the build is a slub or defconfig one. This
# way no critical debug feature will be enabled for perf and shipment
# builds. Other OEMs are also protected using the TARGET_BUILD_VARIANT
# config.

############ UAPI ############
UAPI_DIR :=	uapi
UAPI_INC :=	-I$(AUDIO_ROOT)/include/$(UAPI_DIR)

############ COMMON ############
COMMON_DIR :=	include
COMMON_INC :=	-I$(AUDIO_ROOT)/$(COMMON_DIR)

############ CS35L41 ############

# for CS35L41 Codec
ifdef CONFIG_SND_SOC_CS35L41
	CS35L41_OBJS += cs35l41.o
	CS35L41_OBJS += cs35l41-i2c.o
	CS35L41_OBJS += cs35l41-tables.o
	CS35L41_OBJS += wm_adsp.o
endif

INCS +=		$(COMMON_INC) \
		$(UAPI_INC)

EXTRA_CFLAGS += $(INCS)

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
obj-$(CONFIG_SND_SOC_CS35L41) += cs35l41_dlkm.o
cs35l41_dlkm-y := $(CS35L41_OBJS)

# inject some build related information
DEFINES += -DBUILD_TIMESTAMP=\"$(shell date -u +'%Y-%m-%dT%H:%M:%SZ')\"
