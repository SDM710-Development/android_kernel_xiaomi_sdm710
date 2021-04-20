TARGET_KERNEL_VERSION := 4.9

# These are configurable via Kconfig for kernel-based builds
# Need to explicitly configure for Android-based builds
ifeq ($(CONFIG_ARCH_SDM845), y)
	TARGET_KERNEL_VERSION := 4.9
endif
ifeq ($(CONFIG_ARCH_SDM670), y)
	TARGET_KERNEL_VERSION := 4.9
endif
ifeq ($(CONFIG_ARCH_SDM450), y)
	TARGET_KERNEL_VERSION := 4.9
endif
ifeq ($(CONFIG_ARCH_SDM439), y)
	TARGET_KERNEL_VERSION := 4.9
endif
ifeq ($(CONFIG_ARCH_MSM8909), y)
	TARGET_KERNEL_VERSION := 4.9
endif

ifeq ($(CONFIG_ARCH_MSM8917), y)
	TARGET_KERNEL_VERSION := 4.9
endif
KDIR := $(TOP)/kernel/msm-$(TARGET_KERNEL_VERSION)

AUDIO_ROOT := $(KDIR)/techpack/audio

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

############ IPC ############
ifdef CONFIG_MSM_QDSP6_APRV2
APRV_GLINK += apr.o
APRV_GLINK += apr_v2.o
APRV_GLINK += apr_tal.o
APRV_GLINK += apr_dummy.o
endif

ifdef CONFIG_MSM_QDSP6_APRV3
APRV_GLINK += apr.o
APRV_GLINK += apr_v3.o
APRV_GLINK += apr_tal.o
APRV_GLINK += apr_dummy.o
endif

ifdef CONFIG_MSM_QDSP6_APRV2_GLINK
APRV_GLINK += apr.o
APRV_GLINK += apr_v2.o
APRV_GLINK += apr_tal_glink.o
APRV_GLINK += apr_dummy.o
endif

ifdef CONFIG_MSM_QDSP6_APRV3_GLINK
APRV_GLINK += apr.o
APRV_GLINK += apr_v3.o
APRV_GLINK += apr_tal_glink.o
APRV_GLINK += apr_dummy.o
endif

ifeq ($(CONFIG_ARCH_MSM8909), $(CONFIG_MSM_BGCOM))
	ifdef CONFIG_MSM_BG_GLINK
	BG_GLINK += bg_glink.o
	endif
endif

ifdef CONFIG_WCD_DSP_GLINK
WDSP_GLINK += wcd-dsp-glink.o
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
obj-$(CONFIG_MSM_QDSP6_APRV2) += apr_dlkm.o
obj-$(CONFIG_MSM_QDSP6_APRV3) += apr_dlkm.o
obj-$(CONFIG_MSM_QDSP6_APRV2_GLINK) += apr_dlkm.o
obj-$(CONFIG_MSM_QDSP6_APRV3_GLINK) += apr_dlkm.o
apr_dlkm-y := $(APRV_GLINK)

ifeq ($(CONFIG_ARCH_MSM8909), $(CONFIG_MSM_BGCOM))
	obj-$(CONFIG_MSM_BG_GLINK) += bg_glink_dlkm.o
	bg_glink_dlkm-y := $(BG_GLINK)
endif

obj-$(CONFIG_WCD_DSP_GLINK) += wglink_dlkm.o
wglink_dlkm-y := $(WDSP_GLINK)
