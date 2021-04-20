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

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include

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
