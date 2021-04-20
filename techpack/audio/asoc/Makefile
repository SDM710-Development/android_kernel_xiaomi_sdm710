############ ASoC Drivers ############


# for SDM6xx sound card driver
ifdef CONFIG_SND_SOC_SDM670
	MACHINE_OBJS += sdm660-common.o
endif

# for SDM6xx sound card driver
ifdef CONFIG_SND_SOC_INT_CODEC
	MACHINE_OBJS += sdm660-internal.o
endif

# for SDM6xx sound card driver
ifdef CONFIG_SND_SOC_EXT_CODEC
	MACHINE_OBJS += sdm660-external.o
	MACHINE_OBJS += sdm660-ext-dai-links.o
endif

# for SDM845 sound card driver
ifdef CONFIG_SND_SOC_MACHINE_SDM845
	MACHINE_OBJS += sdm845.o
endif

# for SDXPOORWILLS sound card driver
ifdef CONFIG_SND_SOC_MACHINE_SDXPOORWILLS
	MACHINE_OBJS += sdxpoorwills.o
endif

# for APQ8009 external codec sound card driver
ifdef CONFIG_SND_SOC_EXT_CODEC_8909
	MACHINE_EXT_OBJS += apq8009-i2s-ext-codec.o
endif

# for SDM450 internal codec sound card driver
ifdef CONFIG_SND_SOC_SDM450
	MACHINE_OBJS += msm8952.o
endif

ifdef CONFIG_SND_SOC_MSM8909
	MACHINE_INT_OBJS += msm8952.o
endif

# for SDM450 external codec sound card driver
ifdef CONFIG_SND_SOC_EXT_CODEC_SDM450
	MACHINE_EXT_OBJS += msm8952-slimbus.o
	MACHINE_EXT_OBJS += msm8952-dai-links.o
endif

# for MSM8909 sound card driver
ifdef CONFIG_SND_SOC_BG_8909
	MACHINE_OBJS += msm_bg.o
endif

ifdef CONFIG_SND_SOC_8909_DIG_CDC
	MACHINE_DIGCDC_OBJS += msm8909.o
endif

ifdef CONFIG_SND_SOC_CPE
	CPE_LSM_OBJS += msm-cpe-lsm.o
endif

ifdef CONFIG_SND_SOC_QDSP6V2
	PLATFORM_OBJS += msm-audio-effects-q6-v2.o
	PLATFORM_OBJS += msm-compress-q6-v2.o
	PLATFORM_OBJS += msm-dai-fe.o
	PLATFORM_OBJS += msm-dai-q6-hdmi-v2.o
	PLATFORM_OBJS += msm-dai-q6-v2.o
	PLATFORM_OBJS += msm-dai-slim.o
	PLATFORM_OBJS += msm-dai-stub-v2.o
	PLATFORM_OBJS += msm-lsm-client.o
	PLATFORM_OBJS += msm-pcm-afe-v2.o
	PLATFORM_OBJS += msm-pcm-dtmf-v2.o
	PLATFORM_OBJS += msm-pcm-hostless.o
	PLATFORM_OBJS += msm-pcm-host-voice-v2.o
	PLATFORM_OBJS += msm-pcm-loopback-v2.o
	PLATFORM_OBJS += msm-pcm-q6-noirq.o
	PLATFORM_OBJS += msm-pcm-q6-v2.o
	PLATFORM_OBJS += msm-pcm-routing-v2.o
	PLATFORM_OBJS += msm-pcm-voice-v2.o
	PLATFORM_OBJS += msm-pcm-voip-v2.o
	PLATFORM_OBJS += msm-transcode-loopback-q6-v2.o
	PLATFORM_OBJS += platform_init.o
endif
ifeq ($(TARGET_PRODUCT), pyxis)
AUDIO_FEATURE_ENABLED_SPKR_PROTECTION := true
endif
ifeq ($(TARGET_PRODUCT), vela)
AUDIO_FEATURE_ENABLED_SPKR_PROTECTION := true
endif
ifeq ($(strip $(AUDIO_FEATURE_ENABLED_SPKR_PROTECTION)),true)
	PLATFORM_OBJS += tas2562-algo.o
	PLATFORM_OBJS += tas2562-calib.o
endif
ifdef CONFIG_DOLBY_DS2
	PLATFORM_OBJS += msm-ds2-dap-config.o
endif
ifdef CONFIG_DOLBY_LICENSE
	PLATFORM_OBJS += msm-ds2-dap-config.o
endif
ifdef CONFIG_SND_HWDEP_ROUTING
	PLATFORM_OBJS += msm-pcm-routing-devdep.o
endif
ifdef CONFIG_QTI_PP
	PLATFORM_OBJS += msm-qti-pp-config.o
endif

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include

ifeq ($(TARGET_PRODUCT), grus)
	CDEFINES += -DCONFIG_SND_SOC_CS35L41_FOR_GRUS
endif

ifeq ($(TARGET_PRODUCT), grus)
	CDEFINES += -DAUDIO_FEATURE_SUPPORT_24BIT
endif

ifeq ($(TARGET_PRODUCT), pyxis)
	CDEFINES += -DAUDIO_FEATURE_SUPPORT_24BIT
endif

ifeq ($(TARGET_PRODUCT), vela)
	CDEFINES += -DAUDIO_FEATURE_SUPPORT_24BIT
endif

ifeq ($(strip $(AUDIO_FEATURE_ENABLED_SPKR_PROTECTION)),true)
	CDEFINES += -DCONFIG_SND_SOC_TAS2562_FOR_PYXIS
endif
KBUILD_CPPFLAGS += $(CDEFINES)

obj-y += codecs/

# Module information used by KBuild framework
obj-$(CONFIG_SND_SOC_QDSP6V2) += platform_dlkm.o
platform_dlkm-y := $(PLATFORM_OBJS)

obj-$(CONFIG_SND_SOC_SDM450) += machine_dlkm.o
machine_dlkm-y := $(MACHINE_OBJS)

obj-$(CONFIG_SND_SOC_MSM8909) += machine_int_dlkm.o
machine_int_dlkm-y := $(MACHINE_INT_OBJS)

obj-$(CONFIG_SND_SOC_EXT_CODEC_8909) += machine_ext_dlkm.o
machine_ext_dlkm-y := $(MACHINE_EXT_OBJS)

obj-$(CONFIG_SND_SOC_EXT_CODEC_SDM450) += machine_ext_dlkm.o
machine_ext_dlkm-y := $(MACHINE_EXT_OBJS)

obj-$(CONFIG_SND_SOC_BG_8909) += machine_dlkm.o
machine_dlkm-y := $(MACHINE_OBJS)

obj-$(CONFIG_SND_SOC_8909_DIG_CDC) += machine_digcdc_dlkm.o
machine_digcdc_dlkm-y := $(MACHINE_DIGCDC_OBJS)

obj-$(CONFIG_SND_SOC_MACHINE_SDM845) += machine_dlkm.o
machine_dlkm-y := $(MACHINE_OBJS)

obj-$(CONFIG_SND_SOC_EXT_CODEC) += machine_dlkm.o
machine_dlkm-y := $(MACHINE_OBJS)

obj-$(CONFIG_SND_SOC_INT_CODEC) += machine_dlkm.o
machine_dlkm-y := $(MACHINE_OBJS)

obj-$(CONFIG_SND_SOC_CPE) += cpe_lsm_dlkm.o
cpe_lsm_dlkm-y := $(CPE_LSM_OBJS)
