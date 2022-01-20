############ ASoC Drivers ############

# SDM6xx sound card driver
audio_machine-$(CONFIG_SND_SOC_SDM670) += sdm660-common.o

# SDM6xx sound card driver
obj-$(CONFIG_SND_SOC_INT_CODEC) += audio_machine.o
audio_machine-$(CONFIG_SND_SOC_INT_CODEC) += sdm660-internal.o

# SDM6xx sound card driver
obj-$(CONFIG_SND_SOC_EXT_CODEC) += audio_machine.o
audio_machine-$(CONFIG_SND_SOC_EXT_CODEC) += sdm660-external.o	\
					     sdm660-ext-dai-links.o

# SDM845 sound card driver
obj-$(CONFIG_SND_SOC_MACHINE_SDM845) += audio_machine.o
audio_machine-$(CONFIG_SND_SOC_MACHINE_SDM845) += sdm845.o

# SDXPOORWILLS sound card driver
audio_machine-$(CONFIG_SND_SOC_MACHINE_SDXPOORWILLS) += sdxpoorwills.o

# APQ8009 external codec sound card driver
obj-$(CONFIG_SND_SOC_EXT_CODEC_8909) += audio_machine_ext.o
audio_machine_ext-$(CONFIG_SND_SOC_EXT_CODEC_8909) += apq8009-i2s-ext-codec.o

# SDM450 internal codec sound card driver
obj-$(CONFIG_SND_SOC_SDM450) += audio_machine.o
audio_machine-$(CONFIG_SND_SOC_SDM450) += msm8952.o

# MSM8909 internal codec sound card driver
obj-$(CONFIG_SND_SOC_MSM8909) += audio_machine_int.o
audio_machine_int-$(CONFIG_SND_SOC_MSM8909) += msm8952.o

# SDM450 external codec sound card driver
obj-$(CONFIG_SND_SOC_EXT_CODEC_SDM450) += audio_machine_ext.o
audio_machine_ext-$(CONFIG_SND_SOC_EXT_CODEC_SDM450) += msm8952-dai-links.o \
							msm8952-slimbus.o

obj-$(CONFIG_SND_SOC_8909_DIG_CDC) += audio_machine_digcdc.o
audio_machine_digcdc-y := msm8909.o

obj-$(CONFIG_SND_SOC_CPE) += audio_cpe_lsm.o
audio_cpe_lsm-y := msm-cpe-lsm.o

obj-$(CONFIG_SND_SOC_QDSP6V2) += audio_platform.o
audio_platform-y := msm-audio-effects-q6-v2.o msm-compress-q6-v2.o	\
		    msm-dai-fe.o msm-dai-q6-hdmi-v2.o msm-dai-q6-v2.o	\
		    msm-dai-slim.o msm-dai-stub-v2.o msm-lsm-client.o	\
		    msm-pcm-afe-v2.o msm-pcm-dtmf-v2.o			\
		    msm-pcm-hostless.o msm-pcm-host-voice-v2.o		\
		    msm-pcm-loopback-v2.o msm-pcm-q6-noirq.o		\
		    msm-pcm-q6-v2.o msm-pcm-routing-v2.o		\
		    msm-pcm-voice-v2.o msm-pcm-voip-v2.o		\
		    msm-transcode-loopback-q6-v2.o platform_init.o
audio_platform-$(CONFIG_DOLBY_DS2) += msm-ds2-dap-config.o
audio_platform-$(CONFIG_DOLBY_LICENSE) += msm-ds2-dap-config.o
audio_platform-$(CONFIG_SND_HWDEP_ROUTING) += msm-pcm-routing-devdep.o
audio_platform-$(CONFIG_QTI_PP) += msm-qti-pp-config.o
audio_platform-$(CONFIG_MACH_XIAOMI_F3B) += tas2562-algo.o tas2562-calib.o
audio_platform-$(CONFIG_MACH_XIAOMI_F3M) += tas2562-algo.o tas2562-calib.o

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include

obj-y += codecs/
