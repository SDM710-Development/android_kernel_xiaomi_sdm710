############ QDSP6V2 ############

ifdef CONFIG_SND_SOC_MSM_QDSP6V2_INTF
	Q6_OBJS += audio_calibration.o
	Q6_OBJS += audio_cal_utils.o
	Q6_OBJS += q6adm.o
	Q6_OBJS += q6afe.o
	Q6_OBJS += q6asm.o
	Q6_OBJS += q6audio-v2.o
	Q6_OBJS += q6voice.o
	Q6_OBJS += q6core.o
	Q6_OBJS += rtac.o
	Q6_OBJS += q6lsm.o
	Q6_OBJS += audio_slimslave.o
	Q6_OBJS += adsp_err.o
	Q6_OBJS += msm_audio_ion.o
	Q6_OBJS += avtimer.o
	Q6_OBJS += q6_init.o
	Q6_OBJS += apr_elliptic.o
	Q6_OBJS += elliptic/elliptic.o
	Q6_OBJS += elliptic/elliptic_sysfs.o
	Q6_OBJS += elliptic/elliptic_mixer_controls.o
	Q6_OBJS += elliptic/io_modules/msm/elliptic_data_msm_io.o
	Q6_OBJS += elliptic/io_modules/userspace/elliptic_data_userspace_io.o
	Q6_OBJS += elliptic/io_modules/userspace/elliptic_data_userspace_ctrl.o
endif
ifdef CONFIG_DTS_SRS_TM
	Q6_OBJS += msm-dts-srs-tm-config.o
endif

ifdef CONFIG_MSM_ADSP_LOADER
ADSP_LOADER_OBJS += adsp-loader.o
endif

ifeq ($(TARGET_PRODUCT), pyxis)
AUDIO_FEATURE_ENABLED_SPKR_PROTECTION := true
endif

ifeq ($(TARGET_PRODUCT), vela)
AUDIO_FEATURE_ENABLED_SPKR_PROTECTION := true
endif

ifdef CONFIG_MSM_QDSP6_NOTIFIER
QDSP6_NOTIFIER_OBJS += audio_notifier.o audio_ssr.o
endif

ifdef CONFIG_MSM_QDSP6_PDR
QDSP6_PDR_OBJS += audio_pdr.o
endif

ifdef CONFIG_MSM_ULTRASOUND
USF_OBJS += usf.o usfcdev.o q6usm.o
endif

	Q6_OBJS += msm-cirrus-playback.o

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include

ifeq ($(strip $(AUDIO_FEATURE_ENABLED_SPKR_PROTECTION)),true)
	CDEFINES += -DCONFIG_SND_SOC_TAS2562_FOR_PYXIS
endif

KBUILD_CPPFLAGS += $(CDEFINES)

obj-y += codecs/

obj-$(CONFIG_SND_SOC_MSM_QDSP6V2_INTF) += q6_dlkm.o
q6_dlkm-y := $(Q6_OBJS)

obj-$(CONFIG_MSM_ULTRASOUND) += usf_dlkm.o
usf_dlkm-y := $(USF_OBJS)

obj-$(CONFIG_MSM_ADSP_LOADER) += adsp_loader_dlkm.o
adsp_loader_dlkm-y := $(ADSP_LOADER_OBJS)

obj-$(CONFIG_MSM_QDSP6_NOTIFIER) += q6_notifier_dlkm.o
q6_notifier_dlkm-y := $(QDSP6_NOTIFIER_OBJS)

obj-$(CONFIG_MSM_QDSP6_PDR) += q6_pdr_dlkm.o
q6_pdr_dlkm-y := $(QDSP6_PDR_OBJS)
