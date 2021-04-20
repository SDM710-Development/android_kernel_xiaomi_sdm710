############ MSM Soundwire ############

# MSM Soundwire Codec
obj-$(CONFIG_SND_SOC_MSM_SDW) += audio_msm_sdw.o
audio_msm_sdw-y := msm_sdw_cdc.o msm_sdw_regmap.o msm-sdw-tables.o	\
		   msm_sdw_cdc_utils.o

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include
