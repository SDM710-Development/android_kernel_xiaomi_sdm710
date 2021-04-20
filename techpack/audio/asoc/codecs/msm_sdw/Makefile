############ MSM Soundwire ############

# for MSM Soundwire Codec
ifdef CONFIG_SND_SOC_MSM_SDW
	MSM_SDW_OBJS += msm_sdw_cdc.o
	MSM_SDW_OBJS += msm_sdw_regmap.o
	MSM_SDW_OBJS += msm-sdw-tables.o
	MSM_SDW_OBJS += msm_sdw_cdc_utils.o
endif

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include

# Module information used by KBuild framework
obj-$(CONFIG_SND_SOC_MSM_SDW) += msm_sdw_dlkm.o
msm_sdw_dlkm-y := $(MSM_SDW_OBJS)
