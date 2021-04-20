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
obj-$(CONFIG_SND_SOC_MSM_SDW) += msm_sdw_dlkm.o
msm_sdw_dlkm-y := $(MSM_SDW_OBJS)
