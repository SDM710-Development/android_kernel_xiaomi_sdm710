############ SDM660_CDC ############

# for SDM660_CDC Codec
ifdef CONFIG_SND_SOC_ANALOG_CDC
	ANALOG_CDC_OBJS += msm-analog-cdc.o
	ANALOG_CDC_OBJS += sdm660-cdc-irq.o
endif

ifdef CONFIG_SND_SOC_DIGITAL_CDC
	DIGITAL_CDC_OBJS += msm-digital-cdc.o
	DIGITAL_CDC_OBJS += msm-digital-cdc-regmap.o
endif

ifdef CONFIG_SND_SOC_DIGITAL_CDC_LEGACY
	DIGITAL_CDC_OBJS += msm-digital-cdc-legacy.o
	DIGITAL_CDC_OBJS += msm-digital-cdc-legacy-regmap.o
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
obj-$(CONFIG_SND_SOC_ANALOG_CDC) += analog_cdc_dlkm.o
analog_cdc_dlkm-y := $(ANALOG_CDC_OBJS)

obj-$(CONFIG_SND_SOC_DIGITAL_CDC) += digital_cdc_dlkm.o
digital_cdc_dlkm-y := $(DIGITAL_CDC_OBJS)

obj-$(CONFIG_SND_SOC_DIGITAL_CDC_LEGACY) += digital_cdc_dlkm.o
digital_cdc_dlkm-y := $(DIGITAL_CDC_OBJS)
