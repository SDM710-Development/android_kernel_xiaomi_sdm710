############ SDM660_CDC ############

# SDM660_CDC Codec
obj-$(CONFIG_SND_SOC_ANALOG_CDC) += audio_analog_cdc.o
audio_analog_cdc-y := msm-analog-cdc.o sdm660-cdc-irq.o

obj-$(CONFIG_SND_SOC_DIGITAL_CDC) += audio_digital_cdc.o
obj-$(CONFIG_SND_SOC_DIGITAL_CDC_LEGACY) += audio_digital_cdc.o
audio_digital_cdc-$(CONFIG_SND_SOC_DIGITAL_CDC) += msm-digital-cdc.o	\
						   msm-digital-cdc-regmap.o
audio_digital_cdc-$(CONFIG_SND_SOC_DIGITAL_CDC_LEGACY) +=		\
					msm-digital-cdc-legacy.o	\
					msm-digital-cdc-legacy-regmap.o

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include
