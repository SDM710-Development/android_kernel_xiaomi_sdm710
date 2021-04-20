############ ASoC Codecs ############
ifdef CONFIG_WCD9XXX_CODEC_CORE
	CORE_OBJS += wcd9xxx-rst.o
	CORE_OBJS += wcd9xxx-core-init.o
	CORE_OBJS += wcd9xxx-core.o
	CORE_OBJS += wcd9xxx-irq.o
	CORE_OBJS += wcd9xxx-slimslave.o
	CORE_OBJS += wcd9xxx-utils.o
	CORE_OBJS += wcd9335-regmap.o
	CORE_OBJS += wcd9335-tables.o
	CORE_OBJS += msm-cdc-pinctrl.o
	CORE_OBJS += msm-cdc-supply.o
	CORE_OBJS += wcd934x/wcd934x-regmap.o
	CORE_OBJS += wcd934x/wcd934x-tables.o
endif

ifdef CONFIG_SND_SOC_WCD9XXX_V2
	WCD9XXX_OBJS += wcd9xxx-common-v2.o
	WCD9XXX_OBJS += wcd9xxx-resmgr-v2.o
	WCD9XXX_OBJS += wcdcal-hwdep.o
	WCD9XXX_OBJS += wcd9xxx-soc-init.o
	WCD9XXX_OBJS += wcd-dsp-utils.o
	WCD9XXX_OBJS += wcd-dsp-mgr.o
endif

ifdef CONFIG_COMMON_CLK_QCOM
	WCD9XXX_OBJS += audio-ext-clk-up.o
endif

ifdef CONFIG_COMMON_CLK_MSM
	WCD9XXX_OBJS += audio-ext-clk.o
endif

ifdef CONFIG_SND_SOC_WCD9335
	WCD9335_OBJS += wcd9335.o
endif

ifdef CONFIG_SND_SOC_WSA881X
	WSA881X_OBJS += wsa881x.o
	WSA881X_OBJS += wsa881x-tables.o
	WSA881X_OBJS += wsa881x-regmap.o
	WSA881X_OBJS += wsa881x-temp-sensor.o
endif

ifdef CONFIG_SND_SOC_WSA881X_ANALOG
	WSA881X_ANALOG_OBJS += wsa881x-analog.o
	WSA881X_ANALOG_OBJS += wsa881x-tables-analog.o
	WSA881X_ANALOG_OBJS += wsa881x-regmap-analog.o
endif

ifdef CONFIG_SND_SOC_MSM_STUB
	STUB_OBJS += msm_stub.o
endif
ifdef CONFIG_SND_SOC_WCD_SPI
	SPI_OBJS += wcd-spi.o
endif

ifdef CONFIG_SND_SOC_WCD_CPE
	WCD_CPE_OBJS += wcd_cpe_core.o
	WCD_CPE_OBJS += wcd_cpe_services.o
endif

ifdef CONFIG_SND_SOC_WCD_MBHC
	MBHC_OBJS += wcd-mbhc-v2.o
endif

ifdef CONFIG_SND_SOC_WCD_MBHC_ADC
	MBHC_OBJS += wcd-mbhc-adc.o
endif

ifdef CONFIG_SND_SOC_WCD_MBHC_LEGACY
	MBHC_OBJS += wcd-mbhc-legacy.o
endif

ifdef CONFIG_SND_SOC_MSM_HDMI_CODEC_RX
	HDMICODEC_OBJS += msm_hdmi_codec_rx.o
endif

ifdef CONFIG_SPK_ID
	SPK_OBJS += spk-id.o
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

obj-y += cs35l41/ msm_sdw/ sdm660_cdc/ tas2557/ tas2562/ wcd934x/

# Module information used by KBuild framework
obj-$(CONFIG_WCD9XXX_CODEC_CORE) += wcd_core_dlkm.o
wcd_core_dlkm-y := $(CORE_OBJS)

obj-$(CONFIG_SND_SOC_WCD9XXX_V2) += wcd9xxx_dlkm.o
wcd9xxx_dlkm-y := $(WCD9XXX_OBJS)

obj-$(CONFIG_SND_SOC_WCD9335) += wcd9335_dlkm.o
wcd9335_dlkm-y := $(WCD9335_OBJS)

obj-$(CONFIG_SND_SOC_WSA881X) += wsa881x_dlkm.o
wsa881x_dlkm-y := $(WSA881X_OBJS)

obj-$(CONFIG_SND_SOC_WSA881X_ANALOG) += wsa881x_analog_dlkm.o
wsa881x_analog_dlkm-y := $(WSA881X_ANALOG_OBJS)

obj-$(CONFIG_SND_SOC_MSM_STUB) += stub_dlkm.o
stub_dlkm-y := $(STUB_OBJS)

obj-$(CONFIG_SND_SOC_WCD_CPE) += wcd_cpe_dlkm.o
wcd_cpe_dlkm-y := $(WCD_CPE_OBJS)

obj-$(CONFIG_SND_SOC_WCD_SPI) += wcd_spi_dlkm.o
wcd_spi_dlkm-y := $(SPI_OBJS)

obj-$(CONFIG_SPK_ID)    += spk_id_dlkm.o
spk_id_dlkm-y := $(SPK_OBJS)

obj-$(CONFIG_SND_SOC_WCD_MBHC) += mbhc_dlkm.o
mbhc_dlkm-y := $(MBHC_OBJS)

obj-$(CONFIG_SND_SOC_MSM_HDMI_CODEC_RX) += hdmi_dlkm.o
hdmi_dlkm-y := $(HDMICODEC_OBJS)
