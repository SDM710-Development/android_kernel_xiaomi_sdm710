############ ASoC Codecs ############

obj-$(CONFIG_WCD9XXX_CODEC_CORE) += audio_wcd_core.o
audio_wcd_core-y := wcd9xxx-rst.o wcd9xxx-core-init.o wcd9xxx-core.o	\
		    wcd9xxx-irq.o wcd9xxx-slimslave.o wcd9xxx-utils.o	\
		    wcd9335-regmap.o wcd9335-tables.o			\
		    msm-cdc-pinctrl.o	msm-cdc-supply.o		\
		    wcd934x/wcd934x-regmap.o wcd934x/wcd934x-tables.o

obj-$(CONFIG_SND_SOC_WCD9XXX_V2) += audio_wcd9xxx.o
audio_wcd9xxx-y := wcd9xxx-common-v2.o wcd9xxx-resmgr-v2.o 		\
		   wcdcal-hwdep.o wcd9xxx-soc-init.o wcd-dsp-utils.o	\
		   wcd-dsp-mgr.o
audio_wcd9xxx-$(CONFIG_COMMON_CLK_QCOM) += audio-ext-clk-up.o
audio_wcd9xxx-$(CONFIG_COMMON_CLK_MSM) += audio-ext-clk.o

obj-$(CONFIG_SND_SOC_WCD9335) += audio_wcd9335.o
audio_wcd9335-y := wcd9335.o

obj-$(CONFIG_SND_SOC_WSA881X) += audio_wsa881x.o
audio_wsa881x-y := wsa881x.o wsa881x-tables.o wsa881x-regmap.o		\
		   wsa881x-temp-sensor.o

obj-$(CONFIG_SND_SOC_WSA881X_ANALOG) += audio_wsa881x_analog.o
audio_wsa881x_analog-y := wsa881x-analog.o wsa881x-tables-analog.o	\
			  wsa881x-regmap-analog.o

obj-$(CONFIG_SND_SOC_MSM_STUB) += audio_stub.o
audio_stub-y := msm_stub.o

obj-$(CONFIG_SND_SOC_WCD_CPE) += audio_wcd_cpe.o
audio_wcd_cpe-y := wcd_cpe_core.o wcd_cpe_services.o

obj-$(CONFIG_SND_SOC_WCD_SPI) += audio_wcd_spi.o
audio_wcd_spi-y := wcd-spi.o

obj-$(CONFIG_SND_SOC_WCD_MBHC) += audio_mbhc.o
audio_mbhc-y := wcd-mbhc-v2.o
audio_mbhc-$(CONFIG_SND_SOC_WCD_MBHC_ADC) += wcd-mbhc-adc.o
audio_mbhc-$(CONFIG_SND_SOC_WCD_MBHC_LEGACY) += wcd-mbhc-legacy.o

obj-$(CONFIG_SND_SOC_MSM_HDMI_CODEC_RX) += audio_hdmi.o
audio_hdmi-y := msm_hdmi_codec_rx.o

obj-$(CONFIG_SPK_ID) += audio_spk_id.o
audio_spk_id-y := spk-id.o

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include

obj-y += cs35l41/ msm_sdw/ sdm660_cdc/ tas2557/ tas2562/ wcd934x/
