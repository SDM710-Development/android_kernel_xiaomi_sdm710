############ Native Enc/Dec ############

obj-$(CONFIG_MSM_QDSP6V2_CODECS) += audio_native_mod.o
audio_native_mod-y := aac_in.o amrnb_in.o amrwb_in.o audio_aac.o	\
		  audio_alac.o audio_amrnb.o audio_amrwb.o		\
		  audio_amrwbplus.o audio_ape.o audio_evrc.o		\
		  audio_g711alaw.o audio_g711mlaw.o			\
		  audio_hwacc_effects.o audio_mp3.o audio_multi_aac.o	\
		  audio_native.o audio_qcelp.o audio_utils_aio.o	\
		  audio_utils.o audio_wma.o audio_wmapro.o evrc_in.o	\
		  g711alaw_in.o g711mlaw_in.o qcelp_in.o		\
		  q6audio_v2_aio.o q6audio_v2.o

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include
