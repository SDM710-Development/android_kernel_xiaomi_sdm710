############ IPC ############

obj-$(CONFIG_MSM_QDSP6_APRV2) += audio_apr.o
obj-$(CONFIG_MSM_QDSP6_APRV3) += audio_apr.o
obj-$(CONFIG_MSM_QDSP6_APRV2_GLINK) += audio_apr.o
obj-$(CONFIG_MSM_QDSP6_APRV3_GLINK) += audio_apr.o

audio_apr-y := apr.o apr_dummy.o
audio_apr-$(CONFIG_MSM_QDSP6_APRV2) += apr_v2.o apr_tal.o
audio_apr-$(CONFIG_MSM_QDSP6_APRV3) += apr_v3.o apr_tal.o
audio_apr-$(CONFIG_MSM_QDSP6_APRV2_GLINK) += apr_v2.o apr_tal_glink.o
audio_apr-$(CONFIG_MSM_QDSP6_APRV3_GLINK) += apr_v3.o apr_tal_glink.o

obj-$(CONFIG_MSM_BG_GLINK) += audio_bg_glink.o
audio_bg_glink-y := bg_glink.o

obj-$(CONFIG_WCD_DSP_GLINK) += audio_wglink.o
audio_wglink-y := wcd-dsp-glink.o

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include
