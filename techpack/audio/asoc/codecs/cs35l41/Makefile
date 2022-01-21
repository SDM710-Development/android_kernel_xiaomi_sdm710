############ CS35L41 ############

# CS35L41 Codec
obj-$(CONFIG_SND_SOC_CS35L41) += audio_cs35l41.o
audio_cs35l41-y := cs35l41.o cs35l41-i2c.o cs35l41-tables.o wm_adsp.o

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include
