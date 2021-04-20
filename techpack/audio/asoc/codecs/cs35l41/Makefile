############ CS35L41 ############

# for CS35L41 Codec
ifdef CONFIG_SND_SOC_CS35L41
	CS35L41_OBJS += cs35l41.o
	CS35L41_OBJS += cs35l41-i2c.o
	CS35L41_OBJS += cs35l41-tables.o
	CS35L41_OBJS += wm_adsp.o
endif

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include

# Module information used by KBuild framework
obj-$(CONFIG_SND_SOC_CS35L41) += cs35l41_dlkm.o
cs35l41_dlkm-y := $(CS35L41_OBJS)
