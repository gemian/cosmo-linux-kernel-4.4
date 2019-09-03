MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/tkcore/include/linux/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/tkcore/armtz/

obj-$(CONFIG_MTK_FINGERPRINT_CHIPSAILING_CS358) += cf_ctl.o cf_spi.o