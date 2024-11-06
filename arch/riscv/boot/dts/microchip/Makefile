# SPDX-License-Identifier: GPL-2.0

ifeq ($(CONFIG_OF_OVERLAY),y)
DTC_FLAGS += -@
endif

dtb-$(CONFIG_ARCH_MICROCHIP_POLARFIRE) += mpfs-beaglev-fire.dtb
dtb-$(CONFIG_ARCH_MICROCHIP_POLARFIRE) += mpfs-disco-kit.dtb
dtb-$(CONFIG_ARCH_MICROCHIP_POLARFIRE) += mpfs-icicle-kit.dtb
dtb-$(CONFIG_ARCH_MICROCHIP_POLARFIRE) += mpfs-m100pfsevp-sdcard.dtb
dtb-$(CONFIG_ARCH_MICROCHIP_POLARFIRE) += mpfs-m100pfsevp-emmc.dtb
dtb-$(CONFIG_ARCH_MICROCHIP_POLARFIRE) += mpfs-polarberry.dtb
dtb-$(CONFIG_ARCH_MICROCHIP_POLARFIRE) += mpfs-video-kit.dtb
dtb-$(CONFIG_ARCH_MICROCHIP_POLARFIRE) += mpfs-tysom-m.dtb
dtb-$(CONFIG_ARCH_MICROCHIP_POLARFIRE) += pic64gx-curiosity-kit.dtb
obj-$(CONFIG_BUILTIN_DTB) += $(addsuffix .o, $(dtb-y))
