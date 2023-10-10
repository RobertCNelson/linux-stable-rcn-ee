# SPDX-License-Identifier: GPL-2.0

ifeq ($(CONFIG_OF_OVERLAY),y)
DTC_FLAGS += -@
endif

dtb-$(CONFIG_SOC_MICROCHIP_POLARFIRE) += mpfs-icicle-kit.dtb
dtb-$(CONFIG_SOC_MICROCHIP_POLARFIRE) += mpfs-icicle-kit-context-a.dtb
dtb-$(CONFIG_SOC_MICROCHIP_POLARFIRE) += mpfs-m100pfsevp.dtb
dtb-$(CONFIG_SOC_MICROCHIP_POLARFIRE) += mpfs-polarberry.dtb
dtb-$(CONFIG_SOC_MICROCHIP_POLARFIRE) += mpfs-video-kit.dtb
dtb-$(CONFIG_SOC_MICROCHIP_POLARFIRE) += mpfs-tysom-m.dtb
dtb-$(CONFIG_SOC_MICROCHIP_POLARFIRE) += mpfs-beaglev-fire.dtb
obj-$(CONFIG_BUILTIN_DTB) += $(addsuffix .o, $(dtb-y))
