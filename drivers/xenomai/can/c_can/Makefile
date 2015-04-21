#
#  Makefile for the Bosch C_CAN controller drivers.
#
ifneq ($(VERSION).$(PATCHLEVEL),2.4)

# Makefile frag for Linux v2.6 and v3.x

EXTRA_CFLAGS += -D__IN_XENOMAI__ -Iinclude/xenomai -Idrivers/xenomai/can -Idrivers/xenomai/can/c_can

obj-$(CONFIG_XENO_DRIVERS_CAN_CCAN) += xeno_can_c_can.o

xeno_can_c_can-y := rtcan_c_can.o rtcan_c_can_platform.o

else

# Makefile frag for Linux v2.4

O_TARGET := built-in.o

obj-$(CONFIG_XENO_DRIVERS_CAN_CCAN) += xeno_can_c_can.o

list-multi := xeno_can_c_can.o

xeno_can_c_can-objs := rtcan_c_can.o rtcan_c_can_platform.o


export-objs := $(xeno_can_c_can-objs)

EXTRA_CFLAGS += -D__IN_XENOMAI__ -I$(TOPDIR)/include/xenomai -I$(TOPDIR)/include/xenomai/compat -I..  -I.

include $(TOPDIR)/Rules.make

xeno_can_c_can.o: $(xeno_can_c_can-objs)
	$(LD) -r -o $@ $(xeno_can_c_can-objs)

endif
