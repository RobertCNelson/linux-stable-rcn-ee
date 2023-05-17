/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#ifndef __PVR_VM_MIPS_H__
#define __PVR_VM_MIPS_H__

/* Forward declaration from pvr_device.h. */
struct pvr_device;

/* Forward declaration from pvr_gem.h. */
struct pvr_fw_object;

int
pvr_vm_mips_init(struct pvr_device *pvr_dev);
void
pvr_vm_mips_fini(struct pvr_device *pvr_dev);
int
pvr_vm_mips_map(struct pvr_device *pvr_dev, struct pvr_fw_object *fw_obj);
void
pvr_vm_mips_unmap(struct pvr_device *pvr_dev, struct pvr_fw_object *fw_obj);

#endif /* __PVR_VM_MIPS_H__ */
