.. SPDX-License-Identifier: GPL-2.0-only

============================================================
 accel/thames Driver for the C7x DSPs from Texas Instruments
============================================================

The accel/thames driver supports the C7x DSPs inside some Texas Instruments SoCs
such as the J722S. These can be used as accelerators for various workloads,
including machine learning inference.

This driver controls the power state of the hardware via :doc:`remoteproc </staging/remoteproc>`
and communicates with the firmware running on the DSP via :doc:`rpmsg_virtio </staging/rpmsg_virtio>`.
The kernel driver itself allocates buffers, manages contexts, and submits jobs
to the DSP firmware. Buffers are mapped by the DSP itself using its MMU,
providing memory isolation among different clients.

The source code for the firmware running on the DSP is available at:
https://gitlab.freedesktop.org/tomeu/thames_firmware/.

Everything else is done in userspace, as a Gallium driver (also called thames)
that is part of the Mesa3D project: https://docs.mesa3d.org/teflon.html

If there is more than one core that advertises the same rpmsg_virtio service
name, the driver will load balance jobs between them with drm-gpu-scheduler.

Hardware currently supported:

* J722S
