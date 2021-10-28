/* SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause */
/*
 * Copyright (c) 2021 Microchip Technology Inc. All rights reserved.
 */

#ifndef __DT_BINDINGS_MIV_IHC_H
#define __DT_BINDINGS_MIV_IHC_H

#include "dt-bindings/interrupt-controller/microchip,mpfs-plic.h"

#define IHC_CONTEXT_A	5
#define IHC_CONTEXT_B	6

#define IHC_HART1_INT  PLIC_INT_FABRIC_F2H_62
#define IHC_HART2_INT  PLIC_INT_FABRIC_F2H_61
#define IHC_HART3_INT  PLIC_INT_FABRIC_F2H_60
#define IHC_HART4_INT  PLIC_INT_FABRIC_F2H_59

#endif
