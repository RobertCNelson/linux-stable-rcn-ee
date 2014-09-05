/****************************************************************************
*
*    Copyright (C) 2005 - 2012 by Vivante Corp.
*
*    This program is free software; you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation; either version 2 of the license, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not write to the Free Software
*    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*
*****************************************************************************/

#ifndef __gc_hal_kernel_device_h_
#define __gc_hal_kernel_device_h_

/******************************************************************************\
******************************* gckGALDEVICE Structure *******************************
\******************************************************************************/

typedef struct _gckGALDEVICE
{
    /* Objects. */
    gckOS               os;
    gckKERNEL           kernels[gcdCORE_COUNT];

    /* Attributes. */
    size_t              internalSize;
    gctPHYS_ADDR        internalPhysical;
    void *              internalLogical;
    gckVIDMEM           internalVidMem;
    size_t              externalSize;
    gctPHYS_ADDR        externalPhysical;
    void *              externalLogical;
    gckVIDMEM           externalVidMem;
    gckVIDMEM           contiguousVidMem;
    void *              contiguousBase;
    gctPHYS_ADDR        contiguousPhysical;
    size_t              contiguousSize;
    int                 contiguousMapped;
    void *              contiguousMappedUser;
    size_t              systemMemorySize;
    u32                 systemMemoryBaseAddress;
    void *              registerBases[gcdCORE_COUNT];
    size_t              registerSizes[gcdCORE_COUNT];
    u32                 baseAddress;
    u32                 requestedRegisterMemBases[gcdCORE_COUNT];
    size_t              requestedRegisterMemSizes[gcdCORE_COUNT];
    u32                 requestedContiguousBase;
    size_t              requestedContiguousSize;

    /* IRQ management. */
    int                 irqLines[gcdCORE_COUNT];
    int                 isrInitializeds[gcdCORE_COUNT];
    int                 dataReadys[gcdCORE_COUNT];

    /* Thread management. */
    struct task_struct  *threadCtxts[gcdCORE_COUNT];
    struct semaphore    semas[gcdCORE_COUNT];
    int                 threadInitializeds[gcdCORE_COUNT];
    int                 killThread;

    /* Signal management. */
    int                 signal;

    /* Core mapping */
    gceCORE             coreMapping[8];

    /* States before suspend. */
    gceCHIPPOWERSTATE   statesStored[gcdCORE_COUNT];

    /* Clock management. */
    struct clk          *clk_gpu;
    int                 clk_gpu_enabled;
    struct clk		*clk_2d;
    int			clk_2d_enabled;

    /* Device pointer for dma_alloc_coherent */
    struct device       *dev;
}
* gckGALDEVICE;

/*
 * Helper structure for constructing a gckGALDEVICE. This structure reduces
 * the number of parameters to gckGALDEVICE_CONSTRUCT and allows for future
 * enhancement, such as the GC355 2D vector accelerator.
 */
typedef struct _gckDEVICE_CONSTRUCT
{
    int			IrqLine;
    u32			RegisterMemBase;
    size_t		RegisterMemSize;
    int			IrqLine2D;
    u32			RegisterMemBase2D;
    size_t		RegisterMemSize2D;
    u32			ContiguousBase;
    size_t		ContiguousSize;
    size_t		BankSize;
    int			FastClear;
    int			Compression;
    u32			PhysBaseAddr;
    u32			PhysSize;
    int			Signal;
}
gckDEVICE_CONSTRUCT;

typedef struct _gcsHAL_PRIVATE_DATA
{
    gckGALDEVICE        device;
    void *              mappedMemory;
    void *              contiguousLogical;
    /* The process opening the device may not be the same as the one that closes it. */
    u32                 pidOpen;
}
gcsHAL_PRIVATE_DATA, * gcsHAL_PRIVATE_DATA_PTR;

gceSTATUS gckGALDEVICE_Start(
    IN gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Stop(
    gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Construct(
    IN gckDEVICE_CONSTRUCT *chelp,
    OUT gckGALDEVICE *Device
    );

gceSTATUS gckGALDEVICE_Destroy(
    IN gckGALDEVICE Device
    );

#endif /* __gc_hal_kernel_device_h_ */
