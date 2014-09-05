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




#include <linux/device.h>
#include <linux/slab.h>
#if defined(CONFIG_PREEMPT)
#include <linux/kernel_lock.h>
#endif
#if defined(CONFIG_OF)
#include <linux/of.h>
#include <linux/of_platform.h>
#endif

#include "gc_hal_kernel_linux.h"

/* Platform driver vs. hardwired configuration. Platform driver is preferred if
 * CONFIG_OF is set */
#ifndef USE_PLATFORM_DRIVER
#if !defined(JZSOC) || defined(CONFIG_OF)
#define USE_PLATFORM_DRIVER 1
#else
/* Assume hardwired configuration, e.g., JZSOC */
#define USE_PLATFORM_DRIVER 0
#endif
#endif

#if USE_PLATFORM_DRIVER
#   include <linux/platform_device.h>
#endif

#ifdef CONFIG_PXA_DVFM
#   include <mach/dvfm.h>
#   include <mach/pxa3xx_dvfm.h>
#endif

/* Zone used for header/footer. */
#define _GC_OBJ_ZONE    gcvZONE_DRIVER

MODULE_DESCRIPTION("Vivante Graphics Driver");
MODULE_LICENSE("GPL");

static struct class* gpuClass;

static gckGALDEVICE galDevice;

static int major = 199;
module_param(major, int, 0644);

#ifdef CONFIG_MACH_JZ4770
#include <asm/mach-jz4770/jz4770cpm.h>

#ifndef IRQ_GPU
#define IRQ_GPU 6
#endif
#ifndef GPU_BASE
#define GPU_BASE 0x13040000
#endif
#ifndef JZ_GPU_MEM_BASE
#define JZ_GPU_MEM_BASE 0       /* if GPU_MEM_BASE = 0, alloc gpu memory dynamicly on bootup */
#endif
#ifndef JZ_GPU_MEM_SIZE
#define JZ_GPU_MEM_SIZE 0x400000    /* set default reserved memory 4M Bytes. */
#endif

static int irqLine = IRQ_GPU;
module_param(irqLine, int, 0644);

static long registerMemBase = GPU_BASE;
module_param(registerMemBase, long, 0644);

static ulong registerMemSize = 256 << 10;
module_param(registerMemSize, ulong, 0644);

static int irqLine2D = -1;
module_param(irqLine2D, int, 0644);

static long registerMemBase2D = 0x00000000;
module_param(registerMemBase2D, long, 0644);

static ulong registerMemSize2D = 256 << 10;
module_param(registerMemSize2D, ulong, 0644);

static long contiguousSize = JZ_GPU_MEM_SIZE;
module_param(contiguousSize, long, 0644);

static ulong contiguousBase = JZ_GPU_MEM_BASE;
module_param(contiguousBase, ulong, 0644);

#else /* CONFIG_MACH_JZ4770 */

static int irqLine = -1;
module_param(irqLine, int, 0644);

static long registerMemBase = 0x80000000;
module_param(registerMemBase, long, 0644);

static ulong registerMemSize = 256 << 10;
module_param(registerMemSize, ulong, 0644);

static int irqLine2D = -1;
module_param(irqLine2D, int, 0644);

static long registerMemBase2D = 0x00000000;
module_param(registerMemBase2D, long, 0644);

static ulong registerMemSize2D = 256 << 10;
module_param(registerMemSize2D, ulong, 0644);

static long contiguousSize = 4 << 20;
module_param(contiguousSize, long, 0644);

static ulong contiguousBase = 0;
module_param(contiguousBase, ulong, 0644);
#endif  /* CONFIG_MACH_JZ4770 */

static long bankSize = 32 << 20;
module_param(bankSize, long, 0644);

static int fastClear = -1;
module_param(fastClear, int, 0644);

static int compression = -1;
module_param(compression, int, 0644);

static int signal = 48;
module_param(signal, int, 0644);

static ulong baseAddress = 0;
module_param(baseAddress, ulong, 0644);

static ulong physSize = 0;
module_param(physSize, ulong, 0644);

static int showArgs = 1;
module_param(showArgs, int, 0644);

static int drv_open(
    struct inode* inode,
    struct file* filp
    );

static int drv_release(
    struct inode* inode,
    struct file* filp
    );

static long drv_ioctl(
    struct file* filp,
    unsigned int ioctlCode,
    unsigned long arg
    );

static int drv_mmap(
    struct file* filp,
    struct vm_area_struct* vma
    );

static struct file_operations driver_fops =
{
    .open       = drv_open,
    .release    = drv_release,
    .unlocked_ioctl = drv_ioctl,
    .mmap       = drv_mmap,
};

int drv_open(
    struct inode* inode,
    struct file* filp
    )
{
    gceSTATUS status;
    int attached = gcvFALSE;
    gcsHAL_PRIVATE_DATA_PTR data = NULL;
    int i;

    gcmkHEADER_ARG("inode=0x%08X filp=0x%08X", inode, filp);

    if (filp == NULL)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): filp is NULL\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    data = kmalloc(sizeof(gcsHAL_PRIVATE_DATA), GFP_KERNEL | __GFP_NOWARN);

    if (data == NULL)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): private_data is NULL\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    data->device             = galDevice;
    data->mappedMemory       = NULL;
    data->contiguousLogical  = NULL;
    data->pidOpen            = task_tgid_vnr(current);

    /* Attached the process. */
    for (i = 0; i < gcdCORE_COUNT; i++)
    {
        if (galDevice->kernels[i] != NULL)
        {
            gcmkONERROR(gckKERNEL_AttachProcess(galDevice->kernels[i], gcvTRUE));
        }
    }
    attached = gcvTRUE;

    if (!galDevice->contiguousMapped)
    {
        gcmkONERROR(gckOS_MapMemory(
            galDevice->os,
            galDevice->contiguousPhysical,
            galDevice->contiguousSize,
            &data->contiguousLogical
            ));

        for (i = 0; i < gcdCORE_COUNT; i++)
        {
            if (galDevice->kernels[i] != NULL)
            {
                gcmkVERIFY_OK(gckKERNEL_AddProcessDB(
                    galDevice->kernels[i],
                    data->pidOpen,
                    gcvDB_MAP_MEMORY,
                    data->contiguousLogical,
                    galDevice->contiguousPhysical,
                    galDevice->contiguousSize));
            }
        }
    }

    filp->private_data = data;

    /* Success. */
    gcmkFOOTER_NO();
    return 0;

OnError:
    if (data != NULL)
    {
        if (data->contiguousLogical != NULL)
        {
            gcmkVERIFY_OK(gckOS_UnmapMemory(
                galDevice->os,
                galDevice->contiguousPhysical,
                galDevice->contiguousSize,
                data->contiguousLogical
                ));
        }

        kfree(data);
    }

    if (attached)
    {
        for (i = 0; i < gcdCORE_COUNT; i++)
        {
            if (galDevice->kernels[i] != NULL)
            {
                gcmkVERIFY_OK(gckKERNEL_AttachProcess(galDevice->kernels[i], gcvFALSE));
            }
        }
    }

    gcmkFOOTER();
    return -ENOTTY;
}

int drv_release(
    struct inode* inode,
    struct file* filp
    )
{
    gceSTATUS status;
    gcsHAL_PRIVATE_DATA_PTR data;
    gckGALDEVICE device;
    int i;
    u32 processID;


    gcmkHEADER_ARG("inode=0x%08X filp=0x%08X", inode, filp);

    if (filp == NULL)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): filp is NULL\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    data = filp->private_data;

    if (data == NULL)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): private_data is NULL\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    device = data->device;

    if (device == NULL)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): device is NULL\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    if (!device->contiguousMapped)
    {
        if (data->contiguousLogical != NULL)
        {
	    processID = task_tgid_vnr(current);
            gcmkONERROR(gckOS_UnmapMemoryEx(
                galDevice->os,
                galDevice->contiguousPhysical,
                galDevice->contiguousSize,
                data->contiguousLogical,
                data->pidOpen
                ));

            for (i = 0; i < gcdCORE_COUNT; i++)
            {
                if (galDevice->kernels[i] != NULL)
                {
                    gcmkVERIFY_OK(
                         gckKERNEL_RemoveProcessDB(galDevice->kernels[i],
                                                   processID, gcvDB_MAP_MEMORY,
                                                   data->contiguousLogical));
                }
            }

            data->contiguousLogical = NULL;
        }
    }

    /* Clean user signals if exit unnormally. */
    processID = task_tgid_vnr(current);
    gcmkVERIFY_OK(gckOS_CleanProcessSignal(galDevice->os, (gctHANDLE)processID));

    /* A process gets detached. */
    for (i = 0; i < gcdCORE_COUNT; i++)
    {
        if (galDevice->kernels[i] != NULL)
        {
            gcmkONERROR(gckKERNEL_AttachProcessEx(galDevice->kernels[i], gcvFALSE, data->pidOpen));
        }
    }

    kfree(data);
    filp->private_data = NULL;

    /* Success. */
    gcmkFOOTER_NO();
    return 0;

OnError:
    gcmkFOOTER();
    return -ENOTTY;
}

long drv_ioctl(
    struct file* filp,
    unsigned int ioctlCode,
    unsigned long arg
    )
{
    gceSTATUS status;
    gcsHAL_INTERFACE iface;
    u32 copyLen;
    DRIVER_ARGS drvArgs;
    gckGALDEVICE device;
    gcsHAL_PRIVATE_DATA_PTR data;
    s32 i, count;

#if defined(JZSOC) && defined(CONFIG_PREEMPT)
    /* 1: lock_kernel, fix bug WOWFish. */
    lock_kernel();
#endif
    gcmkHEADER_ARG(
        "filp=0x%08X ioctlCode=0x%08X arg=0x%08X",
        filp, ioctlCode, arg
        );

    if (filp == NULL)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): filp is NULL\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    data = filp->private_data;

    if (data == NULL)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): private_data is NULL\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    device = data->device;

    if (device == NULL)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): device is NULL\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    if ((ioctlCode != IOCTL_GCHAL_INTERFACE)
    &&  (ioctlCode != IOCTL_GCHAL_KERNEL_INTERFACE)
    )
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): unknown command %d\n",
            __FUNCTION__, __LINE__,
            ioctlCode
            );

        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    /* Get the drvArgs. */
    copyLen = copy_from_user(
        &drvArgs, (void *) arg, sizeof(DRIVER_ARGS)
        );

    if (copyLen != 0)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): error copying of the input arguments.\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    /* Now bring in the gcsHAL_INTERFACE structure. */
    if ((drvArgs.InputBufferSize  != sizeof(gcsHAL_INTERFACE))
    ||  (drvArgs.OutputBufferSize != sizeof(gcsHAL_INTERFACE))
    )
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): error copying of the input arguments.\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    copyLen = copy_from_user(
        &iface, drvArgs.InputBuffer, sizeof(gcsHAL_INTERFACE)
        );

    if (copyLen != 0)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): error copying of input HAL interface.\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    if (iface.command == gcvHAL_CHIP_INFO)
    {
        count = 0;
        for (i = 0; i < gcdCORE_COUNT; i++)
        {
            if (device->kernels[i] != NULL)
            {
                gcmkVERIFY_OK(gckHARDWARE_GetType(device->kernels[i]->hardware,
                                                  &iface.u.ChipInfo.types[count]));

                count++;
            }
        }

        iface.u.ChipInfo.count = count;
        status = gcvSTATUS_OK;
    }
    else
    {
        if (iface.hardwareType < 0 || iface.hardwareType > 7)
        {
            gcmkTRACE_ZONE(
                gcvLEVEL_ERROR, gcvZONE_DRIVER,
                "%s(%d): unknown hardwareType %d\n",
                __FUNCTION__, __LINE__,
                iface.hardwareType
                );

            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }

        status = gckKERNEL_Dispatch(device->kernels[device->coreMapping[iface.hardwareType]],
                                    (ioctlCode == IOCTL_GCHAL_INTERFACE),
                                    &iface);
    }

    /* Redo system call after pending signal is handled. */
    if (status == gcvSTATUS_INTERRUPTED)
    {
        gcmkFOOTER();
        return -ERESTARTSYS;
    }

    if (gcmIS_SUCCESS(status) && (iface.command == gcvHAL_LOCK_VIDEO_MEMORY))
    {
        /* Special case for mapped memory. */
        if ((data->mappedMemory != NULL)
        &&  (iface.u.LockVideoMemory.node->VidMem.memory->object.type == gcvOBJ_VIDMEM)
        )
        {
            /* Compute offset into mapped memory. */
            u32 offset
                = (u8 *) iface.u.LockVideoMemory.memory
                - (u8 *) device->contiguousBase;

            /* Compute offset into user-mapped region. */
            iface.u.LockVideoMemory.memory =
                (u8 *) data->mappedMemory + offset;
        }
    }

    /* Copy data back to the user. */
    copyLen = copy_to_user(
        drvArgs.OutputBuffer, &iface, sizeof(gcsHAL_INTERFACE)
        );

    if (copyLen != 0)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): error copying of output HAL interface.\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    /* Success. */
    gcmkFOOTER_NO();
#if defined(JZSOC) && defined(CONFIG_PREEMPT)
    /* 1: lock_kernel, fix bug WOWFish. */
    unlock_kernel();
#endif
    return 0;

OnError:
    gcmkFOOTER();
#if defined(JZSOC) && defined(CONFIG_PREEMPT)
    /* 1: lock_kernel, fix bug WOWFish. */
    unlock_kernel();
#endif
    return -ENOTTY;
}

static int drv_mmap(
    struct file* filp,
    struct vm_area_struct* vma
    )
{
    gceSTATUS status;
    gcsHAL_PRIVATE_DATA_PTR data;
    gckGALDEVICE device;

    gcmkHEADER_ARG("filp=0x%08X vma=0x%08X", filp, vma);

    if (filp == NULL)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): filp is NULL\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    data = filp->private_data;

    if (data == NULL)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): private_data is NULL\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    device = data->device;

    if (device == NULL)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): device is NULL\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

#if !gcdPAGED_MEMORY_CACHEABLE
    vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
    vma->vm_flags    |= VM_IO | VM_DONTCOPY | VM_DONTEXPAND;
#endif
    vma->vm_pgoff     = 0;

    if (device->contiguousMapped)
    {
        unsigned long size = vma->vm_end - vma->vm_start;

        int ret = io_remap_pfn_range(
            vma,
            vma->vm_start,
            (u32) device->contiguousPhysical >> PAGE_SHIFT,
            size,
            vma->vm_page_prot
            );

        if (ret != 0)
        {
            gcmkTRACE_ZONE(
                gcvLEVEL_ERROR, gcvZONE_DRIVER,
                "%s(%d): io_remap_pfn_range failed %d\n",
                __FUNCTION__, __LINE__,
                ret
                );

            data->mappedMemory = NULL;

            gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
        }

        data->mappedMemory = (void *) vma->vm_start;
    }

    /* Success. */
    gcmkFOOTER_NO();
    return 0;

OnError:
    gcmkFOOTER();
    return -ENOTTY;
}

#ifdef CONFIG_JZSOC
static void enable_jzsoc_gpu_clock(void)
{
#ifdef CONFIG_MACH_JZ4770
    {
        /* JZ4770 GPU CLK2x 100MHz -- 500MHz */
#define GPU_CLK_MAX 500000000
        unsigned int GPUCDR_VAL=0;
        int div;
        int gpu_use_pll1 = 1;
        unsigned int pll_clk;
        unsigned int gpu_clk = 0;

        /* Right now: hardcode PLL0.
	 * Later: use generic clock interface.
        pll_clk = cpm_get_pllout1();
        if ( pll_clk == 0 )*/ {
            gpu_use_pll1 = 0;   /* use pll0 */
            pll_clk = cpm_get_pllout();
            if ((INREG32(CPM_CPCCR) & CPCCR_PCS) != 0 )
            pll_clk /= 2;
        }

        for ( div=1; div <= ((GPUCDR_GPUDIV_MASK>>GPUCDR_GPUDIV_LSB)+1); div++ ) {
            gpu_clk = pll_clk/div;
            if ( gpu_clk < GPU_CLK_MAX )
                break;
        }

        cpm_stop_clock(CGM_GPU);
        GPUCDR_VAL = (div-1);
        if (gpu_use_pll1)
            GPUCDR_VAL |= 1<<31;
        REG_CPM_GPUCDR = GPUCDR_VAL;
        cpm_start_clock(CGM_GPU);

        printk("REG_CPM_GPUCDR= 0x%08x\n", GPUCDR_VAL);
        printk("GPU CLOCK USE PLL%d\n", gpu_use_pll1);
        printk("GPU GPU_CLK2x= %d MHz\n", gpu_clk/1000000);
    }
#endif
}
#endif

#if !USE_PLATFORM_DRIVER
static int __init drv_init(void)
{
    gckDEVICE_CONSTRUCT chelp = {
	.IrqLine = irqLine,
	.RegisterMemBase = registerMemBase,
	.RegisterMemSize = registerMemSize,
	.IrqLine2D = irqLine2D,
	.RegisterMemBase2D = registerMemBase2D,
	.RegisterMemSize2D = registerMemSize2D,
	.ContiguousBase = contiguousBase,
	.ContiguousSize = contiguousSize,
	.BankSize = bankSize,
	.FastClear = fastClear,
	.Compression = compression,
	.PhysBaseAddr = baseAddress,
	.PhysSize = physSize,
	.Signal = signal
    };

    return drv_init_chelp(&chelp);
}
#endif

static int drv_init_chelp(IN gckDEVICE_CONSTRUCT *chelp)
{
    int ret;
    int result = -EINVAL;
    gceSTATUS status;
    gckGALDEVICE device = NULL;
    struct class* device_class = NULL;

    gcmkHEADER();

#ifdef CONFIG_JZSOC
    enable_jzsoc_gpu_clock();
#endif

    if (showArgs)
    {
        printk("galcore options:\n");
        printk("  irqLine           = %d\n",     chelp->IrqLine);
        printk("  registerMemBase   = 0x%08X\n", chelp->RegisterMemBase);
        printk("  registerMemSize   = 0x%08X\n", chelp->RegisterMemSize);

        if (chelp->IrqLine2D != -1)
        {
            printk("  irqLine2D         = %d\n",     chelp->IrqLine2D);
            printk("  registerMemBase2D = 0x%08X\n", chelp->RegisterMemBase2D);
            printk("  registerMemSize2D = 0x%08X\n", chelp->RegisterMemSize2D);
        }

        printk("  contiguousSize    = %d\n",     chelp->ContiguousSize);
        printk("  contiguousBase    = 0x%08X\n", chelp->ContiguousBase);
        printk("  bankSize          = 0x%08X\n", chelp->BankSize);
        printk("  fastClear         = %d\n",     chelp->FastClear);
        printk("  compression       = %d\n",     chelp->Compression);
        printk("  signal            = %d\n",     chelp->Signal);
        printk("  baseAddress       = 0x%08X\n", chelp->PhysBaseAddr);
        printk("  physSize          = 0x%08X\n", chelp->PhysSize);
    }

    /* Create the GAL device. */
    gcmkONERROR(gckGALDEVICE_Construct(chelp, &device));

    /* Start the GAL device. */
    gcmkONERROR(gckGALDEVICE_Start(device));

    if ((chelp->PhysSize != 0)
       && (device->kernels[gcvCORE_MAJOR] != NULL)
       && (device->kernels[gcvCORE_MAJOR]->hardware->mmuVersion != 0))
    {
        status = gckMMU_Enable(device->kernels[gcvCORE_MAJOR]->mmu, chelp->PhysBaseAddr, chelp->PhysSize);
        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_DRIVER,
            "Enable new MMU: status=%d\n", status);

        if ((device->kernels[gcvCORE_2D] != NULL)
            && (device->kernels[gcvCORE_2D]->hardware->mmuVersion != 0))
        {
            status = gckMMU_Enable(device->kernels[gcvCORE_2D]->mmu, chelp->PhysBaseAddr, chelp->PhysSize);
            gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_DRIVER,
                "Enable new MMU for 2D: status=%d\n", status);
        }

        /* Reset the base address */
        device->baseAddress = 0;
    }

    /* Register the character device. */
    ret = register_chrdev(major, DRV_NAME, &driver_fops);

    if (ret < 0)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): Could not allocate major number for mmap.\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    if (major == 0)
    {
        major = ret;
    }

    /* Create the device class. */
    device_class = class_create(THIS_MODULE, "graphics_class");

    if (IS_ERR(device_class))
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): Failed to create the class.\n",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
    }

    device_create(device_class, NULL, MKDEV(major, 0), NULL, "galcore");

    galDevice = device;
    gpuClass  = device_class;

    gcmkTRACE_ZONE(
        gcvLEVEL_INFO, gcvZONE_DRIVER,
        "%s(%d): irqLine=%d, contiguousSize=%lu, memBase=0x%lX\n",
        __FUNCTION__, __LINE__,
        irqLine, contiguousSize, registerMemBase
        );

    /* Success. */
    gcmkFOOTER_NO();
    return 0;

OnError:
    /* Roll back. */
    if (device_class != NULL)
    {
        device_destroy(device_class, MKDEV(major, 0));
        class_destroy(device_class);
    }

    if (device != NULL)
    {
        gcmkVERIFY_OK(gckGALDEVICE_Stop(device));
        gcmkVERIFY_OK(gckGALDEVICE_Destroy(device));
    }

    gcmkFOOTER();
    return result;
}

#if !USE_PLATFORM_DRIVER
static void __exit drv_exit(void)
#else
static void drv_exit(void)
#endif
{
#ifndef CONFIG_JZSOC
    struct clk *clk = galDevice->clk_gpu;
    struct clk *clk_2dcore = galDevice->clk_2d;
#endif

    gcmkHEADER();

    gcmkASSERT(gpuClass != NULL);
    device_destroy(gpuClass, MKDEV(major, 0));
    class_destroy(gpuClass);

    unregister_chrdev(major, DRV_NAME);

    gcmkVERIFY_OK(gckGALDEVICE_Stop(galDevice));
    gcmkVERIFY_OK(gckGALDEVICE_Destroy(galDevice));

#ifndef CONFIG_JZSOC
    clk_disable(clk);
    clk_unprepare(clk);
    clk_put(clk);
    if (clk_2dcore != NULL) {
	clk_disable(clk_2dcore);
	clk_unprepare(clk_2dcore);
	clk_put(clk_2dcore);
    }
#endif

    gcmkFOOTER_NO();
}

#if !USE_PLATFORM_DRIVER
    module_init(drv_init);
    module_exit(drv_exit);
#else

#ifdef CONFIG_DOVE_GPU
#   define DEVICE_NAME "dove_gpu"
#else
#   define DEVICE_NAME "galcore"
#endif

static int  gpu_probe(struct platform_device *pdev)
{
    int ret = -ENODEV;
    struct resource* res;
    struct clk *clk = NULL, *clk_2dcore = NULL;
    gckDEVICE_CONSTRUCT chelp = {
	.IrqLine = irqLine,
	.RegisterMemBase = registerMemBase,
	.RegisterMemSize = registerMemSize,
	.IrqLine2D = irqLine2D,
	.RegisterMemBase2D = registerMemBase2D,
	.RegisterMemSize2D = registerMemSize2D,
	.ContiguousBase = contiguousBase,
	.ContiguousSize = contiguousSize,
	.BankSize = bankSize,
	.FastClear = fastClear,
	.Compression = compression,
	.PhysBaseAddr = baseAddress,
	.PhysSize = physSize,
	.Signal = signal
    };

    gcmkHEADER();

    if (pdev->dev.platform_data != NULL) {
	/* Traditional platform device path */
	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "gpu_irq");

	if (!res)
	{
	    printk(KERN_ERR "%s: No irq line supplied.\n",__FUNCTION__);
	    goto gpu_probe_fail;
	}

	chelp.IrqLine = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "gpu_base");

	if (!res)
	{
	    printk(KERN_ERR "%s: No register base supplied.\n",__FUNCTION__);
	    goto gpu_probe_fail;
	}

	chelp.RegisterMemBase = res->start;
	chelp.RegisterMemSize = res->end - res->start + 1;

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "gpu_mem");

	if (!res)
	{
	    printk(KERN_ERR "%s: No memory base supplied.\n",__FUNCTION__);
	    goto gpu_probe_fail;
	}

	chelp.ContiguousBase = res->start;
	chelp.ContiguousSize = res->end - res->start + 1;
    } else {
	/* Flattened Device Tree path: */
        struct device_node *dt_node = pdev->dev.of_node;
	struct device_node *dt_2dcore;
	struct platform_device *pdev_2dcore;
	const u32 *value;
        int plen;

	dev_info(&pdev->dev, "Using settings from Flattened Device Tree (FDT).\n");

	value = of_get_property(dt_node, "interrupts", &plen);
        if (value == NULL) {
	    dev_err(&pdev->dev, "No 'interrupts' property found in FDT\n");
	    return -ENODEV;
	} else if (plen != 3 * sizeof(u32)) {
	    /* #interrupt-cells should be 3 */
	    dev_err(&pdev->dev, "Invalid interrupts property -- should have 3 cells, got %d.", (plen / sizeof(u32)));
	    goto gpu_probe_fail;
	}

	/* Note: DTS uses (IRQ-32), need to add that back here for actual IRQ: */
	chelp.IrqLine = be32_to_cpu(value[1]) + 32;

	value = of_get_property(dt_node, "reg", &plen);
	if (value == NULL || plen == 0) {
	    dev_err(&pdev->dev, "No 'reg' property found in FDT\n");
	    goto gpu_probe_fail;
	}

	if (plen != 2 * sizeof(u32)) {
	    dev_err(&pdev->dev, "Unintelligible 'reg' property, expecting 2 cells got %d.\n", (plen / sizeof(u32)));
	    goto gpu_probe_fail;
	}

	chelp.RegisterMemBase = be32_to_cpu(value[0]);
	chelp.RegisterMemSize = be32_to_cpu(value[1]);
	chelp.ContiguousBase = 0;	/* dynamically allocate 4MB */
	chelp.ContiguousSize = 4 * 1024 * 1024;

	/* Look for the 2D core ('vivante,gpu2d' compatible device): */
	dt_2dcore = of_find_compatible_node(NULL, NULL, "vivante,gpu2d");
	if (dt_2dcore != NULL) {
	    value = of_get_property(dt_2dcore, "interrupts", &plen);
	    if (value == NULL || plen < 3 * sizeof(u32)) {
		dev_err(&pdev->dev,
			"Missing or invalidly sized 2D 'interrupts' property "
			"(should be 3 or 6 cells)\n");
		goto gpu_probe_fail;
	    }

	    /* First 3-tuple is expected to be the 2D core's interrupt line. The
	     * Freescale iMX6 also has the 2D vector core, which is enumerated after. */
	    chelp.IrqLine2D = be32_to_cpu(value[1]) + 32;

	    value = of_get_property(dt_2dcore, "reg", &plen);
	    if (value == NULL || plen == 0) {
		dev_err(&pdev->dev, "No 2D 'reg' property found in FDT\n");
		goto gpu_probe_fail;
	    }

	    if (plen != 2 * sizeof(u32)) {
		dev_err(&pdev->dev,
			"Unintelligible 2D 'reg' property, expecting 2 cells "
			"got %d.\n",
			(plen / sizeof(u32)));
		goto gpu_probe_fail;
	    }

	    chelp.RegisterMemBase2D = be32_to_cpu(value[0]);
	    chelp.RegisterMemSize2D = be32_to_cpu(value[1]);

	    /* Grab 2d core's clock, if specified. */
	    pdev_2dcore = of_find_device_by_node(dt_2dcore);
	    if (pdev_2dcore != NULL) {
		clk_2dcore = clk_get(&pdev_2dcore->dev, NULL);
	    }

	    /* All done...! */
	    of_node_put(dt_2dcore);
	} else {
	    dev_info(&pdev->dev, "2D core ('vivante,gpu2d' compatible) not present in FDT\n");
	}
    }

    dev_info(&pdev->dev, "driver v4.6.6, initializing\n");

    clk = clk_get(&pdev->dev, NULL);
    if (IS_ERR(clk)) {
        dev_err(&pdev->dev, "cannot get clock\n");
        ret = PTR_ERR(clk);
        goto gpu_probe_fail;
    }

    clk_prepare(clk);
    clk_enable(clk);

    if (clk_2dcore != NULL) {
	clk_prepare(clk_2dcore);
	clk_enable(clk_2dcore);
    }

    ret = drv_init_chelp(&chelp);

    if (!ret)
    {
	galDevice->dev = &pdev->dev;
        platform_set_drvdata(pdev, galDevice);
        galDevice->clk_gpu = clk;
        galDevice->clk_gpu_enabled = 0;
	galDevice->clk_2d = clk_2dcore;
	galDevice->clk_2d_enabled = 0;

        dev_info(&pdev->dev, "GPU initialized, clocked at %luMHz\n",
                 clk_get_rate(clk) / 1000000);

        clk_disable(clk);
	if (clk_2dcore != NULL) {
	    clk_disable(clk_2dcore);
	}

        gcmkFOOTER_NO();
        return ret;
    }

    clk_disable(clk);
    clk_unprepare(clk);
    clk_put(clk);
    if (clk_2dcore != NULL) {
	clk_disable(clk_2dcore);
	clk_unprepare(clk_2dcore);
	clk_put(clk_2dcore);
    }

gpu_probe_fail:
    gcmkFOOTER_ARG(KERN_INFO "Failed to register gpu driver: %d\n", ret);
    return ret;
}

static int gpu_remove(struct platform_device *pdev)
{
    gcmkHEADER();
    drv_exit();
    gcmkFOOTER_NO();
    return 0;
}

static int gpu_suspend(struct platform_device *dev, pm_message_t state)
{
    gceSTATUS status;
    gckGALDEVICE device;
    int i;

#ifdef CONFIG_JZSOC
    cpm_stop_clock(CGM_GPU);
#endif
    device = platform_get_drvdata(dev);

    for (i = 0; i < gcdCORE_COUNT; i++)
    {
        if (device->kernels[i] != NULL)
        {
            /* Store states. */
            status = gckHARDWARE_QueryPowerManagementState(device->kernels[i]->hardware, &device->statesStored[i]);
            if (gcmIS_ERROR(status))
            {
                return -1;
            }

            status = gckHARDWARE_SetPowerManagementState(device->kernels[i]->hardware, gcvPOWER_OFF);
            if (gcmIS_ERROR(status))
            {
                return -1;
            }
        }
    }

    return 0;
}

static int gpu_resume(struct platform_device *dev)
{
    gceSTATUS status;
    gckGALDEVICE device;
    int i;
    gceCHIPPOWERSTATE   statesStored;

#ifdef CONFIG_JZSOC
    cpm_start_clock(CGM_GPU);
#endif
    device = platform_get_drvdata(dev);

    for (i = 0; i < gcdCORE_COUNT; i++)
    {
        if (device->kernels[i] != NULL)
        {
            status = gckHARDWARE_SetPowerManagementState(device->kernels[i]->hardware, gcvPOWER_ON);
            if (gcmIS_ERROR(status))
            {
                return -1;
            }

            /* Convert global state to crossponding internal state. */
            switch(device->statesStored[i])
            {
            case gcvPOWER_OFF:
                statesStored = gcvPOWER_OFF_BROADCAST;
                break;
            case gcvPOWER_IDLE:
                statesStored = gcvPOWER_IDLE_BROADCAST;
                break;
            case gcvPOWER_SUSPEND:
                statesStored = gcvPOWER_SUSPEND_BROADCAST;
                break;
            case gcvPOWER_ON:
                statesStored = gcvPOWER_ON_AUTO;
                break;
            default:
                statesStored = device->statesStored[i];
                break;
            }

            /* Restore states. */
            status = gckHARDWARE_SetPowerManagementState(device->kernels[i]->hardware, statesStored);
            if (gcmIS_ERROR(status))
            {
                return -1;
            }
        }
    }

    return 0;
}

#ifdef CONFIG_OF
/* Device tree support: */
static const struct of_device_id gpu_of_ids[] = {
    { .compatible = "vivante,gpu3d" },
    { .compatible = "galcore" },
    { }
};
#endif

static struct platform_driver gpu_driver = {
    .probe      = gpu_probe,
    .remove     = gpu_remove,

    .suspend    = gpu_suspend,
    .resume     = gpu_resume,

    .driver     = {
        .name   = DEVICE_NAME,
#ifdef CONFIG_OF
	.of_match_table = gpu_of_ids
#endif
    }
};

static int __init gpu_init(void)
{
    int ret = 0;

    ret = platform_driver_register(&gpu_driver);
    return ret;
}

static void __exit gpu_exit(void)
{
    platform_driver_unregister(&gpu_driver);
}

module_init(gpu_init);
module_exit(gpu_exit);

#endif
