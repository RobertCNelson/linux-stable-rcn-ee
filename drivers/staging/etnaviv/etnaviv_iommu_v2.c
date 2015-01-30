/*
 * Copyright (C) 2014 Christian Gmeiner <christian.gmeiner@gmail.com>
  *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/iommu.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/bitops.h>

#include "etnaviv_gpu.h"
#include "etnaviv_iommu.h"
#include "state_hi.xml.h"

#define IOMMU_MTLB_SHIFT	24
#define IOMMU_MTLB_SIZE		(1UL << IOMMU_MTLB_SHIFT)
#define IOMMU_MTLB_MASK		(~(IOMMU_MTLB_SIZE-1))

#define IOMMU_MTLB_PRESENT	BIT(0)
#define IOMMU_MTLB_EXCEPT	BIT(1)

#define IOMMU_STLB_SHIFT	12
#define IOMMU_STLB_SIZE		(1UL << IOMMU_STLB_SHIFT)
#define IOMMU_STLB_MASK		(~(IOMMU_STLB_SIZE-1))

#define IOMMU_STLB_PRESENT	BIT(0)
#define IOMMU_STLB_EXEPT	BIT(1)
#define IOMMU_STLB_WRITABLE	BIT(2)

#define IOMMU_MTLB_ADDRESS_MASK 0xFFFFFFC0
#define IOMMU_STLB_ADDRESS_MASK 0xFFFFF000

#define IOMMU_PTRS_PER_MTLB	256
#define IOMMU_PTRS_PER_STLB	4096

struct table {
	u32 *pgtable;
	dma_addr_t paddr;
};

struct etnaviv_iommu_v2_domain {
	spinlock_t map_lock;
	struct table mtlb;
	struct table stlb[IOMMU_PTRS_PER_MTLB];
};

static int stlb_alloc(struct etnaviv_iommu_v2_domain *domain,
		unsigned int index)
{
	struct table *stlb = &domain->stlb[index];

	stlb->pgtable = dma_alloc_coherent(NULL, IOMMU_PTRS_PER_STLB,
			&stlb->paddr, GFP_KERNEL);

	if (!stlb->pgtable)
		return -ENOMEM;

	memset(stlb->pgtable, 0, IOMMU_PTRS_PER_STLB);

	domain->mtlb.pgtable[index] = (stlb->paddr & IOMMU_MTLB_ADDRESS_MASK) |
						 IOMMU_MTLB_PRESENT |
						 IOMMU_MTLB_EXCEPT;
	return 0;
}

static void stlb_free(struct etnaviv_iommu_v2_domain *domain,
		unsigned int index)
{
	struct table *stlb = &domain->stlb[index];

	dma_free_coherent(NULL, IOMMU_PTRS_PER_STLB,
			stlb->pgtable, stlb->paddr);
}

static int mtlb_alloc(struct etnaviv_iommu_v2_domain *domain)
{
	struct table *mtlb = &domain->mtlb;

	mtlb->pgtable = dma_alloc_coherent(NULL, IOMMU_PTRS_PER_MTLB,
			&mtlb->paddr, GFP_KERNEL);

	if (!mtlb->pgtable)
		return -ENOMEM;

	memset(mtlb->pgtable, 0, IOMMU_PTRS_PER_MTLB);

	return 0;
}

static void mtlb_free(struct etnaviv_iommu_v2_domain *domain)
{
	struct table *mtlb = &domain->mtlb;

	dma_free_coherent(NULL, IOMMU_PTRS_PER_MTLB,
			mtlb->pgtable, mtlb->paddr);
}

static int etnaviv_iommu_v2_domain_init(struct iommu_domain *domain)
{
	struct etnaviv_iommu_v2_domain *etnaviv_domain;
	int ret;

	etnaviv_domain = kmalloc(sizeof(*etnaviv_domain), GFP_KERNEL);
	if (!etnaviv_domain)
		return -ENOMEM;

	ret = mtlb_alloc(etnaviv_domain);
	if (ret < 0) {
		kfree(etnaviv_domain);
		return ret;
	}

	spin_lock_init(&etnaviv_domain->map_lock);
	domain->priv = etnaviv_domain;

	domain->geometry.aperture_start = 0;
	domain->geometry.aperture_end   = (1ULL << 32) - 1;
	domain->geometry.force_aperture = true;

	return 0;
}

static void etnaviv_iommu_v2_domain_destroy(struct iommu_domain *domain)
{
	struct etnaviv_iommu_v2_domain *etnaviv_domain = domain->priv;
	u32 i;

	for (i = 0; i < IOMMU_PTRS_PER_MTLB; i++) {
		if (etnaviv_domain->mtlb.pgtable[i] & IOMMU_MTLB_PRESENT)
			stlb_free(etnaviv_domain, i);
	}

	mtlb_free(etnaviv_domain);

	kfree(etnaviv_domain);
	domain->priv = NULL;
}

static int etnaviv_iommu_v2_map(struct iommu_domain *domain,
	unsigned long iova, phys_addr_t paddr, size_t size, int prot)
{
	struct etnaviv_iommu_v2_domain *etnaviv_domain = domain->priv;
	struct table *stlb;
	u32 mtlb_index, stlb_index, value;
	int ret = 0;

	if (size != SZ_4K)
		return -EINVAL;

	spin_lock(&etnaviv_domain->map_lock);

	mtlb_index = (iova & IOMMU_MTLB_MASK) >> IOMMU_MTLB_SHIFT;
	stlb_index = (iova & IOMMU_STLB_MASK) >> IOMMU_STLB_SHIFT;

	stlb = &etnaviv_domain->stlb[stlb_index];

	if (!(etnaviv_domain->mtlb.pgtable[mtlb_index] & IOMMU_MTLB_PRESENT)) {
		ret = stlb_alloc(etnaviv_domain, mtlb_index);
		if (ret) {
			dev_err(NULL, "Could not allocate second level table\n");
			goto fail;
		}
	}

	value = (paddr & IOMMU_STLB_ADDRESS_MASK);
	value |= IOMMU_STLB_PRESENT | IOMMU_STLB_EXEPT | IOMMU_STLB_WRITABLE;
	stlb->pgtable[stlb_index] = value;

fail:
	spin_unlock(&etnaviv_domain->map_lock);
	return ret;
}

static size_t etnaviv_iommu_v2_unmap(struct iommu_domain *domain,
	unsigned long iova, size_t size)
{
	struct etnaviv_iommu_v2_domain *etnaviv_domain = domain->priv;
	u32 mtlb_index, stlb_index;

	if (size != SZ_4K)
		return -EINVAL;

	spin_lock(&etnaviv_domain->map_lock);

	mtlb_index = (iova & IOMMU_MTLB_MASK) >> IOMMU_MTLB_SHIFT;
	stlb_index = (iova & IOMMU_STLB_MASK) >> IOMMU_STLB_SHIFT;

	etnaviv_domain->stlb[mtlb_index].pgtable[stlb_index] = 0;

	spin_unlock(&etnaviv_domain->map_lock);

	return 0;
}

static phys_addr_t etnaviv_iommu_v2_iova_to_phys(struct iommu_domain *domain,
	dma_addr_t iova)
{
	struct etnaviv_iommu_v2_domain *etnaviv_domain = domain->priv;
	phys_addr_t ret = 0;
	u32 mtlb_index, stlb_index;

	spin_lock(&etnaviv_domain->map_lock);

	mtlb_index = (iova & IOMMU_MTLB_MASK) >> IOMMU_MTLB_SHIFT;
	stlb_index = (iova & IOMMU_STLB_MASK) >> IOMMU_STLB_SHIFT;
	ret = etnaviv_domain->stlb[mtlb_index].pgtable[stlb_index];

	spin_unlock(&etnaviv_domain->map_lock);

	return ret;
}

static struct iommu_ops etnaviv_iommu_v2_ops = {
	.domain_init = etnaviv_iommu_v2_domain_init,
	.domain_destroy = etnaviv_iommu_v2_domain_destroy,
	.map = etnaviv_iommu_v2_map,
	.unmap = etnaviv_iommu_v2_unmap,
	.iova_to_phys = etnaviv_iommu_v2_iova_to_phys,
	.pgsize_bitmap = SZ_4K,
};

struct iommu_domain *etnaviv_iommu_v2_domain_alloc(struct etnaviv_gpu *gpu)
{
	struct iommu_domain *domain;
	struct etnaviv_iommu_v2_domain *etnaviv_domain;
	int ret;

	domain = kzalloc(sizeof(*domain), GFP_KERNEL);
	if (!domain)
		return NULL;

	domain->ops = &etnaviv_iommu_v2_ops;

	ret = domain->ops->domain_init(domain);
	if (ret)
		goto out_free;

	/* store page table address */
	etnaviv_domain = domain->priv;
	gpu->pgtable = etnaviv_domain->mtlb.paddr;

	return domain;

out_free:
	kfree(domain);
	return NULL;
}
