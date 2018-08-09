/*
 * Copyright (C) 2018 Microchip
 * Joshua Henderson <joshua.henderson@microchip.com>
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
#ifndef __GFX2D_RINGBUFFER_H__
#define __GFX2D_RINGBUFFER_H__

#include <linux/types.h>
#include <linux/list.h>
#include <linux/spinlock.h>

struct gfx2d_gpu;

struct gfx2d_ringbuffer {
	struct gfx2d_gpu *gpu;
	uint32_t size;
	uint32_t *start, *end, *cur, *next;
	dma_addr_t paddr;
	spinlock_t lock;
};

struct gfx2d_ringbuffer *gfx2d_ringbuffer_new(struct gfx2d_gpu *gpu);
void gfx2d_ringbuffer_destroy(struct gfx2d_ringbuffer *ring);

static inline void OUT_RING(struct gfx2d_ringbuffer *ring, uint32_t data)
{
	/*
	 * ring->next points to the current command being written - it won't be
	 * committed as ring->cur until the flush
	 */
	if (ring->next == ring->end)
		ring->next = ring->start;
	*(ring->next++) = data;
}

#endif /* __GFX2D_RINGBUFFER_H__ */
