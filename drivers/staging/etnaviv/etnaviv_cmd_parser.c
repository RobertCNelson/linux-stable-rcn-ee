/*
 * Copyright (C) 2015 Etnaviv Project
 * Author: Russell King <rmk+kernel@arm.linux.org.uk>
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

#include "etnaviv_gem.h"
#include "etnaviv_gpu.h"

#include "cmdstream.xml.h"

static const char *opcode_to_str(u8 cmd)
{
    const char *op_names[] =
    {
        "FE_OPCODE_LOAD_STATE",
        "FE_OPCODE_END",
        "FE_OPCODE_NOP",
        "FE_OPCODE_DRAW_2D",
        "FE_OPCODE_DRAW_PRIMITIVES",
        "FE_OPCODE_DRAW_INDEXED_PRIMITIVES",
        "FE_OPCODE_WAIT",
        "FE_OPCODE_LINK",
        "FE_OPCODE_STALL",
        "FE_OPCODE_CALL",
        "FE_OPCODE_RETURN",
        "FE_OPCODE_CHIP_SELECT"
    };

    if (cmd <= FE_OPCODE_CHIP_SELECT)
        return op_names[cmd];

    return "UNKOWN OPCODE";
}

static int validate_load_state(u32 cmd, u32 *data)
{
	unsigned int count, off;

	count = VIV_FE_LOAD_STATE_HEADER_COUNT(cmd);
	off = VIV_FE_LOAD_STATE_HEADER_OFFSET(cmd);

	if (count == 0x0)
		return -EINVAL;

	/* TODO: validate offset */

	return count + 1;
}

bool etnaviv_cmd_validate(struct etnaviv_gpu *gpu,
	struct etnaviv_gem_object *obj, unsigned int size)
{
	u32 *start = obj->vaddr;
	u32 *buf = start;
	u32 *end = buf + size;

	while (buf < end) {
		u32 cmd = *buf;
		unsigned int len, n;
		unsigned int op = cmd >> 27;

		switch (op) {
		case FE_OPCODE_LOAD_STATE:
			len = validate_load_state(cmd, buf + 1);
			break;

		case FE_OPCODE_DRAW_2D:
			n = VIV_FE_DRAW_2D_HEADER_COUNT(cmd);
			len = 2 + n * 2;
			break;

		case FE_OPCODE_DRAW_PRIMITIVES:
			len = 4;
			break;

		case FE_OPCODE_DRAW_INDEXED_PRIMITIVES:
			len = 6;
			break;

		case FE_OPCODE_NOP:
		case FE_OPCODE_STALL:
			len = 2;
			break;

		default:
			dev_err(gpu->dev, "%s: op %s (%u) not permitted at offset %u\n",
					__func__, opcode_to_str(op), op, buf - start);
			return false;
		}

		if (len < 0) {
			dev_err(gpu->dev, "%s: op %s (%u) at offset %u is not valid\n",
					__func__, opcode_to_str(op), op, buf - start);
			return false;
		}

		buf += ALIGN(len, 2);
	}

	if (buf > end) {
		dev_err(gpu->dev, "%s: commands overflow end of buffer: %u > %u\n",
			__func__, buf - start, size);
		return false;
	}

	return true;
}
