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
	static const char * const op_names[] = {
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
		"UNKOWN OPCODE",
		"FE_OPCODE_CHIP_SELECT"
	};

	if (cmd <= FE_OPCODE_CHIP_SELECT)
		return op_names[cmd - 1];

	return "UNKOWN OPCODE";
}

static int validate_offset(u32 offset, u32 data)
{
	int ret = 0;
	u32 reg = offset * 4;

	switch (reg) {
	/* DE */
	case 0x00001200:	/* VIVS_DE_SRC_ADDRESS */
	case 0x00001228:	/* VIVS_DE_DEST_ADDRESS */
	case 0x00001238:	/* VIVS_DE_PATTERN_ADDRESS */
	case 0x00001284:	/* VIVS_DE_UPLANE_ADDRESS */
	case 0x0000128c:	/* VIVS_DE_VPLANE_ADDRESS */
	case 0x00001304:	/* VIVS_DE_SRC_EX_ADDRESS */
	case 0x00001310:	/* VIVS_DE_DE_PLANE2_ADDRESS */
	case 0x00001318:	/* VIVS_DE_DE_PLANE3_ADDRESS */
	case 0x00012800:	/* VIVS_DE_BLOCK4_SRC_ADDRESS_0 */
	case 0x00012804:	/* VIVS_DE_BLOCK4_SRC_ADDRESS_1 */
	case 0x00012808:	/* VIVS_DE_BLOCK4_SRC_ADDRESS_2 */
	case 0x0001280c:	/* VIVS_DE_BLOCK4_SRC_ADDRESS_3 */
	case 0x000128a0:	/* VIVS_DE_BLOCK4_ADDRESS_U_0 */
	case 0x000128a4:	/* VIVS_DE_BLOCK4_ADDRESS_U_1 */
	case 0x000128a8:	/* VIVS_DE_BLOCK4_ADDRESS_U_2 */
	case 0x000128ac:	/* VIVS_DE_BLOCK4_ADDRESS_U_3 */
	case 0x000128c0:	/* VIVS_DE_BLOCK4_ADDRESS_V_0 */
	case 0x000128c4:	/* VIVS_DE_BLOCK4_ADDRESS_V_1 */
	case 0x000128c8:	/* VIVS_DE_BLOCK4_ADDRESS_V_2 */
	case 0x000128cc:	/* VIVS_DE_BLOCK4_ADDRESS_V_3 */
	case 0x00012970:	/* VIVS_DE_BLOCK4_SRC_EX_ADDRESS_0 */
	case 0x00012974:	/* VIVS_DE_BLOCK4_SRC_EX_ADDRESS_1 */
	case 0x00012978:	/* VIVS_DE_BLOCK4_SRC_EX_ADDRESS_2 */
	case 0x0001297c:	/* VIVS_DE_BLOCK4_SRC_EX_ADDRESS_3 */
	case 0x00012a00:	/* VIVS_DE_BLOCK8_SRC_ADDRESS_0 */
	case 0x00012a04:	/* VIVS_DE_BLOCK8_SRC_ADDRESS_1 */
	case 0x00012a08:	/* VIVS_DE_BLOCK8_SRC_ADDRESS_2 */
	case 0x00012a0c:	/* VIVS_DE_BLOCK8_SRC_ADDRESS_3 */
	case 0x00012a10:	/* VIVS_DE_BLOCK8_SRC_ADDRESS_4 */
	case 0x00012a14:	/* VIVS_DE_BLOCK8_SRC_ADDRESS_5 */
	case 0x00012a18:	/* VIVS_DE_BLOCK8_SRC_ADDRESS_6 */
	case 0x00012a1c:	/* VIVS_DE_BLOCK8_SRC_ADDRESS_7 */
	case 0x00012b40:	/* VIVS_DE_BLOCK8_ADDRESS_U_0 */
	case 0x00012b44:	/* VIVS_DE_BLOCK8_ADDRESS_U_1 */
	case 0x00012b48:	/* VIVS_DE_BLOCK8_ADDRESS_U_2 */
	case 0x00012b4c:	/* VIVS_DE_BLOCK8_ADDRESS_U_3 */
	case 0x00012b50:	/* VIVS_DE_BLOCK8_ADDRESS_U_4 */
	case 0x00012b54:	/* VIVS_DE_BLOCK8_ADDRESS_U_5 */
	case 0x00012b58:	/* VIVS_DE_BLOCK8_ADDRESS_U_6 */
	case 0x00012b5c:	/* VIVS_DE_BLOCK8_ADDRESS_U_7 */
	case 0x00012b80:	/* VIVS_DE_BLOCK8_ADDRESS_V_0 */
	case 0x00012b84:	/* VIVS_DE_BLOCK8_ADDRESS_V_1 */
	case 0x00012b88:	/* VIVS_DE_BLOCK8_ADDRESS_V_2 */
	case 0x00012b8c:	/* VIVS_DE_BLOCK8_ADDRESS_V_3 */
	case 0x00012b90:	/* VIVS_DE_BLOCK8_ADDRESS_V_4 */
	case 0x00012b94:	/* VIVS_DE_BLOCK8_ADDRESS_V_5 */
	case 0x00012b98:	/* VIVS_DE_BLOCK8_ADDRESS_V_6 */
	case 0x00012b9c:	/* VIVS_DE_BLOCK8_ADDRESS_V_7 */
	case 0x00012ce0:	/* VIVS_DE_BLOCK8_SRC_EX_ADDRESS_0 */
	case 0x00012ce4:	/* VIVS_DE_BLOCK8_SRC_EX_ADDRESS_1 */
	case 0x00012ce8:	/* VIVS_DE_BLOCK8_SRC_EX_ADDRESS_2 */
	case 0x00012cec:	/* VIVS_DE_BLOCK8_SRC_EX_ADDRESS_3 */
	case 0x00012cf0:	/* VIVS_DE_BLOCK8_SRC_EX_ADDRESS_4 */
	case 0x00012cf4:	/* VIVS_DE_BLOCK8_SRC_EX_ADDRESS_5 */
	case 0x00012cf8:	/* VIVS_DE_BLOCK8_SRC_EX_ADDRESS_6 */
	case 0x00012cfc:	/* VIVS_DE_BLOCK8_SRC_EX_ADDRESS_7 */
	/* PE */
	case 0x00001410:	/* VIVS_PE_DEPTH_ADDR */
	case 0x00001430:	/* VIVS_PE_COLOR_ADDR */
	case 0x00001458:	/* VIVS_PE_HDEPTH_ADDR */
	case 0x00001460:	/* VIVS_PE_PIPE_COLOR_ADDR_0 */
	case 0x00001464:	/* VIVS_PE_PIPE_COLOR_ADDR_1 */
	case 0x00001468:	/* VIVS_PE_PIPE_COLOR_ADDR_2 */
	case 0x0000146c:	/* VIVS_PE_PIPE_COLOR_ADDR_3 */
	case 0x00001470:	/* VIVS_PE_PIPE_COLOR_ADDR_4 */
	case 0x00001474:	/* VIVS_PE_PIPE_COLOR_ADDR_5 */
	case 0x00001478:	/* VIVS_PE_PIPE_COLOR_ADDR_6 */
	case 0x0000147c:	/* VIVS_PE_PIPE_COLOR_ADDR_7 */
	case 0x00001480:	/* VIVS_PE_PIPE_DEPTH_ADDR_0 */
	case 0x00001484:	/* VIVS_PE_PIPE_DEPTH_ADDR_1 */
	case 0x00001488:	/* VIVS_PE_PIPE_DEPTH_ADDR_2 */
	case 0x0000148c:	/* VIVS_PE_PIPE_DEPTH_ADDR_3 */
	case 0x00001490:	/* VIVS_PE_PIPE_DEPTH_ADDR_4 */
	case 0x00001494:	/* VIVS_PE_PIPE_DEPTH_ADDR_5 */
	case 0x00001498:	/* VIVS_PE_PIPE_DEPTH_ADDR_6 */
	case 0x0000149c:	/* VIVS_PE_PIPE_DEPTH_ADDR_7 */
	case 0x00001500:	/* VIVS_PE_PIPE_ADDR_UNK01500_0 */
	case 0x00001504:	/* VIVS_PE_PIPE_ADDR_UNK01500_1 */
	case 0x00001508:	/* VIVS_PE_PIPE_ADDR_UNK01500_2 */
	case 0x0000150c:	/* VIVS_PE_PIPE_ADDR_UNK01500_3 */
	case 0x00001510:	/* VIVS_PE_PIPE_ADDR_UNK01500_4 */
	case 0x00001514:	/* VIVS_PE_PIPE_ADDR_UNK01500_5 */
	case 0x00001518:	/* VIVS_PE_PIPE_ADDR_UNK01500_6 */
	case 0x0000151c:	/* VIVS_PE_PIPE_ADDR_UNK01500_7 */
	case 0x00001520:	/* VIVS_PE_PIPE_ADDR_UNK01520_0 */
	case 0x00001524:	/* VIVS_PE_PIPE_ADDR_UNK01520_1 */
	case 0x00001528:	/* VIVS_PE_PIPE_ADDR_UNK01520_2 */
	case 0x0000152c:	/* VIVS_PE_PIPE_ADDR_UNK01520_3 */
	case 0x00001530:	/* VIVS_PE_PIPE_ADDR_UNK01520_4 */
	case 0x00001534:	/* VIVS_PE_PIPE_ADDR_UNK01520_5 */
	case 0x00001538:	/* VIVS_PE_PIPE_ADDR_UNK01520_6 */
	case 0x0000153c:	/* VIVS_PE_PIPE_ADDR_UNK01520_7 */
	/* RS */
	case 0x00001608:	/* VIVS_RS_SOURCE_ADDR */
	case 0x00001610:	/* VIVS_RS_DEST_ADDR */
	case 0x000016c0:	/* VIVS_RS_PIPE_SOURCE_ADDR_0 */
	case 0x000016c4:	/* VIVS_RS_PIPE_SOURCE_ADDR_1 */
	case 0x000016c8:	/* VIVS_RS_PIPE_SOURCE_ADDR_2 */
	case 0x000016cc:	/* VIVS_RS_PIPE_SOURCE_ADDR_3 */
	case 0x000016d0:	/* VIVS_RS_PIPE_SOURCE_ADDR_4 */
	case 0x000016d4:	/* VIVS_RS_PIPE_SOURCE_ADDR_5 */
	case 0x000016d8:	/* VIVS_RS_PIPE_SOURCE_ADDR_6 */
	case 0x000016dc:	/* VIVS_RS_PIPE_SOURCE_ADDR_7 */
	case 0x000016e0:	/* VIVS_RS_PIPE_DEST_ADDR_0 */
	case 0x000016e4:	/* VIVS_RS_PIPE_DEST_ADDR_1 */
	case 0x000016e8:	/* VIVS_RS_PIPE_DEST_ADDR_2 */
	case 0x000016ec:	/* VIVS_RS_PIPE_DEST_ADDR_3 */
	case 0x000016f0:	/* VIVS_RS_PIPE_DEST_ADDR_4 */
	case 0x000016f4:	/* VIVS_RS_PIPE_DEST_ADDR_5 */
	case 0x000016f8:	/* VIVS_RS_PIPE_DEST_ADDR_6 */
	case 0x000016fc:	/* VIVS_RS_PIPE_DEST_ADDR_7 */
		if (data != 0x0)
			ret = -EINVAL;
		break;

	default:
		break;
	}

	return ret;
}

static int validate_load_state(u32 cmd, u32 *data)
{
	unsigned int count, off, i;
	int ret = 0;

	count = (cmd & VIV_FE_LOAD_STATE_HEADER_COUNT__MASK) >>
			VIV_FE_LOAD_STATE_HEADER_COUNT__SHIFT;
	off = (cmd & VIV_FE_LOAD_STATE_HEADER_OFFSET__MASK) >>
			VIV_FE_LOAD_STATE_HEADER_OFFSET__SHIFT;

	if (count == 0x0)
		return -EINVAL;

	for (i = 0; i < count; i++)
		ret |= validate_offset(off + i, *(data + i));

	if (ret)
		return ret;

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
		int len, n;
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
