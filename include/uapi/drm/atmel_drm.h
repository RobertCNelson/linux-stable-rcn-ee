/*
 * Copyright (C) 2014 Atmel
 *
 * Author: Mohamed Jamsheeth <mohamedjamsheeth.hajanajubudeen@atmel.com>
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

#ifndef _UAPI_ATMEL_DRM_H_
#define _UAPI_ATMEL_DRM_H_

#include <drm/drm.h>

#define DRM_ATMEL_GEM_GET		0x00

#define DRM_IOCTL_ATMEL_GEM_GET		DRM_IOWR(DRM_COMMAND_BASE + \
					DRM_ATMEL_GEM_GET, struct drm_mode_map_dumb)

#endif /* _UAPI_ATMEL_DRM_H_ */
