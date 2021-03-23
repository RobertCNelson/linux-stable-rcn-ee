/*
 * Remote Processor Framework Elf loader
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
 *
 * Ohad Ben-Cohen <ohad@wizery.com>
 * Brian Swetland <swetland@google.com>
 * Mark Grosen <mgrosen@ti.com>
 * Fernando Guzman Lugo <fernando.lugo@ti.com>
 * Suman Anna <s-anna@ti.com>
 * Robert Tivy <rtivy@ti.com>
 * Armando Uribe De Leon <x0095078@ti.com>
 * Sjur Brændeland <sjur.brandeland@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)    "%s: " fmt, __func__

#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/remoteproc.h>
#include <linux/elf.h>

#include "remoteproc_internal.h"

#define SHT_TI_PHATTRS 0x7F000004
#define SHT_TI_SH_PAGE 0x7F000007

struct elf32_ti_phattrs {
	Elf32_Half pha_seg_id; /* Segment id */
	Elf32_Half pha_tag_id; /* Attribute kind id */
	union {
		Elf32_Off pha_offset; /* byte offset within the section */
		Elf32_Word pha_value; /* Constant tag value */
	} pha_un;
};

/* this struct is reverse engineered, so not sure what most of the values are */
struct ti_section_page {
	u32 unk0;
	u32 unk1;
	u32 unk2;
	u32 unk3;
	u32 unk4;
	u16 size;
	u16 unk5;
	u16 unk6;
	u8 data[0]; /* array of size */
};

/**
 * rproc_elf_segment_to_map() - Gets memory map for segment
 * @id: segment id
 * @elf_data: pointer to ELF file data
 *
 * Returns the memory map for the segment.
 */
static int rproc_elf_segment_to_map(u32 id, const u8 *elf_data)
{
	struct elf32_hdr *ehdr;
	struct elf32_shdr *shdr;
	struct elf32_ti_phattrs *ti_attrs = NULL;
	int i;

	ehdr = (struct elf32_hdr *)elf_data;
	shdr = (struct elf32_shdr *)(elf_data + ehdr->e_shoff);

	if (ehdr->e_machine != EM_TI_PRU)
		return 0;

	for (i = 0; i < ehdr->e_shnum; i++, shdr++) {
		if (shdr->sh_type == SHT_TI_PHATTRS) {
			ti_attrs = (struct elf32_ti_phattrs *)(elf_data + shdr->sh_offset);
			break;
		}
	}

	if (!ti_attrs)
		return 0;

	/* list is terminated by tag id == 0 (PHA_NULL) */
	for (; ti_attrs->pha_tag_id; ti_attrs++) {
		if (ti_attrs->pha_tag_id == 3 && ti_attrs->pha_seg_id == id)
			return ti_attrs->pha_un.pha_value;
	}

	return 0;
}

/**
 * rproc_elf_section_to_map() - Gets memory map for section
 * @id: segment id
 * @elf_data: pointer to ELF file data
 *
 * Returns the memory map for the section.
 */
static int rproc_elf_section_to_map(u32 id, const u8 *elf_data)
{
	struct elf32_hdr *ehdr;
	struct elf32_shdr *shdr;
	struct ti_section_page *map = NULL;
	int i;

	ehdr = (struct elf32_hdr *)elf_data;
	shdr = (struct elf32_shdr *)(elf_data + ehdr->e_shoff);

	if (ehdr->e_machine != EM_TI_PRU)
		return 0;

	for (i = 0; i < ehdr->e_shnum; i++, shdr++) {
		if (shdr->sh_type == SHT_TI_SH_PAGE) {
			map = (struct ti_section_page *)(elf_data + shdr->sh_offset);
			break;
		}
	}

	if (!map || id >= map->size)
		return 0;

	return map->data[id];
}

/**
 * rproc_elf_sanity_check() - Sanity Check ELF firmware image
 * @rproc: the remote processor handle
 * @fw: the ELF firmware image
 *
 * Make sure this fw image is sane.
 */
int rproc_elf_sanity_check(struct rproc *rproc, const struct firmware *fw)
{
	const char *name = rproc->firmware;
	struct device *dev = &rproc->dev;
	struct elf32_hdr *ehdr;
	char class;

	if (!fw) {
		dev_err(dev, "failed to load %s\n", name);
		return -EINVAL;
	}

	if (fw->size < sizeof(struct elf32_hdr)) {
		dev_err(dev, "Image is too small\n");
		return -EINVAL;
	}

	ehdr = (struct elf32_hdr *)fw->data;

	/* We only support ELF32 at this point */
	class = ehdr->e_ident[EI_CLASS];
	if (class != ELFCLASS32) {
		dev_err(dev, "Unsupported class: %d\n", class);
		return -EINVAL;
	}

	/* We assume the firmware has the same endianness as the host */
# ifdef __LITTLE_ENDIAN
	if (ehdr->e_ident[EI_DATA] != ELFDATA2LSB) {
# else /* BIG ENDIAN */
	if (ehdr->e_ident[EI_DATA] != ELFDATA2MSB) {
# endif
		dev_err(dev, "Unsupported firmware endianness\n");
		return -EINVAL;
	}

	if (fw->size < ehdr->e_shoff + sizeof(struct elf32_shdr)) {
		dev_err(dev, "Image is too small\n");
		return -EINVAL;
	}

	if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG)) {
		dev_err(dev, "Image is corrupted (bad magic)\n");
		return -EINVAL;
	}

	if (ehdr->e_phnum == 0) {
		dev_err(dev, "No loadable segments\n");
		return -EINVAL;
	}

	if (ehdr->e_phoff > fw->size) {
		dev_err(dev, "Firmware size is too small\n");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(rproc_elf_sanity_check);

/**
 * rproc_elf_get_boot_addr() - Get rproc's boot address.
 * @rproc: the remote processor handle
 * @fw: the ELF firmware image
 *
 * This function returns the entry point address of the ELF
 * image.
 *
 * Note that the boot address is not a configurable property of all remote
 * processors. Some will always boot at a specific hard-coded address.
 */
u32 rproc_elf_get_boot_addr(struct rproc *rproc, const struct firmware *fw)
{
	struct elf32_hdr *ehdr  = (struct elf32_hdr *)fw->data;

	return ehdr->e_entry;
}
EXPORT_SYMBOL(rproc_elf_get_boot_addr);

/**
 * rproc_elf_load_segments() - load firmware segments to memory
 * @rproc: remote processor which will be booted using these fw segments
 * @fw: the ELF firmware image
 *
 * This function loads the firmware segments to memory, where the remote
 * processor expects them.
 *
 * Some remote processors will expect their code and data to be placed
 * in specific device addresses, and can't have them dynamically assigned.
 *
 * We currently support only those kind of remote processors, and expect
 * the program header's paddr member to contain those addresses. We then go
 * through the physically contiguous "carveout" memory regions which we
 * allocated (and mapped) earlier on behalf of the remote processor,
 * and "translate" device address to kernel addresses, so we can copy the
 * segments where they are expected.
 *
 * Currently we only support remote processors that required carveout
 * allocations and got them mapped onto their iommus. Some processors
 * might be different: they might not have iommus, and would prefer to
 * directly allocate memory for every segment/resource. This is not yet
 * supported, though.
 */
int rproc_elf_load_segments(struct rproc *rproc, const struct firmware *fw)
{
	struct device *dev = &rproc->dev;
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	int i, map, ret = 0;
	const u8 *elf_data = fw->data;

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);

	/* go through the available ELF segments */
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		u32 da = phdr->p_paddr;
		u32 memsz = phdr->p_memsz;
		u32 filesz = phdr->p_filesz;
		u32 offset = phdr->p_offset;
		void *ptr;

		if (phdr->p_type != PT_LOAD)
			continue;

		dev_dbg(dev, "phdr: type %d da 0x%x memsz 0x%x filesz 0x%x\n",
			phdr->p_type, da, memsz, filesz);

		if (filesz > memsz) {
			dev_err(dev, "bad phdr filesz 0x%x memsz 0x%x\n",
				filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > fw->size) {
			dev_err(dev, "truncated fw: need 0x%x avail 0x%zx\n",
				offset + filesz, fw->size);
			ret = -EINVAL;
			break;
		}

		map = rproc_elf_segment_to_map(i, elf_data);

		/* grab the kernel address for this device address */
		ptr = rproc_da_to_va(rproc, da, memsz, map);
		if (!ptr) {
			dev_err(dev, "bad phdr da 0x%x mem 0x%x\n", da, memsz);
			ret = -EINVAL;
			break;
		}

		/* put the segment where the remote processor expects it */
		if (phdr->p_filesz)
			memcpy(ptr, elf_data + phdr->p_offset, filesz);

		/*
		 * Zero out remaining memory for this segment.
		 *
		 * This isn't strictly required since dma_alloc_coherent already
		 * did this for us. albeit harmless, we may consider removing
		 * this.
		 */
		if (memsz > filesz)
			memset(ptr + filesz, 0, memsz - filesz);
	}

	return ret;
}
EXPORT_SYMBOL(rproc_elf_load_segments);

static struct elf32_shdr *
find_table(struct device *dev, struct elf32_hdr *ehdr, size_t fw_size, int *id)
{
	struct elf32_shdr *shdr;
	int i;
	const char *name_table;
	struct resource_table *table = NULL;
	const u8 *elf_data = (void *)ehdr;

	/* look for the resource table and handle it */
	shdr = (struct elf32_shdr *)(elf_data + ehdr->e_shoff);
	name_table = elf_data + shdr[ehdr->e_shstrndx].sh_offset;

	for (i = 0; i < ehdr->e_shnum; i++, shdr++) {
		u32 size = shdr->sh_size;
		u32 offset = shdr->sh_offset;

		if (strcmp(name_table + shdr->sh_name, ".resource_table"))
			continue;

		table = (struct resource_table *)(elf_data + offset);

		/* make sure we have the entire table */
		if (offset + size > fw_size || offset + size < size) {
			dev_err(dev, "resource table truncated\n");
			return NULL;
		}

		/* make sure table has at least the header */
		if (sizeof(struct resource_table) > size) {
			dev_err(dev, "header-less resource table\n");
			return NULL;
		}

		/* we don't support any version beyond the first */
		if (table->ver != 1) {
			dev_err(dev, "unsupported fw ver: %d\n", table->ver);
			return NULL;
		}

		/* make sure reserved bytes are zeroes */
		if (table->reserved[0] || table->reserved[1]) {
			dev_err(dev, "non zero reserved bytes\n");
			return NULL;
		}

		/* make sure the offsets array isn't truncated */
		if (table->num * sizeof(table->offset[0]) +
				sizeof(struct resource_table) > size) {
			dev_err(dev, "resource table incomplete\n");
			return NULL;
		}

		if (id)
			*id = i;

		return shdr;
	}

	return NULL;
}

/**
 * rproc_elf_load_rsc_table() - load the resource table
 * @rproc: the rproc handle
 * @fw: the ELF firmware image
 *
 * This function finds the resource table inside the remote processor's
 * firmware, load it into the @cached_table and update @table_ptr.
 *
 * Return: 0 on success, negative errno on failure.
 */
int rproc_elf_load_rsc_table(struct rproc *rproc, const struct firmware *fw)
{
	struct elf32_hdr *ehdr;
	struct elf32_shdr *shdr;
	struct device *dev = &rproc->dev;
	struct resource_table *table = NULL;
	const u8 *elf_data = fw->data;
	size_t tablesz;

	ehdr = (struct elf32_hdr *)elf_data;

	shdr = find_table(dev, ehdr, fw->size, NULL);
	if (!shdr)
		return -EINVAL;

	table = (struct resource_table *)(elf_data + shdr->sh_offset);
	tablesz = shdr->sh_size;

	/*
	 * Create a copy of the resource table. When a virtio device starts
	 * and calls vring_new_virtqueue() the address of the allocated vring
	 * will be stored in the cached_table. Before the device is started,
	 * cached_table will be copied into device memory.
	 */
	rproc->cached_table = kmemdup(table, tablesz, GFP_KERNEL);
	if (!rproc->cached_table)
		return -ENOMEM;

	rproc->table_ptr = rproc->cached_table;
	rproc->table_sz = tablesz;

	return 0;
}
EXPORT_SYMBOL(rproc_elf_load_rsc_table);

/**
 * rproc_elf_find_loaded_rsc_table() - find the loaded resource table
 * @rproc: the rproc handle
 * @fw: the ELF firmware image
 *
 * This function finds the location of the loaded resource table. Don't
 * call this function if the table wasn't loaded yet - it's a bug if you do.
 *
 * Returns the pointer to the resource table if it is found or NULL otherwise.
 * If the table wasn't loaded yet the result is unspecified.
 */
struct resource_table *rproc_elf_find_loaded_rsc_table(struct rproc *rproc,
						       const struct firmware *fw)
{
	struct elf32_hdr *ehdr = (struct elf32_hdr *)fw->data;
	struct elf32_shdr *shdr;
	int id, map;

	shdr = find_table(&rproc->dev, ehdr, fw->size, &id);
	if (!shdr)
		return NULL;

	map = rproc_elf_section_to_map(id, fw->data);

	return rproc_da_to_va(rproc, shdr->sh_addr, shdr->sh_size, map);
}
EXPORT_SYMBOL(rproc_elf_find_loaded_rsc_table);
