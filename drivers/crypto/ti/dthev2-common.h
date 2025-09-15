/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * K3 DTHE V2 crypto accelerator driver
 *
 * Copyright (C) Texas Instruments 2025 - https://www.ti.com
 * Author: T Pratham <t-pratham@ti.com>
 */

#ifndef __TI_DTHEV2_H__
#define __TI_DTHEV2_H__

#include <crypto/aead.h>
#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <crypto/engine.h>
#include <crypto/hash.h>
#include <crypto/internal/aead.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/skcipher.h>
#include <crypto/md5.h>
#include <crypto/sha2.h>

#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/scatterlist.h>
#include <linux/types.h>

#define DTHE_REG_SIZE		4
#define DTHE_DMA_TIMEOUT_MS	2000
/*
 * Size of largest possible key (of all algorithms) to be stored in dthe_tfm_ctx
 * This is currently the keysize of HMAC-SHA512
 */
#define DTHE_MAX_KEYSIZE	(SHA512_BLOCK_SIZE)

enum dthe_hash_alg_sel {
	DTHE_HASH_MD5		= 0,
	DTHE_HASH_SHA1		= BIT(1),
	DTHE_HASH_SHA224	= BIT(2),
	DTHE_HASH_SHA256	= BIT(1) | BIT(2),
	DTHE_HASH_SHA384	= BIT(0),
	DTHE_HASH_SHA512	= BIT(0) | BIT(1)
};

enum dthe_aes_mode {
	DTHE_AES_ECB = 0,
	DTHE_AES_CBC,
	DTHE_AES_GCM,
};

/* Driver specific struct definitions */

/**
 * struct dthe_data - DTHE_V2 driver instance data
 * @dev: Device pointer
 * @regs: Base address of the register space
 * @list: list node for dev
 * @aes_engine: Crypto engine instance for AES Engine
 * @dma_aes_rx: AES Rx DMA Channel
 * @dma_aes_tx: AES Tx DMA Channel
 * @dma_sha_tx: SHA Tx DMA Channel
 * @hash_mutex: Mutex protecting access to HASH engine
 */
struct dthe_data {
	struct device *dev;
	void __iomem *regs;
	struct list_head list;
	struct crypto_engine *aes_engine;

	struct dma_chan *dma_aes_rx;
	struct dma_chan *dma_aes_tx;

	struct dma_chan *dma_sha_tx;
	struct mutex hash_mutex;
};

/**
 * struct dthe_list - device data list head
 * @dev_list: linked list head
 * @lock: Spinlock protecting accesses to the list
 */
struct dthe_list {
	struct list_head dev_list;
	spinlock_t lock;
};

/**
 * struct dthe_tfm_ctx - Transform ctx struct containing ctx for all sub-components of DTHE V2
 * @dev_data: Device data struct pointer
 * @keylen: Key length for algorithms that require a key
 * @authsize: Authentication size for modes with authentication
 * @key: Key
 * @aes_mode: AES mode
 * @hash_mode: Hashing Engine mode
 * @block_size: block size of hash algorithm selected
 * @digest_size: digest size of hash algorithm selected
 * @phash_size: partial hash size of the hash algorithm selected
 * @fallback: Fallback crypto aead instance for GCM mode
 * @ahash_fb: Fallback crypto ahash instance for hashing algorithms
 */
struct dthe_tfm_ctx {
	struct dthe_data *dev_data;
	unsigned int keylen;
	unsigned int authsize;
	u32 key[DTHE_MAX_KEYSIZE / sizeof(u32)];
	union {
		enum dthe_aes_mode aes_mode;
		enum dthe_hash_alg_sel hash_mode;
	};
	u32 block_size;
	u32 digest_size;
	u32 phash_size;
	union {
		struct crypto_aead *aead_fb;
		struct crypto_ahash *ahash_fb;
	};
};

/**
 * struct dthe_aes_req_ctx - AES engine req ctx struct
 * @enc: flag indicating encryption or decryption operation
 * @aes_compl: Completion variable for use in manual completion in case of DMA callback failure
 * @fb_req: Fallback aead_request structure for GCM mode
 */
struct dthe_aes_req_ctx {
	int enc;
	struct completion aes_compl;
	struct aead_request fb_req;
};

/**
 * struct dthe_hash_req_ctx - Hashing engine req ctx struct
 * @phash: buffer to store a partial hash/inner digest from a previous operation
 * @odigest: buffer to store the outer digest from a previous operation
 * @data_buf: buffer to store part of input data to be carried over to next operation
 * @digestcnt: stores the digest count from a previous operation
 * @phash_available: flag indicating if a partial hash from a previous operation is available
 * @buflen: length of input data stored in data_buf
 * @flags: flags for internal use
 * @hash_compl: Completion variable for use in manual completion in case of DMA callback failure
 */
struct dthe_hash_req_ctx {
	u32 phash[SHA512_DIGEST_SIZE / sizeof(u32)];
	u32 odigest[SHA512_DIGEST_SIZE / sizeof(u32)];
	u8 data_buf[SHA512_BLOCK_SIZE];
	u32 digestcnt;
	u8 phash_available;
	u8 buflen;
	u8 flags;
	struct completion hash_compl;
};

/* Struct definitions end */

struct dthe_data *dthe_get_dev(struct dthe_tfm_ctx *ctx);

/**
 * dthe_copy_sg - Copy sg entries from src to dst
 * @dst:		Destination sg to be filled
 * @src:		Source sg to be copied from
 * @buflen:		Number of bytes to be copied
 *
 * Description:
 *   Copy buflen bytes of data from src to dst.
 *
 **/
struct scatterlist *dthe_copy_sg(struct scatterlist *dst,
				 struct scatterlist *src,
				 int buflen);

int dthe_register_aes_algs(void);
void dthe_unregister_aes_algs(void);

int dthe_register_hash_algs(void);
void dthe_unregister_hash_algs(void);

#endif
