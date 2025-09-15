// SPDX-License-Identifier: GPL-2.0-only
/*
 * K3 DTHE V2 crypto accelerator driver
 *
 * Copyright (C) Texas Instruments 2025 - https://www.ti.com
 * Author: T Pratham <t-pratham@ti.com>
 */

#include <crypto/algapi.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <crypto/md5.h>
#include <crypto/sha2.h>

#include "dthev2-common.h"

#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/scatterlist.h>

/* Registers */

// Hashing Engine
#define DTHE_P_HASH_BASE		0x5000
#define DTHE_P_HASH512_ODIGEST_A	0x0200
#define DTHE_P_HASH512_IDIGEST_A	0x0240
#define DTHE_P_HASH512_DIGEST_COUNT	0x0280
#define DTHE_P_HASH512_MODE		0x0284
#define DTHE_P_HASH512_LENGTH		0x0288
#define DTHE_P_HASH512_DATA_IN_START	0x0080
#define DTHE_P_HASH512_DATA_IN_END	0x00FC

#define DTHE_P_HASH_SYSCONFIG		0x0110
#define DTHE_P_HASH_IRQSTATUS		0x0118
#define DTHE_P_HASH_IRQENABLE		0x011C

/* Register write values and macros */
#define DTHE_HASH_SYSCONFIG_INT_EN		BIT(2)
#define DTHE_HASH_SYSCONFIG_DMA_EN		BIT(3)
#define DTHE_HASH_IRQENABLE_EN_ALL		GENMASK(3, 0)
#define DTHE_HASH_IRQSTATUS_OP_READY		BIT(0)
#define DTHE_HASH_IRQSTATUS_IP_READY		BIT(1)
#define DTHE_HASH_IRQSTATUS_PH_READY		BIT(2)
#define DTHE_HASH_IRQSTATUS_CTX_READY		BIT(3)

#define DTHE_HASH_MODE_ALG_MASK			GENMASK(2, 0)
#define DTHE_HASH_MODE_USE_ALG_CONST		BIT(3)
#define DTHE_HASH_MODE_CLOSE_HASH		BIT(4)
#define DTHE_HASH_MODE_HMAC_KEY_PROCESSING	BIT(5)
#define DTHE_HASH_MODE_HMAC_OUTER_HASH		BIT(7)
/* Flag to indicate HMAC mode. This bit is not written in hardware registers */
#define DTHE_HASH_IS_HMAC			BIT(8)

/* Misc */
#define MD5_BLOCK_SIZE				(MD5_BLOCK_WORDS * 4)
#define HMAC_SHA512_MAX_KEYSIZE			(SHA512_BLOCK_SIZE)
#define HMAC_SHA256_MAX_KEYSIZE			(SHA256_BLOCK_SIZE)
#define HMAC_MD5_MAX_KEYSIZE			(MD5_BLOCK_SIZE)

enum dthe_hash_dma_callback_src {
	DMA_CALLBACK_FROM_UPDATE = 0,
	DMA_CALLBACK_FROM_FINAL,
	DMA_CALLBACK_FROM_FINUP,
};

struct sha512_export_state {
	struct sha512_state state;
	u32 odigest[SHA512_DIGEST_SIZE / sizeof(u32)];
	u32 key[HMAC_SHA512_MAX_KEYSIZE / sizeof(u32)];
	unsigned int keylen;
	u8 buflen;
	u8 flags;
};

struct sha256_export_state {
	struct sha256_state state;
	u32 odigest[SHA256_DIGEST_SIZE / sizeof(u32)];
	u32 key[HMAC_SHA256_MAX_KEYSIZE / sizeof(u32)];
	unsigned int keylen;
	u8 buflen;
	u8 flags;
};

struct md5_export_state {
	struct md5_state state;
	u32 odigest[MD5_DIGEST_SIZE / sizeof(u32)];
	u32 key[HMAC_MD5_MAX_KEYSIZE / sizeof(u32)];
	unsigned int keylen;
	u8 buflen;
	u8 flags;
};

static int cnt;

static int dthe_ahash_init_fb(struct crypto_ahash *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(tfm);
	struct dthe_data *dev_data = dthe_get_dev(ctx);
	const char *alg_name = crypto_tfm_alg_name(&tfm->base);

	ctx->ahash_fb = crypto_alloc_ahash(alg_name, 0, CRYPTO_ALG_ASYNC);
	if (IS_ERR(ctx->ahash_fb)) {
		dev_err(dev_data->dev, "Failed to allocate fallback driver: %s\n",
			alg_name);
		return PTR_ERR(ctx->ahash_fb);
	}

	crypto_ahash_set_reqsize(tfm, sizeof(struct ahash_request) +
				 crypto_ahash_reqsize(ctx->ahash_fb));

	return 0;
}

static void dthe_hash_write_zero_message(enum dthe_hash_alg_sel mode, void *dst)
{
	switch (mode) {
	case DTHE_HASH_SHA512:
		memcpy(dst, sha512_zero_message_hash, SHA512_DIGEST_SIZE);
		break;
	case DTHE_HASH_SHA384:
		memcpy(dst, sha384_zero_message_hash, SHA384_DIGEST_SIZE);
		break;
	case DTHE_HASH_SHA256:
		memcpy(dst, sha256_zero_message_hash, SHA256_DIGEST_SIZE);
		break;
	case DTHE_HASH_SHA224:
		memcpy(dst, sha224_zero_message_hash, SHA224_DIGEST_SIZE);
		break;
	case DTHE_HASH_MD5:
		memcpy(dst, md5_zero_message_hash, MD5_DIGEST_SIZE);
		break;
	default:
		break;
	}
}

static int dthe_hmac_write_zero_message(struct ahash_request *req, void *dst)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct ahash_request *subreq = ahash_request_ctx(req);
	int ret;

	ahash_request_set_tfm(subreq, ctx->ahash_fb);
	ahash_request_set_callback(subreq, req->base.flags & CRYPTO_TFM_REQ_MAY_SLEEP,
				   req->base.complete, req->base.data);
	ahash_request_set_crypt(subreq, req->src, dst, req->nbytes);

	ret = crypto_ahash_digest(subreq);

	return ret;
}

static int dthe_sha512_init_tfm(struct crypto_ahash *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(tfm);
	struct dthe_data *dev_data = dthe_get_dev(ctx);

	if (!dev_data)
		return -ENODEV;

	crypto_ahash_set_reqsize(tfm, sizeof(struct dthe_hash_req_ctx));

	ctx->hash_mode = DTHE_HASH_SHA512;
	ctx->block_size = SHA512_BLOCK_SIZE;
	ctx->digest_size = SHA512_DIGEST_SIZE;
	ctx->phash_size = SHA512_DIGEST_SIZE;

	return 0;
}

static int dthe_sha384_init_tfm(struct crypto_ahash *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(tfm);
	struct dthe_data *dev_data = dthe_get_dev(ctx);

	if (!dev_data)
		return -ENODEV;

	crypto_ahash_set_reqsize(tfm, sizeof(struct dthe_hash_req_ctx));

	ctx->hash_mode = DTHE_HASH_SHA384;
	ctx->block_size = SHA384_BLOCK_SIZE;
	ctx->digest_size = SHA384_DIGEST_SIZE;
	ctx->phash_size = SHA512_DIGEST_SIZE;

	return 0;
}

static int dthe_sha256_init_tfm(struct crypto_ahash *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(tfm);
	struct dthe_data *dev_data = dthe_get_dev(ctx);

	if (!dev_data)
		return -ENODEV;

	crypto_ahash_set_reqsize(tfm, sizeof(struct dthe_hash_req_ctx));

	ctx->hash_mode = DTHE_HASH_SHA256;
	ctx->block_size = SHA256_BLOCK_SIZE;
	ctx->digest_size = SHA256_DIGEST_SIZE;
	ctx->phash_size = SHA256_DIGEST_SIZE;

	return 0;
}

static int dthe_sha224_init_tfm(struct crypto_ahash *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(tfm);
	struct dthe_data *dev_data = dthe_get_dev(ctx);

	if (!dev_data)
		return -ENODEV;

	crypto_ahash_set_reqsize(tfm, sizeof(struct dthe_hash_req_ctx));

	ctx->hash_mode = DTHE_HASH_SHA224;
	ctx->block_size = SHA224_BLOCK_SIZE;
	ctx->digest_size = SHA224_DIGEST_SIZE;
	ctx->phash_size = SHA256_DIGEST_SIZE;

	return 0;
}

static int dthe_md5_init_tfm(struct crypto_ahash *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(tfm);
	struct dthe_data *dev_data = dthe_get_dev(ctx);

	if (!dev_data)
		return -ENODEV;

	crypto_ahash_set_reqsize(tfm, sizeof(struct dthe_hash_req_ctx));

	ctx->hash_mode = DTHE_HASH_MD5;
	ctx->block_size = MD5_BLOCK_SIZE;
	ctx->digest_size = MD5_DIGEST_SIZE;
	ctx->phash_size = MD5_DIGEST_SIZE;

	return 0;
}

static int dthe_hmac_sha512_init_tfm(struct crypto_ahash *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(tfm);
	struct dthe_data *dev_data = dthe_get_dev(ctx);
	int ret;

	if (!dev_data)
		return -ENODEV;

	crypto_ahash_set_reqsize(tfm, sizeof(struct dthe_hash_req_ctx));

	ctx->hash_mode = DTHE_HASH_SHA512 | DTHE_HASH_IS_HMAC;
	ctx->block_size = SHA512_BLOCK_SIZE;
	ctx->digest_size = SHA512_DIGEST_SIZE;
	ctx->phash_size = SHA512_DIGEST_SIZE;
	ret = dthe_ahash_init_fb(tfm);

	return ret;
}

static int dthe_hmac_sha384_init_tfm(struct crypto_ahash *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(tfm);
	struct dthe_data *dev_data = dthe_get_dev(ctx);
	int ret;

	if (!dev_data)
		return -ENODEV;

	crypto_ahash_set_reqsize(tfm, sizeof(struct dthe_hash_req_ctx));

	ctx->hash_mode = DTHE_HASH_SHA384 | DTHE_HASH_IS_HMAC;
	ctx->block_size = SHA384_BLOCK_SIZE;
	ctx->digest_size = SHA384_DIGEST_SIZE;
	ctx->phash_size = SHA512_DIGEST_SIZE;
	ret = dthe_ahash_init_fb(tfm);

	return ret;
}

static int dthe_hmac_sha256_init_tfm(struct crypto_ahash *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(tfm);
	struct dthe_data *dev_data = dthe_get_dev(ctx);
	int ret;

	if (!dev_data)
		return -ENODEV;

	crypto_ahash_set_reqsize(tfm, sizeof(struct dthe_hash_req_ctx));

	ctx->hash_mode = DTHE_HASH_SHA256 | DTHE_HASH_IS_HMAC;
	ctx->block_size = SHA256_BLOCK_SIZE;
	ctx->digest_size = SHA256_DIGEST_SIZE;
	ctx->phash_size = SHA256_DIGEST_SIZE;
	ret = dthe_ahash_init_fb(tfm);

	return ret;
}

static int dthe_hmac_sha224_init_tfm(struct crypto_ahash *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(tfm);
	struct dthe_data *dev_data = dthe_get_dev(ctx);
	int ret;

	if (!dev_data)
		return -ENODEV;

	crypto_ahash_set_reqsize(tfm, sizeof(struct dthe_hash_req_ctx));

	ctx->hash_mode = DTHE_HASH_SHA224 | DTHE_HASH_IS_HMAC;
	ctx->block_size = SHA224_BLOCK_SIZE;
	ctx->digest_size = SHA224_DIGEST_SIZE;
	ctx->phash_size = SHA256_DIGEST_SIZE;
	ret = dthe_ahash_init_fb(tfm);

	return ret;
}

static int dthe_hmac_md5_init_tfm(struct crypto_ahash *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(tfm);
	struct dthe_data *dev_data = dthe_get_dev(ctx);
	int ret;

	if (!dev_data)
		return -ENODEV;

	crypto_ahash_set_reqsize(tfm, sizeof(struct dthe_hash_req_ctx));

	ctx->hash_mode = DTHE_HASH_MD5 | DTHE_HASH_IS_HMAC;
	ctx->block_size = MD5_BLOCK_SIZE;
	ctx->digest_size = MD5_DIGEST_SIZE;
	ctx->phash_size = MD5_DIGEST_SIZE;
	ret = dthe_ahash_init_fb(tfm);

	return ret;
}

static void dthe_hash_exit_tfm(struct crypto_ahash *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(tfm);

	if (ctx->hash_mode & DTHE_HASH_IS_HMAC)
		crypto_free_ahash(ctx->ahash_fb);
}

static void dthe_hash_dma_in_callback(void *data)
{
	struct ahash_request *req = (struct ahash_request *)data;
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);

	complete(&rctx->hash_compl);
}

static int dthe_hash_dma_start(struct ahash_request *req, struct scatterlist *src,
			       int src_nents, size_t len)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);
	struct dthe_data *dev_data = dthe_get_dev(ctx);
	struct dma_slave_config cfg;
	struct device *tx_dev;
	struct dma_async_tx_descriptor *desc_out;
	int mapped_nents;
	enum dma_data_direction src_dir = DMA_TO_DEVICE;
	u32 hash_mode;
	int ret = 0;
	u32 *dst;
	u32 dst_len;
	void __iomem *sha_base_reg = dev_data->regs + DTHE_P_HASH_BASE;

	u32 hash_sysconfig_val = DTHE_HASH_SYSCONFIG_INT_EN | DTHE_HASH_SYSCONFIG_DMA_EN;
	u32 hash_irqenable_val = DTHE_HASH_IRQENABLE_EN_ALL;

	// Config SHA DMA channel as per SHA mode
	memzero_explicit(&cfg, sizeof(cfg));

	cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.dst_maxburst = ctx->block_size / 4;

	// HACK: Delay to workaround DMA driver issue
	cnt++;
	if (cnt % 20 == 0) {
		unsigned long delay = jiffies + usecs_to_jiffies(1000);

		while (time_before(jiffies, delay))
			cond_resched();
		cnt = 0;
	}

	mutex_lock(&dev_data->hash_mutex);

	ret = dmaengine_slave_config(dev_data->dma_sha_tx, &cfg);
	if (ret) {
		dev_err(dev_data->dev, "Can't configure OUT2 dmaengine slave: %d\n", ret);
		goto hash_err;
	}

	tx_dev = dmaengine_get_dma_device(dev_data->dma_sha_tx);
	if (!tx_dev) {
		ret = -ENODEV;
		goto hash_err;
	}

	mapped_nents = dma_map_sg(tx_dev, src, src_nents, src_dir);
	if (mapped_nents == 0) {
		ret = -EINVAL;
		goto hash_err;
	}

	desc_out = dmaengine_prep_slave_sg(dev_data->dma_sha_tx, src, mapped_nents,
					   DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc_out) {
		dev_err(dev_data->dev, "OUT prep_slave_sg() failed\n");
		ret = -EINVAL;
		goto hash_prep_err;
	}

	desc_out->callback = dthe_hash_dma_in_callback;
	desc_out->callback_param = req;

	init_completion(&rctx->hash_compl);

	writel_relaxed(hash_sysconfig_val, sha_base_reg + DTHE_P_HASH_SYSCONFIG);
	writel_relaxed(hash_irqenable_val, sha_base_reg + DTHE_P_HASH_IRQENABLE);

	hash_mode = (ctx->hash_mode & DTHE_HASH_MODE_ALG_MASK);

	if (rctx->flags == DMA_CALLBACK_FROM_FINAL || rctx->flags == DMA_CALLBACK_FROM_FINUP) {
		hash_mode |= DTHE_HASH_MODE_CLOSE_HASH;
		if (ctx->hash_mode & DTHE_HASH_IS_HMAC)
			hash_mode |= DTHE_HASH_MODE_HMAC_OUTER_HASH;
	}

	if (rctx->phash_available) {
		for (int i = 0; i < ctx->phash_size / sizeof(u32); ++i)
			writel_relaxed(rctx->phash[i],
				       sha_base_reg +
				       DTHE_P_HASH512_IDIGEST_A +
				       (DTHE_REG_SIZE * i));
		if (ctx->hash_mode & DTHE_HASH_IS_HMAC) {
			for (int i = 0; i < ctx->phash_size / sizeof(u32); ++i)
				writel_relaxed(rctx->odigest[i],
					       sha_base_reg +
					       DTHE_P_HASH512_ODIGEST_A +
					       (DTHE_REG_SIZE * i));
		}

		writel_relaxed(rctx->digestcnt,
			       sha_base_reg + DTHE_P_HASH512_DIGEST_COUNT);
	} else if (ctx->hash_mode & DTHE_HASH_IS_HMAC) {
		hash_mode |= DTHE_HASH_MODE_HMAC_KEY_PROCESSING;

		for (int i = 0; i < (ctx->keylen / 2) / sizeof(u32); ++i)
			writel_relaxed(ctx->key[i],
				       sha_base_reg +
				       DTHE_P_HASH512_ODIGEST_A +
				       (DTHE_REG_SIZE * i));
		for (int i = 0; i < (ctx->keylen / 2) / sizeof(u32); ++i)
			writel_relaxed(ctx->key[i + (ctx->keylen / 2) / sizeof(u32)],
				       sha_base_reg +
				       DTHE_P_HASH512_IDIGEST_A +
				       (DTHE_REG_SIZE * i));
	} else {
		hash_mode |= DTHE_HASH_MODE_USE_ALG_CONST;
	}

	writel_relaxed(hash_mode, sha_base_reg + DTHE_P_HASH512_MODE);
	writel_relaxed(len, sha_base_reg + DTHE_P_HASH512_LENGTH);

	dmaengine_submit(desc_out);

	dma_async_issue_pending(dev_data->dma_sha_tx);

	ret = wait_for_completion_timeout(&rctx->hash_compl,
					  msecs_to_jiffies(DTHE_DMA_TIMEOUT_MS));
	if (!ret) {
		dmaengine_terminate_sync(dev_data->dma_sha_tx);
		ret = -ETIMEDOUT;
	} else {
		ret = 0;
	}

	if (rctx->flags == DMA_CALLBACK_FROM_UPDATE) {
		// If coming from update, we need to read the phash and store it for future
		dst = rctx->phash;
		dst_len = ctx->phash_size / sizeof(u32);
	} else {
		// If coming from finup or final, we need to read the final digest
		dst = (u32 *)req->result;
		dst_len = ctx->digest_size / sizeof(u32);
	}

	for (int i = 0; i < dst_len; ++i)
		dst[i] = readl_relaxed(sha_base_reg +
				       DTHE_P_HASH512_IDIGEST_A +
				       (DTHE_REG_SIZE * i));

	if (ctx->hash_mode & DTHE_HASH_IS_HMAC) {
		for (int i = 0; i < dst_len; ++i)
			rctx->odigest[i] = readl_relaxed(sha_base_reg +
							 DTHE_P_HASH512_ODIGEST_A +
							 (DTHE_REG_SIZE * i));
	}

	rctx->digestcnt = readl_relaxed(sha_base_reg + DTHE_P_HASH512_DIGEST_COUNT);
	rctx->phash_available = 1;

hash_prep_err:
	dma_unmap_sg(tx_dev, src, src_nents, src_dir);
hash_err:
	mutex_unlock(&dev_data->hash_mutex);
	ahash_request_complete(req, ret);

	return ret;
}

static int dthe_hash_init(struct ahash_request *req)
{
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);

	rctx->phash_available = 0;
	rctx->buflen = 0;
	rctx->digestcnt = 0;

	return 0;
}

static int dthe_hash_update(struct ahash_request *req)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);

	struct scatterlist *src, *sg;
	int src_nents = 0;
	int in_nents = sg_nents_for_len(req->src, req->nbytes);
	unsigned int tot_len;
	unsigned int len_to_process;
	unsigned int len_to_buffer;
	int ret = 0;

	if (req->nbytes == 0) {
		if (!rctx->phash_available && !rctx->buflen) {
			if (ctx->hash_mode & DTHE_HASH_IS_HMAC)
				ret = dthe_hmac_write_zero_message(req, rctx->phash);
			else
				dthe_hash_write_zero_message(ctx->hash_mode, rctx->phash);
		}

		return ret;
	}

	tot_len = rctx->buflen + req->nbytes;
	len_to_process = tot_len - (tot_len % ctx->block_size);
	len_to_buffer = ((len_to_process == 0) ? req->nbytes : (tot_len % ctx->block_size));

	if (tot_len % ctx->block_size == 0) {
		len_to_process -= ctx->block_size;
		if (tot_len == ctx->block_size)
			len_to_buffer = req->nbytes;
		else
			len_to_buffer = ctx->block_size;
	}

	if (len_to_process == 0) {
		sg_copy_to_buffer(req->src, in_nents, rctx->data_buf + rctx->buflen, len_to_buffer);
		rctx->buflen += len_to_buffer;
		return 0;
	}

	if (len_to_buffer < req->nbytes)
		src_nents = sg_nents_for_len(req->src, req->nbytes - len_to_buffer);
	if (rctx->buflen > 0)
		src_nents++;

	src = kcalloc(src_nents, sizeof(struct scatterlist), GFP_KERNEL);
	if (!src)
		return -ENOMEM;

	sg_init_table(src, src_nents);
	sg = src;

	if (rctx->buflen > 0) {
		sg_set_buf(sg, rctx->data_buf, rctx->buflen);
		sg = sg_next(sg);
	}
	if (len_to_process > rctx->buflen)
		dthe_copy_sg(sg, req->src, len_to_process - rctx->buflen);

	rctx->flags = DMA_CALLBACK_FROM_UPDATE;
	ret = dthe_hash_dma_start(req, src, src_nents, len_to_process);

	sg_pcopy_to_buffer(req->src, in_nents, rctx->data_buf,
			   len_to_buffer, req->nbytes - len_to_buffer);
	rctx->buflen = len_to_buffer;

	kfree(src);

	return ret;
}

static int dthe_hash_final(struct ahash_request *req)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);
	struct scatterlist *src;

	int ret = 0;

	if (rctx->buflen > 0) {
		src = kzalloc(sizeof(*src), GFP_KERNEL);
		if (!src)
			return -ENOMEM;

		/* Certain DMA restrictions forced us to send data in multiples of BLOCK_SIZE
		 * bytes. So, add a padding 0s at the end of src scatterlist if data is not a
		 * multiple of block_size bytes. The extra data is ignored by the DTHE hardware.
		 */
		for (int i = rctx->buflen; i < ctx->block_size; ++i)
			rctx->data_buf[i] = 0;

		sg_init_one(src, rctx->data_buf, ctx->block_size);

		rctx->flags = DMA_CALLBACK_FROM_FINAL;
		ret = dthe_hash_dma_start(req, src, 1, rctx->buflen);

		kfree(src);

		return ret;
	} else if (!rctx->phash_available) {
		if (ctx->hash_mode & DTHE_HASH_IS_HMAC)
			ret = dthe_hmac_write_zero_message(req, req->result);
		else
			dthe_hash_write_zero_message(ctx->hash_mode, req->result);

		return ret;
	}

	memcpy(req->result, rctx->phash, ctx->digest_size);

	return 0;
}

static int dthe_hash_finup(struct ahash_request *req)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);

	unsigned int tot_len = rctx->buflen + req->nbytes;
	unsigned int pad_len = 0;
	struct scatterlist *src, *sg;
	int src_nents = 0;
	u8 pad_buf[SHA512_BLOCK_SIZE];
	int ret = 0;

	if (tot_len == 0) {
		if (rctx->phash_available)
			memcpy(req->result, rctx->phash, ctx->digest_size);
		else if (ctx->hash_mode & DTHE_HASH_IS_HMAC)
			ret = dthe_hmac_write_zero_message(req, req->result);
		else
			dthe_hash_write_zero_message(ctx->hash_mode, req->result);

		return ret;
	}

	if (tot_len % ctx->block_size)
		pad_len = ctx->block_size - (tot_len % ctx->block_size);

	if (req->nbytes > 0)
		src_nents = sg_nents_for_len(req->src, req->nbytes);
	if (rctx->buflen > 0)
		src_nents++;
	if (pad_len > 0)
		src_nents++;

	src = kcalloc(src_nents, sizeof(struct scatterlist), GFP_KERNEL);
	if (!src)
		return -ENOMEM;

	sg_init_table(src, src_nents);
	sg = src;

	if (rctx->buflen > 0) {
		sg_set_buf(sg, rctx->data_buf, rctx->buflen);
		sg = sg_next(sg);
	}
	if (req->nbytes > 0)
		sg = dthe_copy_sg(sg, req->src, req->nbytes);

	/* Padding 0s in an additional nent at the end. See comment in dthe_hash_final function */
	if (pad_len > 0) {
		memset(pad_buf, 0, pad_len);
		sg_set_buf(sg, pad_buf, pad_len);
	}

	rctx->flags = DMA_CALLBACK_FROM_FINUP;
	ret = dthe_hash_dma_start(req, src, src_nents, tot_len);

	kfree(src);

	return ret;
}

static int dthe_hash_digest(struct ahash_request *req)
{
	dthe_hash_init(req);
	return dthe_hash_finup(req);
}

static int dthe_hmac_setkey(struct crypto_ahash *tfm, const u8 *key,
			    unsigned int keylen)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(tfm);
	unsigned int max_keysize;
	const char *hash_alg_name, *alg_name;

	memzero_explicit(ctx->key, HMAC_SHA512_MAX_KEYSIZE);
	alg_name = crypto_ahash_alg_name(tfm);

	if (strcmp(alg_name, "hmac(sha512)") == 0) {
		max_keysize = HMAC_SHA512_MAX_KEYSIZE;
		hash_alg_name = "sha512";
	} else if (strcmp(alg_name, "hmac(sha384)") == 0) {
		max_keysize = HMAC_SHA512_MAX_KEYSIZE;
		hash_alg_name = "sha384";
	} else if (strcmp(alg_name, "hmac(sha256)") == 0) {
		max_keysize = HMAC_SHA256_MAX_KEYSIZE;
		hash_alg_name = "sha256";
	} else if (strcmp(alg_name, "hmac(sha224)") == 0) {
		max_keysize = HMAC_SHA256_MAX_KEYSIZE;
		hash_alg_name = "sha224";
	} else if (strcmp(alg_name, "hmac(md5)") == 0) {
		max_keysize = MD5_BLOCK_SIZE;
		hash_alg_name = "md5";
	} else {
		return -EINVAL;
	}

	if (keylen > max_keysize) {
		struct crypto_shash *ktfm = crypto_alloc_shash(hash_alg_name, 0, 0);
		SHASH_DESC_ON_STACK(desc, ktfm);
		int err;

		desc->tfm = ktfm;
		err = crypto_shash_digest(desc, key, keylen, (u8 *)ctx->key);

		crypto_free_shash(ktfm);

		if (err)
			return err;
	} else {
		memcpy(ctx->key, key, keylen);
	}

	ctx->keylen = max_keysize;

	return crypto_ahash_setkey(ctx->ahash_fb, key, keylen);
}

static int dthe_sha512_export(struct ahash_request *req, void *out)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);
	struct sha512_export_state *state = out;

	state->state.count[0] = rctx->digestcnt;
	state->buflen = rctx->buflen;
	state->flags = rctx->flags;
	memcpy(state->state.state, rctx->phash, ctx->phash_size);
	memcpy(state->state.buf, rctx->data_buf, rctx->buflen);

	return 0;
}

static int dthe_sha512_import(struct ahash_request *req, const void *in)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);
	const struct sha512_export_state *state = in;

	rctx->buflen = state->buflen;
	rctx->digestcnt = state->state.count[0];
	rctx->flags = state->flags;
	rctx->phash_available = ((rctx->digestcnt) ? 1 : 0);
	ctx->hash_mode = DTHE_HASH_SHA512;
	ctx->block_size = SHA512_BLOCK_SIZE;
	ctx->digest_size = SHA512_DIGEST_SIZE;
	ctx->phash_size = SHA512_DIGEST_SIZE;
	memcpy(rctx->phash, state->state.state, ctx->phash_size);
	memcpy(rctx->data_buf, state->state.buf, rctx->buflen);

	return 0;
}

static int dthe_sha384_import(struct ahash_request *req, const void *in)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);
	const struct sha512_export_state *state = in;

	rctx->buflen = state->buflen;
	rctx->digestcnt = state->state.count[0];
	rctx->flags = state->flags;
	rctx->phash_available = ((rctx->digestcnt) ? 1 : 0);
	ctx->hash_mode = DTHE_HASH_SHA384;
	ctx->block_size = SHA384_BLOCK_SIZE;
	ctx->digest_size = SHA384_DIGEST_SIZE;
	ctx->phash_size = SHA512_DIGEST_SIZE;
	memcpy(rctx->phash, state->state.state, ctx->phash_size);
	memcpy(rctx->data_buf, state->state.buf, rctx->buflen);

	return 0;
}

static int dthe_sha256_export(struct ahash_request *req, void *out)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);
	struct sha256_export_state *state = out;

	state->state.count = rctx->digestcnt;
	state->buflen = rctx->buflen;
	state->flags = rctx->flags;
	memcpy(state->state.state, rctx->phash, ctx->phash_size);
	memcpy(state->state.buf, rctx->data_buf, rctx->buflen);

	return 0;
}

static int dthe_sha256_import(struct ahash_request *req, const void *in)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);
	const struct sha256_export_state *state = in;

	rctx->buflen = state->buflen;
	rctx->digestcnt = state->state.count;
	rctx->flags = state->flags;
	rctx->phash_available = ((rctx->digestcnt) ? 1 : 0);
	ctx->hash_mode = DTHE_HASH_SHA256;
	ctx->block_size = SHA256_BLOCK_SIZE;
	ctx->digest_size = SHA256_DIGEST_SIZE;
	ctx->phash_size = SHA256_DIGEST_SIZE;
	memcpy(rctx->phash, state->state.state, ctx->phash_size);
	memcpy(rctx->data_buf, state->state.buf, rctx->buflen);

	return 0;
}

static int dthe_sha224_import(struct ahash_request *req, const void *in)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);
	const struct sha256_export_state *state = in;

	rctx->buflen = state->buflen;
	rctx->digestcnt = state->state.count;
	rctx->flags = state->flags;
	rctx->phash_available = ((rctx->digestcnt) ? 1 : 0);
	ctx->hash_mode = DTHE_HASH_SHA224;
	ctx->block_size = SHA224_BLOCK_SIZE;
	ctx->digest_size = SHA224_DIGEST_SIZE;
	ctx->phash_size = SHA256_DIGEST_SIZE;
	memcpy(rctx->phash, state->state.state, ctx->phash_size);
	memcpy(rctx->data_buf, state->state.buf, rctx->buflen);

	return 0;
}

static int dthe_md5_export(struct ahash_request *req, void *out)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);
	struct md5_export_state *state = out;

	state->state.byte_count = rctx->digestcnt;
	state->buflen = rctx->buflen;
	state->flags = rctx->flags;
	memcpy(state->state.hash, rctx->phash, ctx->phash_size);
	memcpy(state->state.block, rctx->data_buf, rctx->buflen);

	return 0;
}

static int dthe_md5_import(struct ahash_request *req, const void *in)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);
	const struct md5_export_state *state = in;

	rctx->buflen = state->buflen;
	rctx->digestcnt = state->state.byte_count;
	rctx->flags = state->flags;
	rctx->phash_available = ((rctx->digestcnt) ? 1 : 0);
	ctx->hash_mode = DTHE_HASH_MD5;
	ctx->block_size = MD5_BLOCK_SIZE;
	ctx->digest_size = MD5_DIGEST_SIZE;
	ctx->phash_size = MD5_DIGEST_SIZE;
	memcpy(rctx->phash, state->state.hash, ctx->phash_size);
	memcpy(rctx->data_buf, state->state.block, rctx->buflen);

	return 0;
}

static int dthe_hmac_sha512_export(struct ahash_request *req, void *out)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);
	struct sha512_export_state *state = out;

	state->state.count[0] = rctx->digestcnt;
	state->buflen = rctx->buflen;
	state->flags = rctx->flags;
	memcpy(state->state.state, rctx->phash, ctx->phash_size);
	memcpy(state->state.buf, rctx->data_buf, rctx->buflen);
	memcpy(state->key, ctx->key, HMAC_SHA512_MAX_KEYSIZE);
	memcpy(state->odigest, rctx->odigest, SHA512_DIGEST_SIZE);
	state->keylen = ctx->keylen;

	return 0;
}

static int dthe_hmac_sha512_import(struct ahash_request *req, const void *in)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);
	const struct sha512_export_state *state = in;

	rctx->buflen = state->buflen;
	rctx->digestcnt = state->state.count[0];
	rctx->flags = state->flags;
	rctx->phash_available = ((rctx->digestcnt) ? 1 : 0);
	ctx->hash_mode = DTHE_HASH_SHA512 | DTHE_HASH_IS_HMAC;
	ctx->block_size = SHA512_BLOCK_SIZE;
	ctx->digest_size = SHA512_DIGEST_SIZE;
	ctx->phash_size = SHA512_DIGEST_SIZE;
	memcpy(rctx->phash, state->state.state, ctx->phash_size);
	memcpy(rctx->data_buf, state->state.buf, rctx->buflen);
	memcpy(ctx->key, state->key, HMAC_SHA512_MAX_KEYSIZE);
	memcpy(rctx->odigest, state->odigest, SHA512_DIGEST_SIZE);
	ctx->keylen = state->keylen;

	return 0;
}

static int dthe_hmac_sha384_import(struct ahash_request *req, const void *in)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);
	const struct sha512_export_state *state = in;

	rctx->buflen = state->buflen;
	rctx->digestcnt = state->state.count[0];
	rctx->flags = state->flags;
	rctx->phash_available = ((rctx->digestcnt) ? 1 : 0);
	ctx->hash_mode = DTHE_HASH_SHA384 | DTHE_HASH_IS_HMAC;
	ctx->block_size = SHA384_BLOCK_SIZE;
	ctx->digest_size = SHA384_DIGEST_SIZE;
	ctx->phash_size = SHA512_DIGEST_SIZE;
	memcpy(rctx->phash, state->state.state, ctx->phash_size);
	memcpy(rctx->data_buf, state->state.buf, rctx->buflen);
	memcpy(ctx->key, state->key, HMAC_SHA512_MAX_KEYSIZE);
	memcpy(rctx->odigest, state->odigest, SHA512_DIGEST_SIZE);
	ctx->keylen = state->keylen;

	return 0;
}

static int dthe_hmac_sha256_export(struct ahash_request *req, void *out)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);
	struct sha256_export_state *state = out;

	state->state.count = rctx->digestcnt;
	state->buflen = rctx->buflen;
	state->flags = rctx->flags;
	memcpy(state->state.state, rctx->phash, ctx->phash_size);
	memcpy(state->state.buf, rctx->data_buf, rctx->buflen);
	memcpy(state->key, ctx->key, HMAC_SHA256_MAX_KEYSIZE);
	memcpy(state->odigest, rctx->odigest, SHA256_DIGEST_SIZE);
	state->keylen = ctx->keylen;

	return 0;
}

static int dthe_hmac_sha256_import(struct ahash_request *req, const void *in)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);
	const struct sha256_export_state *state = in;

	rctx->buflen = state->buflen;
	rctx->digestcnt = state->state.count;
	rctx->flags = state->flags;
	rctx->phash_available = ((rctx->digestcnt) ? 1 : 0);
	ctx->hash_mode = DTHE_HASH_SHA256 | DTHE_HASH_IS_HMAC;
	ctx->block_size = SHA256_BLOCK_SIZE;
	ctx->digest_size = SHA256_DIGEST_SIZE;
	ctx->phash_size = SHA256_DIGEST_SIZE;
	memcpy(rctx->phash, state->state.state, ctx->phash_size);
	memcpy(rctx->data_buf, state->state.buf, rctx->buflen);
	memcpy(ctx->key, state->key, HMAC_SHA256_MAX_KEYSIZE);
	memcpy(rctx->odigest, state->odigest, SHA256_DIGEST_SIZE);
	ctx->keylen = state->keylen;

	return 0;
}

static int dthe_hmac_sha224_import(struct ahash_request *req, const void *in)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);
	const struct sha256_export_state *state = in;

	rctx->buflen = state->buflen;
	rctx->digestcnt = state->state.count;
	rctx->flags = state->flags;
	rctx->phash_available = ((rctx->digestcnt) ? 1 : 0);
	ctx->hash_mode = DTHE_HASH_SHA224 | DTHE_HASH_IS_HMAC;
	ctx->block_size = SHA224_BLOCK_SIZE;
	ctx->digest_size = SHA224_DIGEST_SIZE;
	ctx->phash_size = SHA256_DIGEST_SIZE;
	memcpy(rctx->phash, state->state.state, ctx->phash_size);
	memcpy(rctx->data_buf, state->state.buf, rctx->buflen);
	memcpy(ctx->key, state->key, HMAC_SHA256_MAX_KEYSIZE);
	memcpy(rctx->odigest, state->odigest, SHA256_DIGEST_SIZE);
	ctx->keylen = state->keylen;

	return 0;
}

static int dthe_hmac_md5_export(struct ahash_request *req, void *out)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);
	struct md5_export_state *state = out;

	state->state.byte_count = rctx->digestcnt;
	state->buflen = rctx->buflen;
	state->flags = rctx->flags;
	memcpy(state->state.hash, rctx->phash, ctx->phash_size);
	memcpy(state->state.block, rctx->data_buf, rctx->buflen);
	memcpy(state->key, ctx->key, MD5_BLOCK_SIZE);
	memcpy(state->odigest, rctx->odigest, MD5_DIGEST_SIZE);
	state->keylen = ctx->keylen;

	return 0;
}

static int dthe_hmac_md5_import(struct ahash_request *req, const void *in)
{
	struct dthe_tfm_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct dthe_hash_req_ctx *rctx = ahash_request_ctx(req);
	const struct md5_export_state *state = in;

	rctx->buflen = state->buflen;
	rctx->digestcnt = state->state.byte_count;
	rctx->flags = state->flags;
	rctx->phash_available = ((rctx->digestcnt) ? 1 : 0);
	ctx->hash_mode = DTHE_HASH_MD5 | DTHE_HASH_IS_HMAC;
	ctx->block_size = MD5_BLOCK_SIZE;
	ctx->digest_size = MD5_DIGEST_SIZE;
	ctx->phash_size = MD5_DIGEST_SIZE;
	memcpy(rctx->phash, state->state.hash, ctx->phash_size);
	memcpy(rctx->data_buf, state->state.block, rctx->buflen);
	memcpy(ctx->key, state->key, MD5_BLOCK_SIZE);
	memcpy(rctx->odigest, state->odigest, MD5_DIGEST_SIZE);
	ctx->keylen = state->keylen;

	return 0;
}

static struct ahash_alg hash_algs[] = {
	{
		.init_tfm	= dthe_sha512_init_tfm,
		.init		= dthe_hash_init,
		.update		= dthe_hash_update,
		.final		= dthe_hash_final,
		.finup		= dthe_hash_finup,
		.digest		= dthe_hash_digest,
		.export		= dthe_sha512_export,
		.import		= dthe_sha512_import,
		.halg	= {
			.digestsize = SHA512_DIGEST_SIZE,
			.statesize = sizeof(struct sha512_export_state),
			.base = {
				.cra_name	 = "sha512",
				.cra_driver_name = "sha512-dthev2",
				.cra_priority	 = 400,
				.cra_flags	 = CRYPTO_ALG_TYPE_AHASH |
						   CRYPTO_ALG_ASYNC |
						   CRYPTO_ALG_OPTIONAL_KEY |
						   CRYPTO_ALG_KERN_DRIVER_ONLY |
						   CRYPTO_ALG_ALLOCATES_MEMORY,
				.cra_blocksize	 = SHA512_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct dthe_tfm_ctx),
				.cra_module	 = THIS_MODULE,
			}
		},
	},
	{
		.init_tfm	= dthe_sha384_init_tfm,
		.init		= dthe_hash_init,
		.update		= dthe_hash_update,
		.final		= dthe_hash_final,
		.finup		= dthe_hash_finup,
		.digest		= dthe_hash_digest,
		.export		= dthe_sha512_export,
		.import		= dthe_sha384_import,
		.halg	= {
			.digestsize = SHA384_DIGEST_SIZE,
			.statesize = sizeof(struct sha512_export_state),
			.base = {
				.cra_name	 = "sha384",
				.cra_driver_name = "sha384-dthev2",
				.cra_priority	 = 400,
				.cra_flags	 = CRYPTO_ALG_TYPE_AHASH |
						   CRYPTO_ALG_ASYNC |
						   CRYPTO_ALG_OPTIONAL_KEY |
						   CRYPTO_ALG_KERN_DRIVER_ONLY |
						   CRYPTO_ALG_ALLOCATES_MEMORY,
				.cra_blocksize	 = SHA384_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct dthe_tfm_ctx),
				.cra_module	 = THIS_MODULE,
			}
		},
	},
	{
		.init_tfm	= dthe_sha256_init_tfm,
		.init		= dthe_hash_init,
		.update		= dthe_hash_update,
		.final		= dthe_hash_final,
		.finup		= dthe_hash_finup,
		.digest		= dthe_hash_digest,
		.export		= dthe_sha256_export,
		.import		= dthe_sha256_import,
		.halg	= {
			.digestsize = SHA256_DIGEST_SIZE,
			.statesize = sizeof(struct sha256_export_state),
			.base = {
				.cra_name	 = "sha256",
				.cra_driver_name = "sha256-dthev2",
				.cra_priority	 = 400,
				.cra_flags	 = CRYPTO_ALG_TYPE_AHASH |
						   CRYPTO_ALG_ASYNC |
						   CRYPTO_ALG_OPTIONAL_KEY |
						   CRYPTO_ALG_KERN_DRIVER_ONLY |
						   CRYPTO_ALG_ALLOCATES_MEMORY,
				.cra_blocksize	 = SHA256_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct dthe_tfm_ctx),
				.cra_module	 = THIS_MODULE,
			}
		},
	},
	{
		.init_tfm	= dthe_sha224_init_tfm,
		.init		= dthe_hash_init,
		.update		= dthe_hash_update,
		.final		= dthe_hash_final,
		.finup		= dthe_hash_finup,
		.digest		= dthe_hash_digest,
		.export		= dthe_sha256_export,
		.import		= dthe_sha224_import,
		.halg	= {
			.digestsize = SHA224_DIGEST_SIZE,
			.statesize = sizeof(struct sha256_export_state),
			.base = {
				.cra_name	 = "sha224",
				.cra_driver_name = "sha224-dthev2",
				.cra_priority	 = 400,
				.cra_flags	 = CRYPTO_ALG_TYPE_AHASH |
						   CRYPTO_ALG_ASYNC |
						   CRYPTO_ALG_OPTIONAL_KEY |
						   CRYPTO_ALG_KERN_DRIVER_ONLY |
						   CRYPTO_ALG_ALLOCATES_MEMORY,
				.cra_blocksize	 = SHA224_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct dthe_tfm_ctx),
				.cra_module	 = THIS_MODULE,
			}
		},
	},
	{
		.init_tfm	= dthe_md5_init_tfm,
		.init		= dthe_hash_init,
		.update		= dthe_hash_update,
		.final		= dthe_hash_final,
		.finup		= dthe_hash_finup,
		.digest		= dthe_hash_digest,
		.export		= dthe_md5_export,
		.import		= dthe_md5_import,
		.halg	= {
			.digestsize = MD5_DIGEST_SIZE,
			.statesize = sizeof(struct md5_export_state),
			.base = {
				.cra_name	 = "md5",
				.cra_driver_name = "md5-dthev2",
				.cra_priority	 = 400,
				.cra_flags	 = CRYPTO_ALG_TYPE_AHASH |
						   CRYPTO_ALG_ASYNC |
						   CRYPTO_ALG_OPTIONAL_KEY |
						   CRYPTO_ALG_KERN_DRIVER_ONLY |
						   CRYPTO_ALG_ALLOCATES_MEMORY,
				.cra_blocksize	 = MD5_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct dthe_tfm_ctx),
				.cra_module	 = THIS_MODULE,
			}
		},
	},
	{
		.init_tfm	= dthe_hmac_sha512_init_tfm,
		.exit_tfm	= dthe_hash_exit_tfm,
		.init		= dthe_hash_init,
		.update		= dthe_hash_update,
		.final		= dthe_hash_final,
		.finup		= dthe_hash_finup,
		.digest		= dthe_hash_digest,
		.export		= dthe_hmac_sha512_export,
		.import		= dthe_hmac_sha512_import,
		.setkey		= dthe_hmac_setkey,
		.halg	= {
			.digestsize = SHA512_DIGEST_SIZE,
			.statesize = sizeof(struct sha512_export_state),
			.base = {
				.cra_name	 = "hmac(sha512)",
				.cra_driver_name = "hmac-sha512-dthev2",
				.cra_priority	 = 400,
				.cra_flags	 = CRYPTO_ALG_TYPE_AHASH |
						   CRYPTO_ALG_ASYNC |
						   CRYPTO_ALG_NEED_FALLBACK |
						   CRYPTO_ALG_KERN_DRIVER_ONLY |
						   CRYPTO_ALG_ALLOCATES_MEMORY,
				.cra_blocksize	 = SHA512_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct dthe_tfm_ctx),
				.cra_module	 = THIS_MODULE,
			}
		},
	},
	{
		.init_tfm	= dthe_hmac_sha384_init_tfm,
		.exit_tfm	= dthe_hash_exit_tfm,
		.init		= dthe_hash_init,
		.update		= dthe_hash_update,
		.final		= dthe_hash_final,
		.finup		= dthe_hash_finup,
		.digest		= dthe_hash_digest,
		.export		= dthe_hmac_sha512_export,
		.import		= dthe_hmac_sha384_import,
		.setkey		= dthe_hmac_setkey,
		.halg	= {
			.digestsize = SHA384_DIGEST_SIZE,
			.statesize = sizeof(struct sha512_export_state),
			.base = {
				.cra_name	 = "hmac(sha384)",
				.cra_driver_name = "hmac-sha384-dthev2",
				.cra_priority	 = 400,
				.cra_flags	 = CRYPTO_ALG_TYPE_AHASH |
						   CRYPTO_ALG_ASYNC |
						   CRYPTO_ALG_NEED_FALLBACK |
						   CRYPTO_ALG_KERN_DRIVER_ONLY |
						   CRYPTO_ALG_ALLOCATES_MEMORY,
				.cra_blocksize	 = SHA384_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct dthe_tfm_ctx),
				.cra_module	 = THIS_MODULE,
			}
		},
	},
	{
		.init_tfm	= dthe_hmac_sha256_init_tfm,
		.exit_tfm	= dthe_hash_exit_tfm,
		.init		= dthe_hash_init,
		.update		= dthe_hash_update,
		.final		= dthe_hash_final,
		.finup		= dthe_hash_finup,
		.digest		= dthe_hash_digest,
		.export		= dthe_hmac_sha256_export,
		.import		= dthe_hmac_sha256_import,
		.setkey		= dthe_hmac_setkey,
		.halg	= {
			.digestsize = SHA256_DIGEST_SIZE,
			.statesize = sizeof(struct sha256_export_state),
			.base = {
				.cra_name	 = "hmac(sha256)",
				.cra_driver_name = "hmac-sha256-dthev2",
				.cra_priority	 = 400,
				.cra_flags	 = CRYPTO_ALG_TYPE_AHASH |
						   CRYPTO_ALG_ASYNC |
						   CRYPTO_ALG_NEED_FALLBACK |
						   CRYPTO_ALG_KERN_DRIVER_ONLY |
						   CRYPTO_ALG_ALLOCATES_MEMORY,
				.cra_blocksize	 = SHA256_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct dthe_tfm_ctx),
				.cra_module	 = THIS_MODULE,
			}
		},
	},
	{
		.init_tfm	= dthe_hmac_sha224_init_tfm,
		.exit_tfm	= dthe_hash_exit_tfm,
		.init		= dthe_hash_init,
		.update		= dthe_hash_update,
		.final		= dthe_hash_final,
		.finup		= dthe_hash_finup,
		.digest		= dthe_hash_digest,
		.export		= dthe_hmac_sha256_export,
		.import		= dthe_hmac_sha224_import,
		.setkey		= dthe_hmac_setkey,
		.halg	= {
			.digestsize = SHA224_DIGEST_SIZE,
			.statesize = sizeof(struct sha256_export_state),
			.base = {
				.cra_name	 = "hmac(sha224)",
				.cra_driver_name = "hmac-sha224-dthev2",
				.cra_priority	 = 400,
				.cra_flags	 = CRYPTO_ALG_TYPE_AHASH |
						   CRYPTO_ALG_ASYNC |
						   CRYPTO_ALG_NEED_FALLBACK |
						   CRYPTO_ALG_KERN_DRIVER_ONLY |
						   CRYPTO_ALG_ALLOCATES_MEMORY,
				.cra_blocksize	 = SHA224_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct dthe_tfm_ctx),
				.cra_module	 = THIS_MODULE,
			}
		},
	},
	{
		.init_tfm	= dthe_hmac_md5_init_tfm,
		.exit_tfm	= dthe_hash_exit_tfm,
		.init		= dthe_hash_init,
		.update		= dthe_hash_update,
		.final		= dthe_hash_final,
		.finup		= dthe_hash_finup,
		.digest		= dthe_hash_digest,
		.export		= dthe_hmac_md5_export,
		.import		= dthe_hmac_md5_import,
		.setkey		= dthe_hmac_setkey,
		.halg	= {
			.digestsize = MD5_DIGEST_SIZE,
			.statesize = sizeof(struct md5_export_state),
			.base = {
				.cra_name	 = "hmac(md5)",
				.cra_driver_name = "hmac-md5-dthev2",
				.cra_priority	 = 400,
				.cra_flags	 = CRYPTO_ALG_TYPE_AHASH |
						   CRYPTO_ALG_ASYNC |
						   CRYPTO_ALG_NEED_FALLBACK |
						   CRYPTO_ALG_KERN_DRIVER_ONLY |
						   CRYPTO_ALG_ALLOCATES_MEMORY,
				.cra_blocksize	 = MD5_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct dthe_tfm_ctx),
				.cra_module	 = THIS_MODULE,
			}
		},
	},
};

int dthe_register_hash_algs(void)
{
	return crypto_register_ahashes(hash_algs, ARRAY_SIZE(hash_algs));
}

void dthe_unregister_hash_algs(void)
{
	crypto_unregister_ahashes(hash_algs, ARRAY_SIZE(hash_algs));
}
