// SPDX-License-Identifier: GPL-2.0-only
/*
 * K3 DTHE V2 crypto accelerator driver
 *
 * Copyright (C) Texas Instruments 2025 - https://www.ti.com
 * Author: T Pratham <t-pratham@ti.com>
 */

#include <crypto/aead.h>
#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <crypto/engine.h>
#include <crypto/gcm.h>
#include <crypto/internal/aead.h>
#include <crypto/internal/skcipher.h>

#include "dthev2-common.h"

#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/scatterlist.h>

/* Registers */

// AES Engine
#define DTHE_P_AES_BASE		0x7000
#define DTHE_P_AES_KEY1_0	0x0038
#define DTHE_P_AES_KEY1_1	0x003C
#define DTHE_P_AES_KEY1_2	0x0030
#define DTHE_P_AES_KEY1_3	0x0034
#define DTHE_P_AES_KEY1_4	0x0028
#define DTHE_P_AES_KEY1_5	0x002C
#define DTHE_P_AES_KEY1_6	0x0020
#define DTHE_P_AES_KEY1_7	0x0024
#define DTHE_P_AES_IV_IN_0	0x0040
#define DTHE_P_AES_IV_IN_1	0x0044
#define DTHE_P_AES_IV_IN_2	0x0048
#define DTHE_P_AES_IV_IN_3	0x004C
#define DTHE_P_AES_CTRL		0x0050
#define DTHE_P_AES_C_LENGTH_0	0x0054
#define DTHE_P_AES_C_LENGTH_1	0x0058
#define DTHE_P_AES_AUTH_LENGTH	0x005C
#define DTHE_P_AES_DATA_IN_OUT	0x0060
#define DTHE_P_AES_TAG_OUT	0x0070

#define DTHE_P_AES_SYSCONFIG	0x0084
#define DTHE_P_AES_IRQSTATUS	0x008C
#define DTHE_P_AES_IRQENABLE	0x0090

/* Register write values and macros */

enum aes_ctrl_mode_masks {
	AES_CTRL_ECB_MASK = 0x00,
	AES_CTRL_CBC_MASK = BIT(5),
	AES_CTRL_GCM_MASK = BIT(17) | BIT(16) | BIT(6),
};

#define DTHE_AES_CTRL_MODE_CLEAR_MASK		~GENMASK(28, 5)

#define DTHE_AES_CTRL_DIR_ENC			BIT(2)

#define DTHE_AES_CTRL_KEYSIZE_16B		BIT(3)
#define DTHE_AES_CTRL_KEYSIZE_24B		BIT(4)
#define DTHE_AES_CTRL_KEYSIZE_32B		(BIT(3) | BIT(4))

#define DTHE_AES_CTRL_SAVE_CTX_SET		BIT(29)

#define DTHE_AES_CTRL_OUTPUT_READY		BIT_MASK(0)
#define DTHE_AES_CTRL_INPUT_READY		BIT_MASK(1)
#define DTHE_AES_CTRL_SAVED_CTX_READY		BIT_MASK(30)
#define DTHE_AES_CTRL_CTX_READY			BIT_MASK(31)

#define DTHE_AES_SYSCONFIG_DMA_DATA_IN_OUT_EN	GENMASK(6, 5)
#define DTHE_AES_IRQENABLE_EN_ALL		GENMASK(3, 0)

/* Misc */
#define AES_IV_SIZE				AES_BLOCK_SIZE
#define AES_BLOCK_WORDS				(AES_BLOCK_SIZE / sizeof(u32))
#define AES_IV_WORDS				AES_BLOCK_WORDS
#define POLL_TIMEOUT_INTERVAL			HZ

static int cnt;

static int dthe_poll_reg(struct dthe_data *dev_data, u32 reg, u32 bit)
{
	void __iomem *aes_base_reg = dev_data->regs + DTHE_P_AES_BASE;
	unsigned long timeout = jiffies + POLL_TIMEOUT_INTERVAL;

	while (!(readl_relaxed(aes_base_reg + reg) & bit)) {
		if (time_is_before_jiffies(timeout))
			return -ETIMEDOUT;
	}

	return 0;
}

static int dthe_cipher_init_tfm(struct crypto_skcipher *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_skcipher_ctx(tfm);
	struct dthe_data *dev_data = dthe_get_dev(ctx);

	crypto_skcipher_set_reqsize(tfm, sizeof(struct dthe_aes_req_ctx));

	ctx->dev_data = dev_data;
	ctx->keylen = 0;

	return 0;
}

static int dthe_aes_ecb_setkey(struct crypto_skcipher *tfm, const u8 *key, unsigned int keylen)
{
	struct dthe_tfm_ctx *ctx = crypto_skcipher_ctx(tfm);

	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 && keylen != AES_KEYSIZE_256)
		return -EINVAL;

	ctx->aes_mode = DTHE_AES_ECB;
	ctx->keylen = keylen;
	memcpy(ctx->key, key, keylen);

	return 0;
}

static int dthe_aes_cbc_setkey(struct crypto_skcipher *tfm, const u8 *key, unsigned int keylen)
{
	struct dthe_tfm_ctx *ctx = crypto_skcipher_ctx(tfm);

	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 && keylen != AES_KEYSIZE_256)
		return -EINVAL;

	ctx->aes_mode = DTHE_AES_CBC;
	ctx->keylen = keylen;
	memcpy(ctx->key, key, keylen);

	return 0;
}

static void dthe_aes_set_ctrl_key(struct dthe_tfm_ctx *ctx,
				  struct dthe_aes_req_ctx *rctx,
				  u32 *iv_in)
{
	struct dthe_data *dev_data = dthe_get_dev(ctx);
	void __iomem *aes_base_reg = dev_data->regs + DTHE_P_AES_BASE;
	u32 ctrl_val = 0;

	writel_relaxed(ctx->key[0], aes_base_reg + DTHE_P_AES_KEY1_0);
	writel_relaxed(ctx->key[1], aes_base_reg + DTHE_P_AES_KEY1_1);
	writel_relaxed(ctx->key[2], aes_base_reg + DTHE_P_AES_KEY1_2);
	writel_relaxed(ctx->key[3], aes_base_reg + DTHE_P_AES_KEY1_3);

	if (ctx->keylen > AES_KEYSIZE_128) {
		writel_relaxed(ctx->key[4], aes_base_reg + DTHE_P_AES_KEY1_4);
		writel_relaxed(ctx->key[5], aes_base_reg + DTHE_P_AES_KEY1_5);
	}
	if (ctx->keylen == AES_KEYSIZE_256) {
		writel_relaxed(ctx->key[6], aes_base_reg + DTHE_P_AES_KEY1_6);
		writel_relaxed(ctx->key[7], aes_base_reg + DTHE_P_AES_KEY1_7);
	}

	if (rctx->enc)
		ctrl_val |= DTHE_AES_CTRL_DIR_ENC;

	if (ctx->keylen == AES_KEYSIZE_128)
		ctrl_val |= DTHE_AES_CTRL_KEYSIZE_16B;
	else if (ctx->keylen == AES_KEYSIZE_192)
		ctrl_val |= DTHE_AES_CTRL_KEYSIZE_24B;
	else
		ctrl_val |= DTHE_AES_CTRL_KEYSIZE_32B;

	// Write AES mode
	ctrl_val &= DTHE_AES_CTRL_MODE_CLEAR_MASK;
	switch (ctx->aes_mode) {
	case DTHE_AES_ECB:
		ctrl_val |= AES_CTRL_ECB_MASK;
		break;
	case DTHE_AES_CBC:
		ctrl_val |= AES_CTRL_CBC_MASK;
		break;
	case DTHE_AES_GCM:
		ctrl_val |= AES_CTRL_GCM_MASK;
		break;
	}

	if (iv_in) {
		ctrl_val |= DTHE_AES_CTRL_SAVE_CTX_SET;
		for (int i = 0; i < AES_IV_WORDS; ++i)
			writel_relaxed(iv_in[i],
				       aes_base_reg + DTHE_P_AES_IV_IN_0 + (DTHE_REG_SIZE * i));
	}

	writel_relaxed(ctrl_val, aes_base_reg + DTHE_P_AES_CTRL);
}

static void dthe_aes_dma_in_callback(void *data)
{
	struct skcipher_request *req = (struct skcipher_request *)data;
	struct dthe_aes_req_ctx *rctx = skcipher_request_ctx(req);

	complete(&rctx->aes_compl);
}

static int dthe_aes_run(struct crypto_engine *engine, void *areq)
{
	struct skcipher_request *req = container_of(areq, struct skcipher_request, base);
	struct dthe_tfm_ctx *ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	struct dthe_data *dev_data = dthe_get_dev(ctx);
	struct dthe_aes_req_ctx *rctx = skcipher_request_ctx(req);

	unsigned int len = req->cryptlen;
	struct scatterlist *src = req->src;
	struct scatterlist *dst = req->dst;

	int src_nents = sg_nents_for_len(src, len);
	int dst_nents;

	int src_mapped_nents;
	int dst_mapped_nents;

	bool diff_dst;
	enum dma_data_direction src_dir, dst_dir;

	struct device *tx_dev, *rx_dev;
	struct dma_async_tx_descriptor *desc_in, *desc_out;

	int ret;

	void __iomem *aes_base_reg = dev_data->regs + DTHE_P_AES_BASE;

	u32 aes_sysconfig_val = readl_relaxed(aes_base_reg + DTHE_P_AES_SYSCONFIG);

	aes_sysconfig_val |= DTHE_AES_SYSCONFIG_DMA_DATA_IN_OUT_EN;
	writel_relaxed(aes_sysconfig_val, aes_base_reg + DTHE_P_AES_SYSCONFIG);

	if (src == dst) {
		diff_dst = false;
		src_dir = DMA_BIDIRECTIONAL;
		dst_dir = DMA_BIDIRECTIONAL;
	} else {
		diff_dst = true;
		src_dir = DMA_TO_DEVICE;
		dst_dir  = DMA_FROM_DEVICE;
	}

	tx_dev = dmaengine_get_dma_device(dev_data->dma_aes_tx);
	rx_dev = dmaengine_get_dma_device(dev_data->dma_aes_rx);

	src_mapped_nents = dma_map_sg(tx_dev, src, src_nents, src_dir);
	if (src_mapped_nents == 0) {
		ret = -EINVAL;
		goto aes_map_src_err;
	}

	if (!diff_dst) {
		dst_nents = src_nents;
		dst_mapped_nents = src_mapped_nents;
	} else {
		dst_nents = sg_nents_for_len(dst, len);
		dst_mapped_nents = dma_map_sg(rx_dev, dst, dst_nents, dst_dir);
		if (dst_mapped_nents == 0) {
			dma_unmap_sg(tx_dev, src, src_nents, src_dir);
			ret = -EINVAL;
			goto aes_map_dst_err;
		}
	}

	// HACK: Delay to workaround DMA driver issue
	cnt++;
	if (cnt % 20 == 0) {
		unsigned long delay = jiffies + usecs_to_jiffies(1000);

		while (time_before(jiffies, delay))
			cond_resched();
		cnt = 0;
	}

	desc_in = dmaengine_prep_slave_sg(dev_data->dma_aes_rx, dst, dst_mapped_nents,
					  DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc_in) {
		dev_err(dev_data->dev, "IN prep_slave_sg() failed\n");
		ret = -EINVAL;
		goto aes_prep_err;
	}

	desc_out = dmaengine_prep_slave_sg(dev_data->dma_aes_tx, src, src_mapped_nents,
					   DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc_out) {
		dev_err(dev_data->dev, "OUT prep_slave_sg() failed\n");
		ret = -EINVAL;
		goto aes_prep_err;
	}

	desc_in->callback = dthe_aes_dma_in_callback;
	desc_in->callback_param = req;

	init_completion(&rctx->aes_compl);

	if (ctx->aes_mode == DTHE_AES_ECB)
		dthe_aes_set_ctrl_key(ctx, rctx, NULL);
	else
		dthe_aes_set_ctrl_key(ctx, rctx, (u32 *)req->iv);

	writel_relaxed(lower_32_bits(req->cryptlen), aes_base_reg + DTHE_P_AES_C_LENGTH_0);
	writel_relaxed(upper_32_bits(req->cryptlen), aes_base_reg + DTHE_P_AES_C_LENGTH_1);

	dmaengine_submit(desc_in);
	dmaengine_submit(desc_out);

	dma_async_issue_pending(dev_data->dma_aes_rx);
	dma_async_issue_pending(dev_data->dma_aes_tx);

	// Need to do a timeout to ensure finalise gets called if DMA callback fails for any reason
	ret = wait_for_completion_timeout(&rctx->aes_compl, msecs_to_jiffies(DTHE_DMA_TIMEOUT_MS));
	if (!ret) {
		ret = -ETIMEDOUT;
		dmaengine_terminate_sync(dev_data->dma_aes_rx);
		dmaengine_terminate_sync(dev_data->dma_aes_tx);

		for (int i = 0; i < AES_BLOCK_WORDS; ++i)
			readl_relaxed(aes_base_reg + DTHE_P_AES_DATA_IN_OUT + (DTHE_REG_SIZE * i));
	} else {
		ret = 0;
	}

	// For modes other than ECB, read IV_OUT
	if (ctx->aes_mode != DTHE_AES_ECB) {
		u32 *iv_out = (u32 *)req->iv;

		for (int i = 0; i < AES_IV_WORDS; ++i)
			iv_out[i] = readl_relaxed(aes_base_reg +
						  DTHE_P_AES_IV_IN_0 +
						  (DTHE_REG_SIZE * i));
	}

aes_prep_err:
	if (dst_dir != DMA_BIDIRECTIONAL)
		dma_unmap_sg(rx_dev, dst, dst_nents, dst_dir);
aes_map_dst_err:
	dma_unmap_sg(tx_dev, src, src_nents, src_dir);

aes_map_src_err:
	local_bh_disable();
	crypto_finalize_skcipher_request(dev_data->aes_engine, req, ret);
	local_bh_enable();
	return 0;
}

static int dthe_aes_crypt(struct skcipher_request *req)
{
	struct dthe_tfm_ctx *ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	struct dthe_data *dev_data = dthe_get_dev(ctx);
	struct crypto_engine *engine;

	/*
	 * If data is not a multiple of AES_BLOCK_SIZE, need to return -EINVAL
	 * If data length input is zero, no need to do any operation.
	 */
	if (req->cryptlen % AES_BLOCK_SIZE)
		return -EINVAL;

	if (req->cryptlen == 0)
		return 0;

	engine = dev_data->aes_engine;
	return crypto_transfer_skcipher_request_to_engine(engine, req);
}

static int dthe_aes_encrypt(struct skcipher_request *req)
{
	struct dthe_aes_req_ctx *rctx = skcipher_request_ctx(req);

	rctx->enc = 1;
	return dthe_aes_crypt(req);
}

static int dthe_aes_decrypt(struct skcipher_request *req)
{
	struct dthe_aes_req_ctx *rctx = skcipher_request_ctx(req);

	rctx->enc = 0;
	return dthe_aes_crypt(req);
}

static int dthe_aead_init_tfm(struct crypto_aead *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_aead_ctx(tfm);
	struct dthe_data *dev_data = dthe_get_dev(ctx);

	memzero_explicit(ctx, sizeof(*ctx));
	ctx->dev_data = dev_data;

	const char *alg_name = crypto_tfm_alg_name(&tfm->base);

	ctx->aead_fb = crypto_alloc_aead(alg_name, 0,
					 CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(ctx->aead_fb)) {
		dev_err(dev_data->dev, "fallback driver %s couldn't be loaded\n",
			alg_name);
		return PTR_ERR(ctx->aead_fb);
	}

	crypto_aead_set_reqsize(tfm, sizeof(struct dthe_aes_req_ctx));

	return 0;
}

static void dthe_aead_exit_tfm(struct crypto_aead *tfm)
{
	struct dthe_tfm_ctx *ctx = crypto_aead_ctx(tfm);

	crypto_free_aead(ctx->aead_fb);
}

/**
 * dthe_aead_prep_src - Prepare source scatterlist for AEAD from input req->src
 * @sg: Input req->src scatterlist
 * @assoclen: Input req->assoclen
 * @cryptlen: Input req->cryptlen (minus size of TAG for AES-GCM decryption)
 *
 * Description:
 *   For modes with authentication, DTHEv2 hardware requires the input AAD and
 *   plaintext/ciphertext to be individually aligned to AES_BLOCK_SIZE. However,
 *   linux crypto's aead_request provides the input with AAD and plaintext/ciphertext
 *   contiguously in a single scatterlist.
 *
 *   This helper function takes the input scatterlist and splits it into separate
 *   scatterlists for AAD and plaintext/ciphertext, ensuring each is aligned to
 *   AES_BLOCK_SIZE, and then merges the aligned scatterlists back into a single
 *   scatterlist for processing.
 *
 * Return:
 *   Pointer to the merged scatterlist, or NULL on failure.
 **/
static struct scatterlist *dthe_aead_prep_src(struct scatterlist *sg,
					      unsigned int assoclen,
					      unsigned int cryptlen)
{
	struct scatterlist *in_sg[2];
	struct scatterlist *to_sg;
	struct scatterlist *src, *ret;
	size_t split_sizes[2] = {assoclen, cryptlen};
	int out_mapped_nents[2];
	int crypt_nents = 0, assoc_nents = 0, src_nents = 0;
	u8 *pad_buf_1, *pad_buf_2;

	/* sg_split does not work properly if one of the split_sizes is 0 */
	if (cryptlen == 0 || assoclen == 0) {
		/*
		 * Assigning both to sg does not matter as assoclen = 0 or cryptlen = 0
		 * being passed to dthe_copy_sg will take care to copy the sg correctly
		 */
		in_sg[0] = sg;
		in_sg[1] = sg;

		src_nents = sg_nents_for_len(sg, assoclen + cryptlen);
	} else {
		sg_split(sg, 0, 0, 2, split_sizes, in_sg, out_mapped_nents, GFP_KERNEL);
		assoc_nents = sg_nents_for_len(in_sg[0], assoclen);
		crypt_nents = sg_nents_for_len(in_sg[1], cryptlen);

		src_nents = assoc_nents + crypt_nents;
	}

	if (assoclen % AES_BLOCK_SIZE)
		src_nents++;
	if (cryptlen % AES_BLOCK_SIZE)
		src_nents++;

	src = kmalloc_array(src_nents, sizeof(struct scatterlist), GFP_KERNEL);
	if (!src) {
		ret = NULL;
		goto dthe_aead_prep_src_mem_err;
	}

	sg_init_table(src, src_nents);
	to_sg = src;
	ret = src;

	to_sg = dthe_copy_sg(to_sg, in_sg[0], assoclen);
	if (assoclen % AES_BLOCK_SIZE) {
		unsigned int pad_len = AES_BLOCK_SIZE - (assoclen % AES_BLOCK_SIZE);

		pad_buf_1 = kcalloc(pad_len, sizeof(u8), GFP_KERNEL);
		if (!pad_buf_1) {
			kfree(src);
			ret = NULL;
			goto dthe_aead_prep_src_mem_err;
		}

		sg_set_buf(to_sg, pad_buf_1, pad_len);
		to_sg = sg_next(to_sg);
	}

	to_sg = dthe_copy_sg(to_sg, in_sg[1], cryptlen);
	if (cryptlen % AES_BLOCK_SIZE) {
		unsigned int pad_len = AES_BLOCK_SIZE - (cryptlen % AES_BLOCK_SIZE);

		pad_buf_2 = kcalloc(pad_len, sizeof(u8), GFP_KERNEL);
		if (!pad_buf_2) {
			if (assoclen % AES_BLOCK_SIZE)
				kfree(pad_buf_1);
			kfree(src);
			ret = NULL;
			goto dthe_aead_prep_src_mem_err;
		}

		sg_set_buf(to_sg, pad_buf_2, pad_len);
		to_sg = sg_next(to_sg);
	}

dthe_aead_prep_src_mem_err:
	if (cryptlen != 0 && assoclen != 0) {
		kfree(in_sg[0]);
		kfree(in_sg[1]);
	}

	return ret;
}

/**
 * dthe_aead_prep_dst - Prepare destination scatterlist for AEAD from input req->dst
 * @sg:	Input req->dst scatterlist
 * @assoclen: Input req->assoclen
 * @cryptlen: Input req->cryptlen (minus size of TAG for AES-GCM decryption)
 *
 * Description:
 *   For modes with authentication, DTHEv2 hardware returns encrypted ciphertext/decrypted
 *   plaintext through DMA and TAG through MMRs. However, the dst scatterlist in linux
 *   crypto's aead_request is allocated same as input req->src scatterlist. That is, it
 *   contains space for AAD in the beginning and ciphertext/plaintext at the end, with no
 *   alignment padding. This causes issues with DMA engine and DTHEv2 hardware.
 *
 *   This helper function takes the output scatterlist and maps the part of the buffer
 *   which holds only the ciphertext/plaintext to a new scatterlist. It also adds a padding
 *   to align it with AES_BLOCK_SIZE.
 *
 * Return:
 *   Pointer to the trimmed scatterlist, or NULL on failure.
 **/
static struct scatterlist *dthe_aead_prep_dst(struct scatterlist *sg,
					      unsigned int assoclen,
					      unsigned int cryptlen)
{
	struct scatterlist *out_sg[1];
	struct scatterlist *dst;
	struct scatterlist *to_sg;
	size_t split_sizes[1] = {cryptlen};
	int out_mapped_nents[1];
	int dst_nents = 0;

	sg_split(sg, 0, assoclen, 1, split_sizes, out_sg, out_mapped_nents, GFP_KERNEL);
	dst_nents = sg_nents_for_len(out_sg[0], cryptlen);
	if (cryptlen % AES_BLOCK_SIZE)
		dst_nents++;

	dst = kmalloc_array(dst_nents, sizeof(struct scatterlist), GFP_KERNEL);
	if (!dst) {
		kfree(out_sg[0]);
		return NULL;
	}
	sg_init_table(dst, dst_nents);

	to_sg = dthe_copy_sg(dst, out_sg[0], cryptlen);
	if (cryptlen % AES_BLOCK_SIZE) {
		unsigned int pad_len = AES_BLOCK_SIZE - (cryptlen % AES_BLOCK_SIZE);
		u8 *pad_buf = kzalloc(sizeof(u8) * pad_len, GFP_KERNEL);

		if (!pad_buf) {
			kfree(dst);
			kfree(out_sg[0]);
			return NULL;
		}

		sg_set_buf(to_sg, pad_buf, pad_len);
		to_sg = sg_next(to_sg);
	}

	kfree(out_sg[0]);

	return dst;
}

static int dthe_aead_read_tag(struct dthe_tfm_ctx *ctx, u32 *tag)
{
	struct dthe_data *dev_data = dthe_get_dev(ctx);
	void __iomem *aes_base_reg = dev_data->regs + DTHE_P_AES_BASE;
	int ret;

	ret = dthe_poll_reg(dev_data, DTHE_P_AES_CTRL, DTHE_AES_CTRL_SAVED_CTX_READY);
	if (ret)
		return ret;

	for (int i = 0; i < AES_BLOCK_WORDS; ++i)
		tag[i] = readl_relaxed(aes_base_reg +
				       DTHE_P_AES_TAG_OUT +
				       DTHE_REG_SIZE * i);
	return 0;
}

static int dthe_gcm_aes_enc_get_tag(struct aead_request *req)
{
	struct dthe_tfm_ctx *ctx = crypto_aead_ctx(crypto_aead_reqtfm(req));
	u32 tag[AES_BLOCK_WORDS];
	int nents;
	int ret;

	ret = dthe_aead_read_tag(ctx, tag);
	if (ret)
		return ret;

	nents = sg_nents_for_len(req->dst, req->cryptlen + req->assoclen + ctx->authsize);

	sg_pcopy_from_buffer(req->dst, nents, tag, ctx->authsize,
			     req->assoclen + req->cryptlen);

	return 0;
}

static int dthe_gcm_aes_dec_verify_tag(struct aead_request *req)
{
	struct dthe_tfm_ctx *ctx = crypto_aead_ctx(crypto_aead_reqtfm(req));
	u32 tag_out[AES_BLOCK_WORDS];
	u32 tag_in[AES_BLOCK_WORDS];
	int nents;
	int ret;

	ret = dthe_aead_read_tag(ctx, tag_out);
	if (ret)
		return ret;

	nents = sg_nents_for_len(req->src, req->assoclen + req->cryptlen);

	sg_pcopy_to_buffer(req->src, nents, tag_in, ctx->authsize,
			   req->assoclen + req->cryptlen - ctx->authsize);

	if (memcmp(tag_in, tag_out, ctx->authsize))
		return -EBADMSG;
	else
		return 0;
}

static int dthe_aead_setkey(struct crypto_aead *tfm, const u8 *key, unsigned int keylen)
{
	struct dthe_tfm_ctx *ctx = crypto_aead_ctx(tfm);

	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 && keylen != AES_KEYSIZE_256)
		return -EINVAL;

	ctx->aes_mode = DTHE_AES_GCM;
	ctx->keylen = keylen;
	memcpy(ctx->key, key, keylen);

	crypto_aead_clear_flags(ctx->aead_fb, CRYPTO_TFM_REQ_MASK);
	crypto_aead_set_flags(ctx->aead_fb,
			      crypto_aead_get_flags(tfm) &
			      CRYPTO_TFM_REQ_MASK);

	return crypto_aead_setkey(ctx->aead_fb, key, keylen);
}

static int dthe_aead_setauthsize(struct crypto_aead *tfm, unsigned int authsize)
{
	struct dthe_tfm_ctx *ctx = crypto_aead_ctx(tfm);

	/* Invalid auth size will be handled by crypto_aead_setauthsize() */
	ctx->authsize = authsize;

	return crypto_aead_setauthsize(ctx->aead_fb, authsize);
}

static void dthe_aead_dma_in_callback(void *data)
{
	struct aead_request *req = (struct aead_request *)data;
	struct dthe_aes_req_ctx *rctx = aead_request_ctx(req);

	complete(&rctx->aes_compl);
}

static int dthe_aead_run(struct crypto_engine *engine, void *areq)
{
	struct aead_request *req = container_of(areq, struct aead_request, base);
	struct dthe_tfm_ctx *ctx = crypto_aead_ctx(crypto_aead_reqtfm(req));
	struct dthe_aes_req_ctx *rctx = aead_request_ctx(req);
	struct dthe_data *dev_data = dthe_get_dev(ctx);

	unsigned int cryptlen = req->cryptlen;
	unsigned int assoclen = req->assoclen;
	unsigned int authsize = ctx->authsize;
	unsigned int unpadded_cryptlen;
	struct scatterlist *src = req->src;
	struct scatterlist *dst = req->dst;

	int src_nents;
	int dst_nents;
	int src_mapped_nents, dst_mapped_nents;

	enum dma_data_direction src_dir, dst_dir;

	struct device *tx_dev, *rx_dev;
	struct dma_async_tx_descriptor *desc_in, *desc_out;

	int ret;

	void __iomem *aes_base_reg = dev_data->regs + DTHE_P_AES_BASE;

	u32 aes_sysconfig_val = readl_relaxed(aes_base_reg + DTHE_P_AES_SYSCONFIG);

	aes_sysconfig_val |= DTHE_AES_SYSCONFIG_DMA_DATA_IN_OUT_EN;
	writel_relaxed(aes_sysconfig_val, aes_base_reg + DTHE_P_AES_SYSCONFIG);

	/* In decryption, the last authsize bytes are the TAG */
	if (!rctx->enc)
		cryptlen -= authsize;
	unpadded_cryptlen = cryptlen;

	// Prep src and dst scatterlists
	src = dthe_aead_prep_src(req->src, req->assoclen, cryptlen);
	if (!src) {
		ret = -ENOMEM;
		goto aead_err;
	}

	if (req->assoclen % AES_BLOCK_SIZE)
		assoclen += AES_BLOCK_SIZE - (req->assoclen % AES_BLOCK_SIZE);
	if (cryptlen % AES_BLOCK_SIZE)
		cryptlen += AES_BLOCK_SIZE - (cryptlen % AES_BLOCK_SIZE);

	src_nents = sg_nents_for_len(src, assoclen + cryptlen);

	if (cryptlen != 0) {
		dst = dthe_aead_prep_dst(req->dst, req->assoclen, unpadded_cryptlen);
		if (!dst) {
			ret = -ENOMEM;
			goto aead_prep_dst_err;
		}

		dst_nents = sg_nents_for_len(dst, cryptlen);
	}
	// Prep finished

	src_dir = DMA_TO_DEVICE;
	dst_dir = DMA_FROM_DEVICE;

	tx_dev = dmaengine_get_dma_device(dev_data->dma_aes_tx);
	rx_dev = dmaengine_get_dma_device(dev_data->dma_aes_rx);

	// HACK: Delay to workaround DMA driver issue
	cnt++;
	if (cnt % 20 == 0) {
		unsigned long delay = jiffies + usecs_to_jiffies(1000);

		while (time_before(jiffies, delay))
			cond_resched();
		cnt = 0;
	}

	src_mapped_nents = dma_map_sg(tx_dev, src, src_nents, src_dir);
	if (src_mapped_nents == 0) {
		ret = -EINVAL;
		goto aead_dma_map_src_err;
	}

	desc_out = dmaengine_prep_slave_sg(dev_data->dma_aes_tx, src, src_mapped_nents,
					   DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc_out) {
		ret = -EINVAL;
		goto aead_dma_prep_src_err;
	}

	desc_out->callback = dthe_aead_dma_in_callback;
	desc_out->callback_param = req;

	if (cryptlen != 0) {
		dst_mapped_nents = dma_map_sg(rx_dev, dst, dst_nents, dst_dir);
		if (dst_mapped_nents == 0) {
			ret = -EINVAL;
			goto aead_dma_prep_src_err;
		}

		desc_in = dmaengine_prep_slave_sg(dev_data->dma_aes_rx, dst,
						  dst_mapped_nents, DMA_DEV_TO_MEM,
						  DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!desc_in) {
			ret = -EINVAL;
			goto aead_dma_prep_dst_err;
		}
	}

	init_completion(&rctx->aes_compl);

	/*
	 * HACK: There is an unknown hw issue where if the previous operation had alen = 0 and
	 * plen != 0, the current operation's tag calculation is incorrect in the case where
	 * plen = 0 and alen != 0 currently. This is a workaround for now which somwhow works;
	 * by resetting the context by writing a 1 to the C_LENGTH_0 and AUTH_LENGTH registers.
	 */
	if (cryptlen == 0) {
		writel_relaxed(1, aes_base_reg + DTHE_P_AES_C_LENGTH_0);
		writel_relaxed(1, aes_base_reg + DTHE_P_AES_AUTH_LENGTH);
	}

	u32 iv_in[AES_BLOCK_SIZE / sizeof(u32)];

	if (req->iv) {
		memcpy(iv_in, req->iv, GCM_AES_IV_SIZE);
	} else {
		iv_in[0] = 0;
		iv_in[1] = 0;
		iv_in[2] = 0;
	}
	iv_in[3] = 0x01000000;

	// Clear key2 to reset previous GHASH intermediate data
	for (int i = 0; i < AES_KEYSIZE_256 / sizeof(u32); ++i)
		writel_relaxed(0, aes_base_reg + DTHE_REG_SIZE * i);

	dthe_aes_set_ctrl_key(ctx, rctx, iv_in);

	writel_relaxed(lower_32_bits(unpadded_cryptlen), aes_base_reg + DTHE_P_AES_C_LENGTH_0);
	writel_relaxed(upper_32_bits(unpadded_cryptlen), aes_base_reg + DTHE_P_AES_C_LENGTH_1);
	writel_relaxed(req->assoclen, aes_base_reg + DTHE_P_AES_AUTH_LENGTH);

	if (cryptlen != 0)
		dmaengine_submit(desc_in);
	dmaengine_submit(desc_out);

	if (cryptlen != 0)
		dma_async_issue_pending(dev_data->dma_aes_rx);
	dma_async_issue_pending(dev_data->dma_aes_tx);

	// Need to do a timeout to ensure mutex gets unlocked if DMA callback fails for any reason
	ret = wait_for_completion_timeout(&rctx->aes_compl, msecs_to_jiffies(DTHE_DMA_TIMEOUT_MS));
	if (!ret) {
		ret = -ETIMEDOUT;
		if (cryptlen != 0)
			dmaengine_terminate_sync(dev_data->dma_aes_rx);
		dmaengine_terminate_sync(dev_data->dma_aes_tx);

		for (int i = 0; i < AES_BLOCK_SIZE / sizeof(int); ++i)
			readl_relaxed(aes_base_reg + DTHE_P_AES_DATA_IN_OUT + DTHE_REG_SIZE * i);
	} else {
		ret = 0;
	}

	if (cryptlen != 0)
		dma_sync_sg_for_cpu(rx_dev, dst, dst_nents, dst_dir);
	if (rctx->enc)
		ret = dthe_gcm_aes_enc_get_tag(req);
	else
		ret = dthe_gcm_aes_dec_verify_tag(req);

aead_dma_prep_dst_err:
	if (cryptlen != 0)
		dma_unmap_sg(rx_dev, dst, dst_nents, dst_dir);
aead_dma_prep_src_err:
	dma_unmap_sg(tx_dev, src, src_nents, src_dir);

aead_dma_map_src_err:
	if (unpadded_cryptlen % AES_BLOCK_SIZE && cryptlen != 0)
		kfree(sg_virt(&dst[dst_nents - 1]));

	if (cryptlen != 0)
		kfree(dst);

aead_prep_dst_err:
	if (req->assoclen % AES_BLOCK_SIZE) {
		int assoc_nents = sg_nents_for_len(src, req->assoclen);

		kfree(sg_virt(&src[assoc_nents]));
	}
	if (unpadded_cryptlen % AES_BLOCK_SIZE)
		kfree(sg_virt(&src[src_nents - 1]));

	kfree(src);

aead_err:
	local_bh_disable();
	crypto_finalize_aead_request(dev_data->aes_engine, req, ret);
	local_bh_enable();
	return 0;
}

static int dthe_aead_crypt(struct aead_request *req)
{
	struct dthe_tfm_ctx *ctx = crypto_aead_ctx(crypto_aead_reqtfm(req));
	struct dthe_aes_req_ctx *rctx = aead_request_ctx(req);
	struct dthe_data *dev_data = dthe_get_dev(ctx);
	struct crypto_engine *engine;
	unsigned int cryptlen = req->cryptlen;

	/* In decryption, last authsize bytes are the TAG */
	if (!rctx->enc)
		cryptlen -= ctx->authsize;

	/*
	 * Need to fallback to software in the following cases due to HW restrictions:
	 * - Both AAD and plaintext/ciphertext are zero length
	 * - AAD length is more than 2^32 - 1 bytes
	 *
	 * PS: req->cryptlen is currently unsigned int type, which causes the second
	 * case above tautologically false. If req->cryptlen is to be changed to a 64-bit
	 * type, the check for these would also need to be added below.
	 */
	if (req->assoclen == 0 && cryptlen == 0) {
		struct aead_request *subreq = &rctx->fb_req;
		int ret;

		aead_request_set_tfm(subreq, ctx->aead_fb);
		aead_request_set_callback(subreq, req->base.flags,
					  req->base.complete, req->base.data);
		aead_request_set_crypt(subreq, req->src, req->dst,
				       req->cryptlen, req->iv);
		aead_request_set_ad(subreq, req->assoclen);

		ret = rctx->enc ? crypto_aead_encrypt(subreq) :
			crypto_aead_decrypt(subreq);

		return ret;
	}

	engine = dev_data->aes_engine;
	return crypto_transfer_aead_request_to_engine(engine, req);
}

static int dthe_aead_encrypt(struct aead_request *req)
{
	struct dthe_aes_req_ctx *rctx = aead_request_ctx(req);

	rctx->enc = 1;
	return dthe_aead_crypt(req);
}

static int dthe_aead_decrypt(struct aead_request *req)
{
	struct dthe_aes_req_ctx *rctx = aead_request_ctx(req);

	rctx->enc = 0;
	return dthe_aead_crypt(req);
}

static struct skcipher_engine_alg cipher_algs[] = {
	{
		.base.init			= dthe_cipher_init_tfm,
		.base.setkey			= dthe_aes_ecb_setkey,
		.base.encrypt			= dthe_aes_encrypt,
		.base.decrypt			= dthe_aes_decrypt,
		.base.min_keysize		= AES_MIN_KEY_SIZE,
		.base.max_keysize		= AES_MAX_KEY_SIZE,
		.base.base = {
			.cra_name		= "ecb(aes)",
			.cra_driver_name	= "ecb-aes-dthev2",
			.cra_priority		= 30000,
			.cra_flags		= CRYPTO_ALG_TYPE_SKCIPHER |
						  CRYPTO_ALG_KERN_DRIVER_ONLY,
			.cra_alignmask		= AES_BLOCK_SIZE - 1,
			.cra_blocksize		= AES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct dthe_tfm_ctx),
			.cra_module		= THIS_MODULE,
		},
		.op.do_one_request = dthe_aes_run,
	}, /* ECB AES */
	{
		.base.init			= dthe_cipher_init_tfm,
		.base.setkey			= dthe_aes_cbc_setkey,
		.base.encrypt			= dthe_aes_encrypt,
		.base.decrypt			= dthe_aes_decrypt,
		.base.min_keysize		= AES_MIN_KEY_SIZE,
		.base.max_keysize		= AES_MAX_KEY_SIZE,
		.base.ivsize			= AES_IV_SIZE,
		.base.base = {
			.cra_name		= "cbc(aes)",
			.cra_driver_name	= "cbc-aes-dthev2",
			.cra_priority		= 30000,
			.cra_flags		= CRYPTO_ALG_TYPE_SKCIPHER |
						  CRYPTO_ALG_KERN_DRIVER_ONLY,
			.cra_alignmask		= AES_BLOCK_SIZE - 1,
			.cra_blocksize		= AES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct dthe_tfm_ctx),
			.cra_module		= THIS_MODULE,
		},
		.op.do_one_request = dthe_aes_run,
	} /* CBC AES */
};

static struct aead_engine_alg aead_algs[] = {
	{
		.base.init			= dthe_aead_init_tfm,
		.base.exit			= dthe_aead_exit_tfm,
		.base.setkey			= dthe_aead_setkey,
		.base.setauthsize		= dthe_aead_setauthsize,
		.base.maxauthsize		= AES_BLOCK_SIZE,
		.base.encrypt			= dthe_aead_encrypt,
		.base.decrypt			= dthe_aead_decrypt,
		.base.chunksize			= AES_BLOCK_SIZE,
		.base.ivsize			= GCM_AES_IV_SIZE,
		.base.base = {
			.cra_name		= "gcm(aes)",
			.cra_driver_name	= "gcm-aes-dthev2",
			.cra_priority		= 3000,
			.cra_flags		= CRYPTO_ALG_TYPE_AEAD |
						  CRYPTO_ALG_KERN_DRIVER_ONLY |
						  CRYPTO_ALG_ASYNC |
						  CRYPTO_ALG_NEED_FALLBACK,
			.cra_blocksize		= 1,
			.cra_module		= THIS_MODULE,
			.cra_ctxsize		= sizeof(struct dthe_tfm_ctx),
		},
		.op.do_one_request = dthe_aead_run,
	} /* GCM AES */
};

int dthe_register_aes_algs(void)
{
	int ret = 0;

	ret |= crypto_engine_register_skciphers(cipher_algs, ARRAY_SIZE(cipher_algs));
	ret |= crypto_engine_register_aeads(aead_algs, ARRAY_SIZE(aead_algs));

	return ret;
}

void dthe_unregister_aes_algs(void)
{
	crypto_engine_unregister_skciphers(cipher_algs, ARRAY_SIZE(cipher_algs));
	crypto_engine_unregister_aeads(aead_algs, ARRAY_SIZE(aead_algs));
}
