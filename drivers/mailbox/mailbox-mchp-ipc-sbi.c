// SPDX-License-Identifier: GPL-2.0
/*
 * Microchip Inter-Processor communication (IPC) driver
 *
 * Copyright (c) 2021 - 2024 Microchip Technology Inc. All rights reserved.
 *
 * Author: Valentina Fernandez <valentina.fernandezalanis@microchip.com>
 *
 */

#include <linux/io.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/mailbox/mchp-ipc.h>
#include <asm/sbi.h>
#include <asm/smp.h>
#include <asm/vendorid_list.h>

#define IRQ_STATUS_BITS			12
#define NUM_CHANS_PER_CLUSTER		5
#define IPC_DMA_BIT_MASK		32
#define SBI_EXT_MICROCHIP_TECHNOLOGY	(SBI_EXT_VENDOR_START | \
					 MICROCHIP_VENDOR_ID)

enum {
	SBI_EXT_IPC_PROBE = 0x100,
	SBI_EXT_IPC_CH_INIT,
	SBI_EXT_IPC_SEND,
	SBI_EXT_IPC_RECEIVE,
	SBI_EXT_IPC_STATUS,
};

enum ipc_hw {
	MIV_IHC,
	RESERVED1,
	RESERVED2,
};

enum ipc_irq_type {
	IPC_OPS_NOT_SUPPORTED	= 1,
	IPC_MP_IRQ		= 2,
	IPC_MC_IRQ		= 4,
};

/**
 * struct mchp_ipc_probe - IPC probe message format
 *
 * @hw_type:		IPC implementation available in the hardware
 * @num_channels:	number of IPC channels available in the hardware
 *
 * Used to retrieve information on the IPC implementation
 * using the SBI_EXT_IPC_PROBE SBI function id.
 */
struct mchp_ipc_probe {
	enum ipc_hw hw_type;
	u8 num_channels;
};

/**
 * struct mchp_ipc_init - IPC channel init message format
 *
 * @max_msg_size:	maxmimum message size in bytes of a given channel
 *
 * struct used by the SBI_EXT_IPC_CH_INIT SBI function id to get
 * the max message size in bytes of the initialized channel.
 */
struct mchp_ipc_init {
	u16 max_msg_size;
};

/**
 * struct mchp_ipc_status - IPC status message format
 *
 * @status:	interrupt status for all channels associated to a cluster
 * @cluster:	specifies the cluster instance that originated an irq
 *
 * struct used by the SBI_EXT_IPC_STATUS SBI function id to get
 * the message present and message clear interrupt status for all the
 * channels associated to a cluster.
 */
struct mchp_ipc_status {
	u32 status;
	u8 cluster;
};

/**
 * struct mchp_ipc_sbi_msg - IPC SBI payload message
 *
 * @buf_addr:	physical address where the received data should be copied to
 * @size:	maximum size(in bytes) that can be stored in the buffer pointed to by `buf`
 * @irq_type:	mask representing the irq types that triggered an irq
 *
 * struct used by the SBI_EXT_IPC_SEND/SBI_EXT_IPC_RECEIVE SBI function
 * ids to send/receive a message from an associated processor using
 * the IPC.
 */
struct mchp_ipc_sbi_msg {
	u64 buf_addr;
	u16 size;
	u8 irq_type;
};

struct mchp_ipc_cluster_cfg {
	void *buf_base;
	dma_addr_t dma_addr;
	int irq;
};

struct ipc_chan_info {
	void *buf_base_tx;
	void *buf_base_rx;
	void *msg_buf_tx;
	void *msg_buf_rx;
	dma_addr_t dma_addr_tx;
	dma_addr_t dma_addr_rx;
	dma_addr_t msg_buf_dma_tx;
	dma_addr_t msg_buf_dma_rx;
	int chan_aggregated_irq;
	int mp_irq;
	int mc_irq;
	u32 id;
	u32 max_msg_size;
};

struct microchip_ipc {
	struct device *dev;
	struct mbox_chan *chans;
	struct mchp_ipc_cluster_cfg *cluster_cfg;
	struct ipc_chan_info *priv;
	void *buf_base;
	dma_addr_t dma_addr;
	struct mbox_controller controller;
	u8 num_channels;
	enum ipc_hw hw_type;
};

static int mchp_ipc_sbi_chan_send(u32 command, u32 channel, dma_addr_t address)
{
	struct sbiret ret;

	ret = sbi_ecall(SBI_EXT_MICROCHIP_TECHNOLOGY, command, channel,
			address, 0, 0, 0, 0);

	if (ret.error)
		return sbi_err_map_linux_errno(ret.error);
	else
		return ret.value;
}

static int mchp_ipc_sbi_send(u32 command, dma_addr_t address)
{
	struct sbiret ret;

	ret = sbi_ecall(SBI_EXT_MICROCHIP_TECHNOLOGY, command, address,
			0, 0, 0, 0, 0);

	if (ret.error)
		return sbi_err_map_linux_errno(ret.error);
	else
		return ret.value;
}

static struct microchip_ipc *to_mchp_ipc_mbox(struct mbox_controller *mbox)
{
	return container_of(mbox, struct microchip_ipc, controller);
}

u32 mchp_ipc_get_chan_id(struct mbox_chan *chan)
{
	struct ipc_chan_info *chan_info = (struct ipc_chan_info *)chan->con_priv;

	return chan_info->id;
}
EXPORT_SYMBOL(mchp_ipc_get_chan_id);

static inline void mchp_ipc_prepare_receive_req(struct mbox_chan *chan)
{
	struct ipc_chan_info *chan_info = (struct ipc_chan_info *)chan->con_priv;
	struct mchp_ipc_sbi_msg request;

	request.buf_addr = chan_info->msg_buf_dma_rx;
	request.size = chan_info->max_msg_size;
	memcpy(chan_info->buf_base_rx, &request, sizeof(struct mchp_ipc_sbi_msg));
}

static inline void mchp_ipc_process_received_data(struct mbox_chan *chan,
						  struct mchp_ipc_msg *ipc_msg)
{
	struct ipc_chan_info *chan_info = (struct ipc_chan_info *)chan->con_priv;
	struct mchp_ipc_sbi_msg sbi_msg;

	memcpy(&sbi_msg, chan_info->buf_base_rx, sizeof(struct mchp_ipc_sbi_msg));
	ipc_msg->buf = (u32 *)chan_info->msg_buf_rx;
	ipc_msg->size = sbi_msg.size;
}

static irqreturn_t mchp_ipc_cluster_aggr_isr(int irq, void *data)
{
	struct mbox_chan *chan;
	struct ipc_chan_info *chan_info;
	struct microchip_ipc *ipc = (struct microchip_ipc *)data;
	struct mchp_ipc_msg ipc_msg;
	struct mchp_ipc_status status_msg;
	int ret;
	unsigned long hartid;
	u32 i, chan_index, chan_id;

	/* Find out the hart that originated the irq */
	for_each_online_cpu(i) {
		hartid = cpuid_to_hartid_map(i);
		if (irq == ipc->cluster_cfg[hartid].irq)
			break;
	}

	status_msg.cluster = hartid;
	memcpy(ipc->cluster_cfg[hartid].buf_base, &status_msg, sizeof(struct mchp_ipc_status));

	ret = mchp_ipc_sbi_send(SBI_EXT_IPC_STATUS, ipc->cluster_cfg[hartid].dma_addr);
	if (ret < 0) {
		dev_err_ratelimited(ipc->dev, "could not get IHC irq status ret=%d\n", ret);
		return IRQ_HANDLED;
	}

	memcpy(&status_msg, ipc->cluster_cfg[hartid].buf_base, sizeof(struct mchp_ipc_status));

	/*
	 * Iterate over each bit set in the IHC interrupt status register (IRQ_STATUS) to identify
	 * the channel(s) that have a message to be processed/acknowledged.
	 * The bits are organized in alternating format, where each pair of bits represents
	 * the status of the message present and message clear interrupts for each cluster/hart
	 * (from hart 0 to hart 5). Each cluster can have up to 5 fixed channels associated.
	 */

	for_each_set_bit(i, (unsigned long *)&status_msg.status, IRQ_STATUS_BITS) {
		/* Find out the destination hart that triggered the interrupt */
		chan_index = i / 2;

		/*
		 * The IP has no loopback channels, so we need to decrement the index when
		 * the target hart has a greater index than our own
		 */
		if (chan_index >= status_msg.cluster)
			chan_index--;

		/*
		 * Calculate the channel id given the hart and channel index. Channel IDs
		 * are unique across all clusters of an IPC, and iterate contiguously
		 * across all clusters.
		 */
		chan_id = status_msg.cluster * (NUM_CHANS_PER_CLUSTER + chan_index);

		chan = &ipc->chans[chan_id];
		chan_info = (struct ipc_chan_info *)chan->con_priv;

		if (i % 2 == 0) {
			mchp_ipc_prepare_receive_req(chan);
			ret = mchp_ipc_sbi_chan_send(SBI_EXT_IPC_RECEIVE, chan_id,
						     chan_info->dma_addr_rx);
			if (ret < 0)
				continue;

			mchp_ipc_process_received_data(chan, &ipc_msg);
			mbox_chan_received_data(&ipc->chans[chan_id], (void *)&ipc_msg);

		} else {
			ret = mchp_ipc_sbi_chan_send(SBI_EXT_IPC_RECEIVE, chan_id,
						     chan_info->dma_addr_rx);
			mbox_chan_txdone(&ipc->chans[chan_id], ret);
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t mchp_ipc_isr(int irq, void *data)
{
	struct mbox_chan *chan = (struct mbox_chan *)data;
	struct ipc_chan_info *chan_info = (struct ipc_chan_info *)chan->con_priv;
	struct microchip_ipc *ipc = to_mchp_ipc_mbox(chan->mbox);
	struct mchp_ipc_msg ipc_msg;
	int ret;

	mchp_ipc_prepare_receive_req(chan);

	ret = mchp_ipc_sbi_chan_send(SBI_EXT_IPC_RECEIVE, chan_info->id, chan_info->dma_addr_rx);
	if (ret < 0) {
		dev_err_ratelimited(ipc->dev,
				    "failed to receive message on channel %d, ret = %d\n",
				    chan_info->id,
				    ret);
		return IRQ_HANDLED;
	}

	if (irq == chan_info->mp_irq) {
		mchp_ipc_process_received_data(chan, &ipc_msg);
		mbox_chan_received_data(&ipc->chans[chan_info->id], (void *)&ipc_msg);
	}

	if (irq == chan_info->mc_irq)
		mbox_chan_txdone(&ipc->chans[chan_info->id], ret);

	return IRQ_HANDLED;
}

static irqreturn_t mchp_ipc_chan_aggr_isr(int irq, void *data)
{
	struct mbox_chan *chan = (struct mbox_chan *)data;
	struct ipc_chan_info *chan_info = (struct ipc_chan_info *)chan->con_priv;
	struct microchip_ipc *ipc = to_mchp_ipc_mbox(chan->mbox);
	struct mchp_ipc_sbi_msg sbi_msg;
	struct mchp_ipc_msg ipc_msg;
	int ret;

	mchp_ipc_prepare_receive_req(chan);

	ret = mchp_ipc_sbi_chan_send(SBI_EXT_IPC_RECEIVE, chan_info->id, chan_info->dma_addr_rx);
	if (ret < 0) {
		dev_err_ratelimited(ipc->dev,
				    "failed to receive message on channel %d, ret = %d\n",
				    chan_info->id,
				    ret);
		return IRQ_HANDLED;
	}

	memcpy(&sbi_msg, chan_info->buf_base_rx, sizeof(struct mchp_ipc_sbi_msg));

	if (sbi_msg.irq_type & IPC_MP_IRQ) {
		ipc_msg.buf = (u32 *)chan_info->msg_buf_rx;
		ipc_msg.size = sbi_msg.size;
		mbox_chan_received_data(&ipc->chans[chan_info->id], (void *)&ipc_msg);
	}

	if (sbi_msg.irq_type & IPC_MC_IRQ)
		mbox_chan_txdone(&ipc->chans[chan_info->id], ret);

	return IRQ_HANDLED;
}

static int mchp_ipc_get_irq(struct microchip_ipc *ipc, u32 id, const char *name)
{
	struct platform_device *pdev = to_platform_device(ipc->dev);
	char *irq_name;

	irq_name = devm_kasprintf(ipc->dev, GFP_KERNEL, "ch%u-%s", id, name);
	if (!irq_name)
		return -ENOMEM;

	return platform_get_irq_byname_optional(pdev, irq_name);
}

static int mchp_ipc_get_cluster_aggr_irq(struct microchip_ipc *ipc)
{
	struct platform_device *pdev = to_platform_device(ipc->dev);
	char *irq_name;
	int cpuid, ret;
	unsigned long hartid;
	bool irq_found = false;

	for_each_online_cpu(cpuid) {
		hartid = cpuid_to_hartid_map(cpuid);
		irq_name = devm_kasprintf(ipc->dev, GFP_KERNEL, "hart-%lu", hartid);
		ret = platform_get_irq_byname_optional(pdev, irq_name);
		if (ret <= 0)
			continue;

		ipc->cluster_cfg[hartid].irq = ret;
		ret = devm_request_irq(ipc->dev, ipc->cluster_cfg[hartid].irq,
				       mchp_ipc_cluster_aggr_isr, IRQF_SHARED,
				       "miv-ihc-irq", ipc);
		if (ret)
			return ret;

		ipc->cluster_cfg[hartid].buf_base = dmam_alloc_coherent(ipc->dev,
									sizeof(struct mchp_ipc_status),
									&ipc->cluster_cfg[hartid].dma_addr,
									GFP_KERNEL);

		if (!ipc->cluster_cfg[hartid].buf_base)
			return -ENOMEM;

		irq_found = true;
	}

	return irq_found;
}

static int mchp_ipc_get_chan_aggr_irq(struct microchip_ipc *ipc, u32 channel)
{
	struct platform_device *pdev = to_platform_device(ipc->dev);
	char *irq_name;

	irq_name = devm_kasprintf(ipc->dev, GFP_KERNEL, "ch%u", channel);
	return platform_get_irq_byname_optional(pdev, irq_name);
}

static int mchp_ipc_send_data(struct mbox_chan *chan, void *data)
{
	struct ipc_chan_info *chan_info = (struct ipc_chan_info *)chan->con_priv;
	const struct mchp_ipc_msg *msg = data;
	struct mchp_ipc_sbi_msg sbi_payload;

	memcpy(chan_info->msg_buf_tx, msg->buf, msg->size);
	sbi_payload.buf_addr = chan_info->msg_buf_dma_tx;
	sbi_payload.size = msg->size;
	memcpy(chan_info->buf_base_tx, &sbi_payload, sizeof(sbi_payload));

	return mchp_ipc_sbi_chan_send(SBI_EXT_IPC_SEND, chan_info->id, chan_info->dma_addr_tx);
}

static int mchp_ipc_startup(struct mbox_chan *chan)
{
	struct ipc_chan_info *chan_info = (struct ipc_chan_info *)chan->con_priv;
	struct microchip_ipc *ipc = to_mchp_ipc_mbox(chan->mbox);
	struct mchp_ipc_init ch_init_msg;
	int ret;

	chan_info->buf_base_tx = dma_alloc_coherent(ipc->dev, sizeof(struct mchp_ipc_sbi_msg),
						    &chan_info->dma_addr_tx, GFP_KERNEL);
	if (!chan_info->buf_base_tx) {
		ret = -ENOMEM;
		goto fail;
	}

	chan_info->buf_base_rx = dma_alloc_coherent(ipc->dev, sizeof(struct mchp_ipc_sbi_msg),
						    &chan_info->dma_addr_rx, GFP_KERNEL);
	if (!chan_info->buf_base_rx) {
		ret = -ENOMEM;
		goto fail_free_buf_base_tx;
	}

	ret = mchp_ipc_sbi_chan_send(SBI_EXT_IPC_CH_INIT, chan_info->id, chan_info->dma_addr_tx);
	if (ret < 0) {
		dev_err(ipc->dev, "channel %u init failed\n", chan_info->id);
		goto fail_free_buf_base_rx;
	}

	memcpy(&ch_init_msg, chan_info->buf_base_tx, sizeof(struct mchp_ipc_init));
	chan_info->max_msg_size = ch_init_msg.max_msg_size;

	chan_info->msg_buf_tx = dma_alloc_coherent(ipc->dev, chan_info->max_msg_size,
						   &chan_info->msg_buf_dma_tx, GFP_KERNEL);
	if (!chan_info->msg_buf_tx) {
		ret = -ENOMEM;
		goto fail_free_buf_base_rx;
	}

	chan_info->msg_buf_rx = dma_alloc_coherent(ipc->dev, chan_info->max_msg_size,
						   &chan_info->msg_buf_dma_rx, GFP_KERNEL);
	if (!chan_info->msg_buf_rx) {
		ret = -ENOMEM;
		goto fail_free_buf_msg_tx;
	}

	switch (ipc->hw_type) {
	case MIV_IHC:
		return 0;
	case RESERVED1:
		ret = devm_request_irq(ipc->dev, chan_info->mp_irq, mchp_ipc_isr, IRQF_SHARED,
				       "ipc-irq-mp", &ipc->chans[chan_info->id]);
		if (ret)
			goto fail_free_buf_msg_rx;
		ret = devm_request_irq(ipc->dev, chan_info->mc_irq, mchp_ipc_isr, IRQF_SHARED,
				       "ipc-irq-mc", &ipc->chans[chan_info->id]);
		break;
	case RESERVED2:
		ret = devm_request_irq(ipc->dev, chan_info->chan_aggregated_irq,
				       mchp_ipc_chan_aggr_isr, IRQF_SHARED, "ipc-irq",
				       &ipc->chans[chan_info->id]);
		break;
	default:
		goto fail_free_buf_msg_rx;
	}

	if (ret) {
		dev_err(ipc->dev, "failed to register interrupt(s)\n");
		goto fail_free_buf_msg_rx;
	}

	return ret;

fail_free_buf_msg_rx:
	dma_free_coherent(ipc->dev, chan_info->max_msg_size,
			  chan_info->msg_buf_rx, chan_info->msg_buf_dma_rx);
fail_free_buf_msg_tx:
	dma_free_coherent(ipc->dev, chan_info->max_msg_size,
			  chan_info->msg_buf_tx, chan_info->msg_buf_dma_tx);
fail_free_buf_base_rx:
	dma_free_coherent(ipc->dev, sizeof(struct mchp_ipc_sbi_msg),
			  chan_info->buf_base_rx, chan_info->dma_addr_rx);
fail_free_buf_base_tx:
	dma_free_coherent(ipc->dev, sizeof(struct mchp_ipc_sbi_msg),
			  chan_info->buf_base_tx, chan_info->dma_addr_tx);
fail:
	return ret;
}

static void mchp_ipc_shutdown(struct mbox_chan *chan)
{
	struct ipc_chan_info *chan_info = (struct ipc_chan_info *)chan->con_priv;
	struct microchip_ipc *ipc = to_mchp_ipc_mbox(chan->mbox);

	switch (ipc->hw_type) {
	case RESERVED1:
		devm_free_irq(ipc->dev, chan_info->mp_irq, chan);
		devm_free_irq(ipc->dev, chan_info->mc_irq, chan);
		break;
	case RESERVED2:
		devm_free_irq(ipc->dev, chan_info->chan_aggregated_irq, chan);
		break;
	default:
		break;
	}
	dma_free_coherent(ipc->dev, chan_info->max_msg_size,
			  chan_info->msg_buf_tx, chan_info->msg_buf_dma_tx);

	dma_free_coherent(ipc->dev, chan_info->max_msg_size,
			  chan_info->msg_buf_rx, chan_info->msg_buf_dma_rx);

	dma_free_coherent(ipc->dev, sizeof(struct mchp_ipc_sbi_msg),
			  chan_info->buf_base_tx, chan_info->dma_addr_tx);

	dma_free_coherent(ipc->dev, sizeof(struct mchp_ipc_sbi_msg),
			  chan_info->buf_base_rx, chan_info->dma_addr_rx);
}

static const struct mbox_chan_ops mchp_ipc_ops = {
	.startup = mchp_ipc_startup,
	.send_data = mchp_ipc_send_data,
	.shutdown = mchp_ipc_shutdown,
};

static struct mbox_chan *mchp_ipc_mbox_xlate(struct mbox_controller *controller,
					     const struct of_phandle_args *spec)
{
	struct microchip_ipc *ipc = to_mchp_ipc_mbox(controller);
	unsigned int chan_id = spec->args[0];

	if (chan_id >= ipc->num_channels) {
		dev_err(ipc->dev, "invalid channel id %d\n", chan_id);
		return ERR_PTR(-EINVAL);
	}

	return &ipc->chans[chan_id];
}

static const struct of_device_id mchp_ipc_of_match[] = {
	{.compatible = "microchip,sbi-ipc", },
	{}
};

static int mchp_ipc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mchp_ipc_probe ipc_info;
	struct microchip_ipc *ipc;
	struct ipc_chan_info *priv;
	bool irq_avail = false;
	int ret;
	u32 chan_id;

	ret = sbi_probe_extension(SBI_EXT_MICROCHIP_TECHNOLOGY);
	if (ret <= 0)
		return dev_err_probe(dev, ret, "Microchip SBI extension not detected\n");

	ipc = devm_kzalloc(dev, sizeof(*ipc), GFP_KERNEL);
	if (!ipc)
		return -ENOMEM;

	platform_set_drvdata(pdev, ipc);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(IPC_DMA_BIT_MASK));
	if (ret)
		return dev_err_probe(dev, ret, "dma_set_mask_and_coherent failed\n");

	ipc->buf_base = dmam_alloc_coherent(dev, sizeof(u32), &ipc->dma_addr, GFP_KERNEL);

	if (!ipc->buf_base)
		return -ENOMEM;

	ret = mchp_ipc_sbi_send(SBI_EXT_IPC_PROBE, ipc->dma_addr);
	if (ret < 0)
		return dev_err_probe(dev, ret, "could not probe IPC SBI service\n");

	memcpy(&ipc_info, ipc->buf_base, sizeof(struct mchp_ipc_probe));
	ipc->num_channels = ipc_info.num_channels;
	ipc->hw_type = ipc_info.hw_type;

	ipc->chans = devm_kcalloc(dev, ipc->num_channels, sizeof(*ipc->chans), GFP_KERNEL);
	if (!ipc->chans)
		return -ENOMEM;

	ipc->dev = dev;
	ipc->controller.txdone_irq = true;
	ipc->controller.dev = ipc->dev;
	ipc->controller.ops = &mchp_ipc_ops;
	ipc->controller.chans = ipc->chans;
	ipc->controller.num_chans = ipc->num_channels;
	ipc->controller.of_xlate = mchp_ipc_mbox_xlate;

	for (chan_id = 0; chan_id < ipc->num_channels; chan_id++) {
		priv = devm_kmalloc(dev, sizeof(*priv), GFP_KERNEL);
		if (!priv)
			return -ENOMEM;

		ipc->chans[chan_id].con_priv = priv;
		priv->id = chan_id;

		if (ipc->hw_type == RESERVED1) {
			priv->mp_irq = mchp_ipc_get_irq(ipc, chan_id, "mp");
			priv->mc_irq = mchp_ipc_get_irq(ipc, chan_id, "mc");

			if (priv->mp_irq > 0 && priv->mc_irq > 0)
				irq_avail = true;
		}

		if (ipc->hw_type == RESERVED2) {
			priv->chan_aggregated_irq = mchp_ipc_get_chan_aggr_irq(ipc, chan_id);
			if (priv->chan_aggregated_irq > 0)
				irq_avail = true;
		}
	}

	if (ipc->hw_type == MIV_IHC) {
		ipc->cluster_cfg = devm_kcalloc(dev, num_online_cpus(),
						sizeof(struct mchp_ipc_cluster_cfg),
						GFP_KERNEL);
		if (!ipc->cluster_cfg)
			return -ENOMEM;

		if (mchp_ipc_get_cluster_aggr_irq(ipc))
			irq_avail = true;
	}

	if (!irq_avail)
		return dev_err_probe(dev, -ENODEV, "missing interrupt property\n");

	ret = devm_mbox_controller_register(dev, &ipc->controller);
	if (ret)
		return dev_err_probe(dev, ret,
					 "Inter-Processor communication (IPC) registration failed\n");

	return 0;
}

MODULE_DEVICE_TABLE(of, mchp_ipc_of_match);

static struct platform_driver mchp_ipc_driver = {
	.driver = {
		.name = "microchip_ipc",
		.of_match_table = of_match_ptr(mchp_ipc_of_match),
	},
	.probe = mchp_ipc_probe,
};

module_platform_driver(mchp_ipc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Valentina Fernandez <valentina.fernandezalanis@microchip.com>");
MODULE_DESCRIPTION("Microchip Inter-Processor Communication (IPC) driver");
