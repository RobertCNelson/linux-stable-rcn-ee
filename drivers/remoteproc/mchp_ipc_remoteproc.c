// SPDX-License-Identifier: GPL-2.0-only
/*
 * Microchip IPC Remoteproc driver
 *
 * Copyright (c) 2021 - 2024 Microchip Technology Inc. All rights reserved.
 *
 * Author: Valentina Fernandez <valentina.fernandezalanis@microchip.com>
 *
 * Derived from the imx_rproc implementation:
 * Copyright (c) 2017 Pengutronix, Oleksij Rempel <kernel@pengutronix.de>
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/workqueue.h>

#include <asm/sbi.h>
#include <asm/vendorid_list.h>
#include <linux/mailbox/mchp-ipc.h>

#include "remoteproc_internal.h"

#define SBI_EXT_MICROCHIP_TECHNOLOGY	(SBI_EXT_VENDOR_START | \
					 MICROCHIP_VENDOR_ID)

#define MIV_RPROC_MEM_MAX		2

enum {
	SBI_EXT_RPROC_STATE = 0x3,
	SBI_EXT_RPROC_START,
	SBI_EXT_RPROC_STOP,
};

/**
 * enum mchp_ipc_rproc_mbox_messages - predefined mailbox messages
 *
 * @MCHP_IPC_RPROC_MBOX_READY: a ready message response from a remote context indicating
 * that the remote context is up and running.
 *
 * @MIV_RP_MBOX_PENDING_MSG: Not currently in use, but reserved for future use
 * to inform the receiver that there is a message awaiting in its receive-side
 * vring. At the moment, one can explicitly send the index of the triggered
 * virtqueue as a payload.
 *
 * @MIV_RP_MBOX_STOP: a stop request for the remote context
 *
 * @MIV_RP_MBOX_END_MSG: Indicates end of known/defined messages.
 * This should be the last definition.
 *
 */
enum mchp_ipc_rproc_mbox_messages {
	MCHP_IPC_RPROC_MBOX_READY = 0xFFFFFF00,
	MCHP_IPC_RPROC_MBOX_PENDING_MSG = 0xFFFFFF01,
	MCHP_IPC_RPROC_MBOX_STOP = 0xFFFFFF02,
	MCHP_IPC_RPROC_MBOX_END_MSG = 0xFFFFFF03,
};

struct mchp_ipc_rproc {
	struct device *dev;
	struct rproc *rproc;
	struct mbox_chan *mbox_channel;
	struct workqueue_struct *workqueue;
	struct mbox_client mbox_client;
	struct completion start_done;
	struct work_struct rproc_work;
	struct mchp_ipc_msg message;
	void __iomem *rsc_table;
	bool initialized;
	u32 chan_id;
};

static int mchp_ipc_rproc_get_status(u32 chan)
{
	struct sbiret ret;

	ret = sbi_ecall(SBI_EXT_MICROCHIP_TECHNOLOGY, SBI_EXT_RPROC_STATE,
			chan, 0, 0, 0, 0, 0);

	if (ret.error)
		return sbi_err_map_linux_errno(ret.error);

	return ret.value;
}

static int mchp_ipc_rproc_start(struct rproc *rproc)
{
	struct mchp_ipc_rproc *priv = rproc->priv;
	struct sbiret ret;
	int result;

	ret = sbi_ecall(SBI_EXT_MICROCHIP_TECHNOLOGY, SBI_EXT_RPROC_START,
			priv->chan_id, rproc->bootaddr, 0, 0, 0, 0);

	if (ret.error)
		return sbi_err_map_linux_errno(ret.error);

	result = wait_for_completion_timeout(&priv->start_done,
					     msecs_to_jiffies(5000));
	if (!result) {
		dev_err(priv->dev, "timeout waiting for ready notification\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int mchp_ipc_rproc_stop(struct rproc *rproc)
{
	struct mchp_ipc_rproc *priv = rproc->priv;
	struct sbiret ret;

	ret = sbi_ecall(SBI_EXT_MICROCHIP_TECHNOLOGY, SBI_EXT_RPROC_STOP,
			priv->chan_id, MCHP_IPC_RPROC_MBOX_STOP, 0, 0, 0, 0);

	if (ret.error)
		return sbi_err_map_linux_errno(ret.error);

	return ret.value;
}

static int mchp_ipc_rproc_mem_alloc(struct rproc *rproc, struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;
	void *va;

	dev_dbg(dev, "map memory: %pad+%zx\n", &mem->dma, mem->len);
	va = ioremap_wc(mem->dma, mem->len);
	if (IS_ERR_OR_NULL(va)) {
		dev_err(dev, "Unable to map memory region: %p+%zx\n",
			&mem->dma, mem->len);
		return -ENOMEM;
	}

	mem->va = va;

	return 0;
}

static int mchp_ipc_rproc_mem_release(struct rproc *rproc,
				      struct rproc_mem_entry *mem)
{
	dev_dbg(rproc->dev.parent, "unmap memory: %pad\n", &mem->dma);
	iounmap(mem->va);

	return 0;
}

static int mchp_ipc_rproc_prepare(struct rproc *rproc)
{
	struct mchp_ipc_rproc *priv = rproc->priv;
	struct device_node *np = priv->dev->of_node;
	struct rproc_mem_entry *mem;
	struct reserved_mem *rmem;
	struct of_phandle_iterator it;
	u64 device_address;

	reinit_completion(&priv->start_done);

	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	while (of_phandle_iterator_next(&it) == 0) {
		/*
		 * Ignore the first memory region which will be used vdev
		 * buffer. No need to do extra handlings, rproc_add_virtio_dev
		 * will handle it.
		 */
		if (!strcmp(it.node->name, "vdev0buffer"))
			continue;

		if (!strcmp(it.node->name, "rsc-table"))
			continue;

		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			dev_err(priv->dev, "unable to acquire memory-region\n");
			return -EINVAL;
		}

		device_address = rmem->base;

		mem = rproc_mem_entry_init(priv->dev, NULL, (dma_addr_t)rmem->base,
					   rmem->size, device_address, mchp_ipc_rproc_mem_alloc,
					   mchp_ipc_rproc_mem_release, it.node->name);

		if (!mem)
			return -ENOMEM;

		rproc_coredump_add_segment(rproc, device_address, rmem->size);
		rproc_add_carveout(rproc, mem);
	}

	return 0;
}

static int mchp_ipc_rproc_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	int ret;

	ret = rproc_elf_load_rsc_table(rproc, fw);
	if (ret)
		dev_info(&rproc->dev, "No resource table in elf\n");

	return 0;
}

static void mchp_ipc_rproc_kick(struct rproc *rproc, int vqid)
{
	struct mchp_ipc_rproc *priv = (struct mchp_ipc_rproc *)rproc->priv;
	struct mchp_ipc_msg msg;
	int ret;

	msg.buf = (void *)&vqid;
	msg.size = sizeof(vqid);

	ret = mbox_send_message(priv->mbox_channel, (void *)&msg);
	if (ret < 0)
		dev_err(priv->dev,
			"failed to send mbox message, status = %d\n", ret);
}

static int mchp_ipc_rproc_attach(struct rproc *rproc)
{
	return 0;
}

static struct resource_table
*mchp_ipc_rproc_get_loaded_rsc_table(struct rproc *rproc, size_t *table_sz)
{
	struct mchp_ipc_rproc *priv = rproc->priv;

	if (!priv->rsc_table)
		return NULL;

	*table_sz = SZ_1K;
	return (struct resource_table *)priv->rsc_table;
}

static const struct rproc_ops mchp_ipc_rproc_ops = {
	.prepare = mchp_ipc_rproc_prepare,
	.start = mchp_ipc_rproc_start,
	.get_loaded_rsc_table = mchp_ipc_rproc_get_loaded_rsc_table,
	.attach = mchp_ipc_rproc_attach,
	.stop = mchp_ipc_rproc_stop,
	.kick = mchp_ipc_rproc_kick,
	.load = rproc_elf_load_segments,
	.parse_fw = mchp_ipc_rproc_parse_fw,
	.find_loaded_rsc_table = rproc_elf_find_loaded_rsc_table,
	.sanity_check = rproc_elf_sanity_check,
	.get_boot_addr = rproc_elf_get_boot_addr,
};

static int mchp_ipc_rproc_addr_init(struct mchp_ipc_rproc *priv,
				    struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int i, err, rmem_np;

	rmem_np = of_count_phandle_with_args(np, "memory-region", NULL);
	if (rmem_np <= 0)
		return 0;

	for (i = 0; i < rmem_np; i++) {
		struct device_node *node;
		struct resource res;

		node = of_parse_phandle(np, "memory-region", i);

		if (!strncmp(node->name, "vdev", strlen("vdev"))) {
			of_node_put(node);
			continue;
		}

		if (!strncmp(node->name, "rsc-table", strlen("rsc-table"))) {
			of_node_put(node);
			err = of_address_to_resource(node, 0, &res);
			if (err)
				return dev_err_probe(dev, err,
						     "unable to resolve memory region\n");
			priv->rsc_table = devm_ioremap(&pdev->dev,
						       res.start, resource_size(&res));
		}
	}

	return 0;
}

static void mchp_ipc_rproc_vq_work(struct work_struct *work)
{
	struct mchp_ipc_rproc *priv = container_of(work, struct mchp_ipc_rproc, rproc_work);
	struct device *dev = priv->rproc->dev.parent;

	u32 msg = priv->message.buf[0];

	/*
	 * Currently, we are expected to receive the following messages
	 * from the remote cluster: a ready message or receive the index
	 * of the triggered virtqueue as a payload.
	 * We can silently ignore any other type of mailbox messages since
	 * they are not meant for us and are meant to be received by the
	 * remote cluster only.
	 */
	switch (msg) {
	case MCHP_IPC_RPROC_MBOX_READY:
		complete(&priv->start_done);
		break;
	default:
		if (msg >= MCHP_IPC_RPROC_MBOX_READY && msg < MCHP_IPC_RPROC_MBOX_END_MSG)
			return;
		if (msg > priv->rproc->max_notifyid) {
			dev_info(dev, "dropping unknown message 0x%x", msg);
			return;
		}
		/* msg contains the index of the triggered vring */
		if (rproc_vq_interrupt(priv->rproc, msg) == IRQ_NONE)
			dev_dbg(dev, "no message was found in vqid %d\n", msg);
	}
}

static void mchp_ipc_rproc_rx_callback(struct mbox_client *mbox_client, void *msg)
{
	struct rproc *rproc = dev_get_drvdata(mbox_client->dev);
	struct mchp_ipc_rproc *priv = rproc->priv;

	priv->message = *(struct mchp_ipc_msg *)msg;
	queue_work(priv->workqueue, &priv->rproc_work);
}

static int mchp_ipc_rproc_mbox_init(struct rproc *rproc)
{
	struct mchp_ipc_rproc *priv = rproc->priv;
	struct device *dev = priv->dev;
	struct mbox_client *mbox_client;

	mbox_client = &priv->mbox_client;
	mbox_client->dev = dev;
	mbox_client->tx_block = true;
	mbox_client->tx_tout = 100;
	mbox_client->knows_txdone = false;
	mbox_client->rx_callback = mchp_ipc_rproc_rx_callback;

	priv->mbox_channel = mbox_request_channel(mbox_client, 0);
	if (IS_ERR(priv->mbox_channel))
		return dev_err_probe(mbox_client->dev,
				     PTR_ERR(priv->mbox_channel),
				     "failed to request mailbox channel\n");

	priv->chan_id = mchp_ipc_get_chan_id(priv->mbox_channel);

	return 0;
}

static int mchp_ipc_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct mchp_ipc_rproc *priv;
	struct rproc *rproc;
	int ret;

	rproc = rproc_alloc(dev, np->name, &mchp_ipc_rproc_ops,
			    NULL, sizeof(*priv));
	if (!rproc)
		return -ENOMEM;

	priv = rproc->priv;
	priv->rproc = rproc;
	priv->dev = dev;

	dev_set_drvdata(dev, rproc);

	priv->workqueue = create_workqueue(dev_name(dev));
	if (!priv->workqueue) {
		ret = -ENOMEM;
		dev_err_probe(dev, ret, "cannot create workqueue\n");
		goto err_put_rproc;
	}

	INIT_WORK(&priv->rproc_work, mchp_ipc_rproc_vq_work);
	init_completion(&priv->start_done);

	ret = mchp_ipc_rproc_mbox_init(rproc);
	if (ret)
		goto err_put_wkq;

	ret = mchp_ipc_rproc_addr_init(priv, pdev);
	if (ret)
		goto err_put_mbox;

	rproc->state = mchp_ipc_rproc_get_status(priv->chan_id);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Couldn't get channel status\n");
		goto err_put_mbox;
	}

	if (rproc->state != RPROC_DETACHED)
		rproc->auto_boot = of_property_present(np, "microchip,auto-boot");

	ret = rproc_add(rproc);
	if (ret) {
		dev_err_probe(dev, ret, "rproc_add failed\n");
		goto err_put_mbox;
	}

	return 0;

err_put_mbox:
	mbox_free_channel(priv->mbox_channel);
err_put_wkq:
	destroy_workqueue(priv->workqueue);
err_put_rproc:
	rproc_free(rproc);

	return ret;
}

static int mchp_ipc_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct mchp_ipc_rproc *priv = rproc->priv;

	rproc_del(rproc);
	mbox_free_channel(priv->mbox_channel);
	destroy_workqueue(priv->workqueue);
	rproc_free(rproc);

	return 0;
}

static const struct of_device_id mchp_ipc_rproc_of_match[] __maybe_unused = {
	{ .compatible = "microchip,ipc-remoteproc", },
	{}
};
MODULE_DEVICE_TABLE(of, mchp_ipc_rproc_of_match);

static struct platform_driver mchp_ipc_rproc_driver = {
	.probe = mchp_ipc_rproc_probe,
	.remove = mchp_ipc_rproc_remove,
	.driver = {
		.name = "microchip-ipc-rproc",
		.of_match_table = of_match_ptr(mchp_ipc_rproc_of_match),
	},
};

module_platform_driver(mchp_ipc_rproc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Valentina Fernandez <valentina.fernandezalanis@microchip.com>");
MODULE_DESCRIPTION("Microchip IPC Remote Processor control driver");
