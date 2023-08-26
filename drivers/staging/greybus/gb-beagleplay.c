// SPDX-License-Identifier: GPL-2.0
/*
 * Beagleplay Linux Driver for Greybus
 *
 * Copyright (c) 2023 Ayush Singh <ayushdevel1325@gmail.com>
 */

#include <linux/gfp.h>
#include <linux/greybus.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/serdev.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/greybus/hd.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/crc-ccitt.h>
#include <linux/circ_buf.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#define RX_HDLC_PAYLOAD 1024
#define CRC_LEN 2
#define MAX_RX_HDLC (1 + RX_HDLC_PAYLOAD + CRC_LEN)
#define TX_CIRC_BUF_SIZE 1024

#define ADDRESS_GREYBUS 0x01
#define ADDRESS_DBG 0x02
#define ADDRESS_CONTROL 0x04

#define HDLC_FRAME 0x7E
#define HDLC_ESC 0x7D
#define HDLC_XOR 0x20

#define CONTROL_SVC_START 0x01
#define CONTROL_SVC_STOP 0x02

/* The maximum number of CPorts supported by Greybus Host Device */
#define BEAGLEPLAY_GB_MAX_CPORTS 32

static const struct of_device_id gb_beagleplay_of_match[] = {
	{
		/* The UART connecting AM62 and CC1352 */
		.compatible = "beagle,beagleplaygreybus",
	},
	{},
};
MODULE_DEVICE_TABLE(of, gb_beagleplay_of_match);

/*
 * BeaglePlay Greybus driver
 *
 * @param serdev device
 *
 * @param greybud host device
 *
 * @param callback called when hdlc block is received
 * @param callback called to send hdlc block
 *
 * @param transmit work
 * @param transmit producer lock
 * @param transmit consumer lock
 * @param transmit circular buffer
 * @param HDCL CRC
 * @param current TX ACK sequence number
 *
 * @param Rx buffer length
 * @param Rx Flag to indicate if ESC
 * @parma Rx buffer
 */
struct gb_beagleplay {
	struct serdev_device *serdev;

	struct gb_host_device *gb_host_device;

	struct work_struct tx_work;
	/* tx_producer_lock: HDLC producer lock */
	spinlock_t tx_producer_lock;
	/* tx_consumer_lock: HDLC consumer lock */
	spinlock_t tx_consumer_lock;
	struct circ_buf tx_circ_buf;
	u16 tx_crc;
	u8 tx_ack_seq;

	u16 rx_buffer_len;
	u8 rx_address;
	u8 rx_in_esc;
	u8 rx_buffer[MAX_RX_HDLC];
};

struct hdlc_payload {
	u16 length;
	void *payload;
};

static void hdlc_handle_greybus_frame(struct gb_beagleplay *bg, u8 *buffer,
				      size_t buffer_len)
{
	u16 cport_id;
	struct gb_operation_msg_hdr *hdr =
		(struct gb_operation_msg_hdr *)buffer;

	memcpy(&cport_id, hdr->pad, sizeof(cport_id));

	if (hdr->result) {
		dev_warn(
			&bg->serdev->dev,
			"Failed Greybus Operation %u of Type %X on CPort %u with Status %u",
			hdr->operation_id, hdr->type, cport_id, hdr->result);
	} else {
		dev_dbg(&bg->serdev->dev,
			"Successful Greybus Operation %u of Type %X on CPort %u",
			hdr->operation_id, hdr->type, cport_id);
	}

	greybus_data_rcvd(bg->gb_host_device, cport_id, buffer, buffer_len);
}

static void hdlc_handle_dbg_frame(struct gb_beagleplay *bg, const char *buffer,
				  size_t buffer_len)
{
	dev_dbg(&bg->serdev->dev, "CC1352 Debug: %.*s", (int)buffer_len,
		buffer);
}

static void hdlc_handle_frame(struct gb_beagleplay *bg, u8 *buffer,
			      size_t buffer_len, uint8_t address)
{
	switch (address) {
	case ADDRESS_DBG:
		hdlc_handle_dbg_frame(bg, buffer, buffer_len);
		break;
	case ADDRESS_GREYBUS:
		hdlc_handle_greybus_frame(bg, buffer, buffer_len);
		break;
	default:
		dev_warn(&bg->serdev->dev, "Got Unknown Frame %u", address);
	}
}

static void hdlc_write(struct gb_beagleplay *bg)
{
	int head = smp_load_acquire(&bg->tx_circ_buf.head);
	int tail = bg->tx_circ_buf.tail;
	int count = CIRC_CNT_TO_END(head, tail, TX_CIRC_BUF_SIZE);
	int written;

	if (count >= 1) {
		written = serdev_device_write_buf(
			bg->serdev, &bg->tx_circ_buf.buf[tail], count);

		/* Finish consuming HDLC data */
		smp_store_release(&bg->tx_circ_buf.tail,
				  (tail + written) & (TX_CIRC_BUF_SIZE - 1));
	}
}

static void hdlc_append(struct gb_beagleplay *bg, u8 value)
{
	int tail, head = bg->tx_circ_buf.head;

	while (true) {
		tail = READ_ONCE(bg->tx_circ_buf.tail);

		if (CIRC_SPACE(head, tail, TX_CIRC_BUF_SIZE) >= 1) {
			bg->tx_circ_buf.buf[head] = value;

			/* Finish producing HDLC byte */
			smp_store_release(&bg->tx_circ_buf.head,
					  (head + 1) & (TX_CIRC_BUF_SIZE - 1));
			return;
		}
		dev_warn(&bg->serdev->dev, "Tx circ buf full");
		usleep_range(3000, 5000);
	}
}

static void hdlc_append_escaped(struct gb_beagleplay *bg, u8 value)
{
	if (value == HDLC_FRAME || value == HDLC_ESC) {
		hdlc_append(bg, HDLC_ESC);
		value ^= HDLC_XOR;
	}
	hdlc_append(bg, value);
}

static void hdlc_append_tx_frame(struct gb_beagleplay *bg)
{
	bg->tx_crc = 0xFFFF;
	hdlc_append(bg, HDLC_FRAME);
}

static void hdlc_append_tx_u8(struct gb_beagleplay *bg, u8 value)
{
	bg->tx_crc = crc_ccitt(bg->tx_crc, &value, 1);
	hdlc_append_escaped(bg, value);
}

static void hdlc_append_tx_buffer(struct gb_beagleplay *bg, const u8 *buffer,
				  size_t len)
{
	size_t i;

	for (i = 0; i < len; i++)
		hdlc_append_tx_u8(bg, buffer[i]);
}

static void hdlc_append_tx_crc(struct gb_beagleplay *bg)
{
	bg->tx_crc ^= 0xffff;
	hdlc_append_escaped(bg, bg->tx_crc & 0xff);
	hdlc_append_escaped(bg, (bg->tx_crc >> 8) & 0xff);
}

static void hdlc_handle_rx_frame(struct gb_beagleplay *bg)
{
	u8 address = bg->rx_buffer[0];
	char *buf = &bg->rx_buffer[2];
	size_t buf_len = bg->rx_buffer_len - 4;

	hdlc_handle_frame(bg, buf, buf_len, address);
}

static void hdlc_transmit(struct work_struct *work)
{
	struct gb_beagleplay *bg =
		container_of(work, struct gb_beagleplay, tx_work);

	spin_lock_bh(&bg->tx_consumer_lock);
	hdlc_write(bg);
	spin_unlock_bh(&bg->tx_consumer_lock);
}

static void hdlc_send_async(struct gb_beagleplay *bg, u8 address, u8 control,
			    const struct hdlc_payload payloads[], size_t count)
{
	size_t i;

	/* HDLC_FRAME
	 * 0 address : 0x01
	 * 1 control : 0x03
	 * contents
	 * x/y crc
	 * HDLC_FRAME
	 */

	spin_lock(&bg->tx_producer_lock);

	hdlc_append_tx_frame(bg);
	hdlc_append_tx_u8(bg, address);
	hdlc_append_tx_u8(bg, control);
	for (i = 0; i < count; ++i) {
		hdlc_append_tx_buffer(bg, payloads[i].payload,
				      payloads[i].length);
	}
	hdlc_append_tx_crc(bg);
	hdlc_append_tx_frame(bg);

	spin_unlock(&bg->tx_producer_lock);

	schedule_work(&bg->tx_work);
}

static void hdlc_send_s_frame_ack(struct gb_beagleplay *bg)
{
	hdlc_send_async(bg, bg->rx_buffer[0], (bg->rx_buffer[1] >> 1) & 0x7,
			NULL, 0);
}

static int hdlc_rx(struct gb_beagleplay *bg, const u8 *data, size_t count)
{
	u16 crc_check;
	size_t i;
	u8 c, ctrl;

	for (i = 0; i < count; ++i) {
		c = data[i];

		switch (c) {
		case HDLC_FRAME: {
			if (bg->rx_buffer_len) {
				crc_check = crc_ccitt(0xffff, bg->rx_buffer,
						      bg->rx_buffer_len);

				if (crc_check == 0xf0b8) {
					ctrl = bg->rx_buffer[1];
					if ((ctrl & 1) == 0) {
						/* I-Frame, send S-Frame ACK */
						hdlc_send_s_frame_ack(bg);
					}

					hdlc_handle_rx_frame(bg);
				} else {
					dev_err(&bg->serdev->dev,
						"CRC Failed from %02x: 0x%04x",
						bg->rx_buffer[0], crc_check);
				}
			}
			bg->rx_buffer_len = 0;
			break;
		}
		case HDLC_ESC:
			bg->rx_in_esc = 1;
			break;
		default:
			if (bg->rx_in_esc) {
				c ^= 0x20;
				bg->rx_in_esc = 0;
			}

			if (bg->rx_buffer_len < MAX_RX_HDLC) {
				bg->rx_buffer[bg->rx_buffer_len] = c;
				bg->rx_buffer_len++;
			} else {
				dev_err(&bg->serdev->dev, "RX Buffer Overflow");
				bg->rx_buffer_len = 0;
			}
		}
	}

	return count;
}

static void hdlc_init(struct gb_beagleplay *bg)
{
	INIT_WORK(&bg->tx_work, hdlc_transmit);
	spin_lock_init(&bg->tx_producer_lock);
	spin_lock_init(&bg->tx_consumer_lock);
	bg->tx_circ_buf.head = 0;
	bg->tx_circ_buf.tail = 0;
	bg->tx_circ_buf.buf =
		devm_kmalloc(&bg->serdev->dev, TX_CIRC_BUF_SIZE, GFP_KERNEL);
	bg->rx_buffer_len = 0;
	bg->rx_in_esc = 0;
}

static void hdlc_deinit(struct gb_beagleplay *bg)
{
	flush_work(&bg->tx_work);
}

static int gb_beagleplay_tty_receive(struct serdev_device *serdev,
				     const unsigned char *data, size_t count)
{
	struct gb_beagleplay *bg = serdev_device_get_drvdata(serdev);

	return hdlc_rx(bg, data, count);
}

static void beagleplay_greybus_tty_wakeup(struct serdev_device *serdev)
{
	struct gb_beagleplay *bg = serdev_device_get_drvdata(serdev);

	schedule_work(&bg->tx_work);
}

static struct serdev_device_ops gb_beagleplay_ops = {
	.receive_buf = gb_beagleplay_tty_receive,
	.write_wakeup = beagleplay_greybus_tty_wakeup,
};

static int gb_message_send(struct gb_host_device *hd, u16 cport_id,
			   struct gb_message *msg, gfp_t gfp_mask)
{
	struct gb_beagleplay *bg = dev_get_drvdata(&hd->dev);
	struct hdlc_payload payloads[2];

	dev_dbg(&hd->dev,
		"Sending Greybus message with Operation %u, Type: %X on Cport %u",
		msg->header->operation_id, msg->header->type, cport_id);

	if (msg->header->size > RX_HDLC_PAYLOAD) {
		dev_err(&hd->dev, "Greybus message too big");
		return -E2BIG;
	}

	memcpy(msg->header->pad, &cport_id, sizeof(cport_id));

	payloads[0].payload = msg->header;
	payloads[0].length = sizeof(*msg->header);
	payloads[1].payload = msg->payload;
	payloads[1].length = msg->payload_size;

	hdlc_send_async(bg, ADDRESS_GREYBUS, 0x03, payloads, 2);
	greybus_message_sent(bg->gb_host_device, msg, 0);

	return 0;
}

static void gb_message_cancel(struct gb_message *message)
{
}

static struct gb_hd_driver gb_hdlc_driver = { .message_send = gb_message_send,
					      .message_cancel =
						      gb_message_cancel };

static void gb_beagleplay_start_svc(struct gb_beagleplay *bg)
{
	const u8 command = CONTROL_SVC_START;
	const struct hdlc_payload payload = { .length = 1,
					      .payload = (void *)&command };

	hdlc_send_async(bg, ADDRESS_CONTROL, 0x03, &payload, 1);
}

static void gb_beagleplay_stop_svc(struct gb_beagleplay *bg)
{
	const u8 command = CONTROL_SVC_STOP;
	const struct hdlc_payload payload = { .length = 1,
					      .payload = (void *)&command };

	hdlc_send_async(bg, ADDRESS_CONTROL, 0x03, &payload, 1);
}

static int gb_beagleplay_probe(struct serdev_device *serdev)
{
	u32 speed = 115200;
	int ret = 0;
	struct gb_beagleplay *bg =
		devm_kmalloc(&serdev->dev, sizeof(*bg), GFP_KERNEL);

	if (!bg) {
		dev_err(&serdev->dev, "Failed to allocate driver");
		return -ENOMEM;
	}

	bg->serdev = serdev;
	serdev_device_set_drvdata(serdev, bg);
	serdev_device_set_client_ops(serdev, &gb_beagleplay_ops);
	ret = serdev_device_open(serdev);
	if (ret) {
		dev_err(&bg->serdev->dev, "Unable to Open Device");
		return ret;
	}
	speed = serdev_device_set_baudrate(serdev, speed);
	dev_info(&bg->serdev->dev, "Using baudrate %u", speed);
	serdev_device_set_flow_control(serdev, false);

	hdlc_init(bg);

	/* Greybus setup */
	bg->gb_host_device = gb_hd_create(&gb_hdlc_driver, &serdev->dev,
					  TX_CIRC_BUF_SIZE,
					  BEAGLEPLAY_GB_MAX_CPORTS);
	if (IS_ERR(bg->gb_host_device)) {
		dev_err(&bg->serdev->dev,
			"Unable to create Greybus Host Device");
		ret = -1;
		goto free_hdlc;
	}
	ret = gb_hd_add(bg->gb_host_device);
	if (ret) {
		dev_err(&serdev->dev, "Failed to add Greybus Host Device");
		goto free_gb_hd;
	}
	dev_set_drvdata(&bg->gb_host_device->dev, bg);

	gb_beagleplay_start_svc(bg);

	dev_info(&bg->serdev->dev, "Successful Beagleplay Greybus Probe");

	return 0;

free_gb_hd:
	gb_hd_del(bg->gb_host_device);
	gb_hd_put(bg->gb_host_device);
free_hdlc:
	hdlc_deinit(bg);
	serdev_device_close(serdev);
	return ret;
}

static void gb_beagleplay_remove(struct serdev_device *serdev)
{
	struct gb_beagleplay *beagleplay_greybus =
		serdev_device_get_drvdata(serdev);

	dev_info(&beagleplay_greybus->serdev->dev,
		 "Remove BeaglePlay Greybus Driver");

	gb_hd_del(beagleplay_greybus->gb_host_device);
	gb_hd_put(beagleplay_greybus->gb_host_device);

	gb_beagleplay_stop_svc(beagleplay_greybus);

	hdlc_deinit(beagleplay_greybus);

	serdev_device_close(serdev);
}

static struct serdev_device_driver gb_beagleplay_driver = {
	.probe = gb_beagleplay_probe,
	.remove = gb_beagleplay_remove,
	.driver = {
	      .name = "gb_beagleplay",
	      .of_match_table = of_match_ptr(gb_beagleplay_of_match),
	    },
};

module_serdev_device_driver(gb_beagleplay_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ayush Singh <ayushdevel1325@gmail.com>");
MODULE_DESCRIPTION("A Greybus driver for BeaglePlay");
