#include <linux/irq.h>
#include <linux/kthread.h>
#include <linux/firmware.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>

#include "cfg80211.h"
#include "wlan_cfg.h"
#include "wilcs02_loopback.h"
#ifdef WILC_S02_TEST_BUS_INTERFACE
u8 test_frame[WILC_S02_TEST_FRAME_COUNT][WILC_S02_TEST_FRAME_SIZE];
u32 test_in_progress;
static u32 test_run_cnt;
static u8 fmt_str[4096] = {0};

extern int wilc_wlan_txq_add_cfg_pkt(struct wilc_vif *vif, u8 *buffer,
				     u32 buffer_size);

int time_compare(struct timespec64 *time1, struct timespec64 *time2)
{
	if (time1->tv_sec < time2->tv_sec)
		return -1;
	if (time1->tv_sec > time2->tv_sec)
		return 1;
	if (time1->tv_nsec < time2->tv_nsec)
		return -1;
	if (time1->tv_nsec > time2->tv_nsec)
		return 1;

	return 0;
}

int time_diff(struct timespec64 *time1, struct timespec64 *time2,
	      struct timespec64 *diff)
{
	int past = 0;
	int cmp = 0;

	cmp = time_compare(time1, time2);
	if (cmp == 0) {
		diff->tv_sec = 0;
		diff->tv_nsec = 0;
		past = 1;
	} else if (cmp == 1) {
		diff->tv_sec = time1->tv_sec - time2->tv_sec;
		diff->tv_nsec = time1->tv_nsec;

		if (diff->tv_nsec < time2->tv_nsec) {
			diff->tv_sec -= 1;
			diff->tv_nsec += 1000000;
		}
		diff->tv_nsec = diff->tv_nsec - time2->tv_nsec;
	} else {
		diff->tv_sec = time2->tv_sec - time1->tv_sec;
		diff->tv_nsec = time2->tv_nsec;
		if (diff->tv_nsec < time1->tv_nsec) {
			diff->tv_sec -= 1;
			diff->tv_nsec += 1000000;
		}
		diff->tv_nsec = diff->tv_nsec - time1->tv_nsec;
		past = 1;
	}
	return past;
}

static void wilc_test_frame_tx_complete(void *priv, int status)
{
	pr_debug("%s tx frame copy completed\n", __func__);
}

void wilc_test_bus_interface(struct wilc_vif *vif, u32 frame_size,
			     u32 frame_count)
{
	int i, ret;
	struct wilc *wilc = 0;
	u32 test_frame_size;
	struct tx_complete_data *tx_data = NULL;

	if (!vif) {
		pr_err("Test Failed: vif is null");
		return;
	}

	test_frame_size = ALIGN((frame_size + ETH_ETHERNET_HDR_OFFSET), 4) & 0xFFFF;

	pr_info("TEST::STARTED ->wilc_test_bus_interface %d %d", test_frame_size,
		frame_size);

	wilc = vif->wilc;
	ret = wilc->hif_func->hif_write_reg(wilc,
					    wilc->vmm_ctl.host_vmm_tx_ctl,
					    0x6 | (test_frame_size << 8));
	if (ret) {
		pr_err("fail write reg host_vmm_ctl..");
		return;
	}
	for (i = 0; i < frame_size; i++)
		test_frame[0][i] = i % 0xFF;

	test_in_progress = 0;

	tx_data = kmalloc(sizeof(*tx_data), GFP_ATOMIC);
	if (!tx_data)
		return;

	tx_data->buff = (u8 *)&test_frame[0];
	tx_data->size = frame_size;

	if (!wilc_wlan_txq_add_net_pkt(vif->ndev, tx_data, tx_data->buff, frame_size,
				       wilc_test_frame_tx_complete)) {
		pr_err("Failed to send the test packet");
		return;
	}
	test_run_cnt++;

	// Required only for bulk loopback test
	for (i = 1; i < frame_count; i++) {
		memcpy(&test_frame[i], &test_frame[0][0], frame_size);
		tx_data->buff = (u8 *)&test_frame[i][0];
		tx_data->size = frame_size;

		if (!wilc_wlan_txq_add_net_pkt(vif->ndev, tx_data, tx_data->buff,
					       frame_size,
					       wilc_test_frame_tx_complete)) {
			pr_err("Failed to send the test packet");
			return;
		}
		test_run_cnt++;
	}
	test_in_progress = 1;
}

int vspfunc(u8 *buffer, char *format, ...)
{
	va_list aptr;
	int ret;

	va_start(aptr, format);
	ret = vsprintf(buffer, format, aptr);
	va_end(aptr);
	return ret;
}

void print_hex_string(char *buf, int len)
{
	int i;
	int offset = 0;

	if (len == 0) {
		vspfunc(fmt_str, "<empty string>"); return;
	}

	for (i = 0; i < len; i++) {
		offset += vspfunc(fmt_str + offset, "%02x ",
				  *((unsigned char *)buf + i));

		if ((i & 0x1f) == 31) {
			pr_debug("%s", fmt_str);
			offset = 0;
		}
	}

	if (offset)
		pr_err("%s", fmt_str);
}

static void wilc_check_result(struct wilc *wilc, u8 *buffer, int size,
			      u32 frame_size)
{
	u32 offset = ETH_ETHERNET_HDR_OFFSET;
	u32 i;
	static u32 tested_frame_count;

	while ((offset +  frame_size)  <= size) {
		for (i = 0; i < frame_size; i++)
			if (buffer[offset + i] != test_frame[0][i])
				break;
		if (i == frame_size) {
			pr_info("Packet-%d Received correctly",
				tested_frame_count + 1);
			tested_frame_count++;
		} else {
			pr_err("Packet-%d Failed at offset = %d",
			       tested_frame_count + 1, i);
			print_hex_string((buffer + offset + i), (frame_size - i));
			break;
		}

		offset += ALIGN((frame_size + ETH_ETHERNET_HDR_OFFSET), 4);
	}

	pr_info("TEST: Total RECEIVED frames: %d received size = %d %d",
		tested_frame_count, size, test_run_cnt);

	if (tested_frame_count == WILC_S02_TEST_FRAME_COUNT - 1)
		pr_info("TEST: COMPLETED::Success");
}

void wilc_test_verify_result(struct wilc *wilc, u8 *buffer, int size)
{
	int ret;
	struct wilc_vif *vif = wilc_get_wl_to_vif(wilc);

	if (size > (WILC_S02_TEST_2BLOCK_FRAME_SIZE + ETH_ETHERNET_HDR_OFFSET) + 8) {
		pr_info("BULK LOOPBACK TEST:\n Test Packet Sent to host: size=:%d %d %d",
			WILC_S02_TEST_FRAME_SIZE, size, ETH_CONFIG_PKT_HDR_OFFSET);
		wilc_check_result(wilc, buffer, size, WILC_S02_TEST_FRAME_SIZE);

		//Test2 - block transfer loopback
		wilc_test_bus_interface(vif, WILC_S02_TEST_2BLOCK_FRAME_SIZE, 1);
	} else if (size == ALIGN((WILC_S02_TEST_2BLOCK_FRAME_SIZE + ETH_ETHERNET_HDR_OFFSET), 4)) {
		pr_info("2-BLOCK LOOPBACK TEST:\n Test Packet Sent to host: size= :%d",
			WILC_S02_TEST_2BLOCK_FRAME_SIZE);
		wilc_check_result(wilc, buffer, size,
				  WILC_S02_TEST_2BLOCK_FRAME_SIZE);

		//Test2 - block transfer loopback
		wilc_test_bus_interface(vif, WILC_S02_TEST_1BLOCK_FRAME_SIZE, 1);

	} else if (size == ALIGN((WILC_S02_TEST_1BLOCK_FRAME_SIZE + ETH_ETHERNET_HDR_OFFSET), 4)) {
		pr_info("1-BLOCK LOOPBACK TEST:\n Test Packet Sent to host: size= :%d",
			WILC_S02_TEST_1BLOCK_FRAME_SIZE);
		wilc_check_result(wilc, buffer, size,
				  WILC_S02_TEST_1BLOCK_FRAME_SIZE);

		//Test2 - block transfer loopback
		wilc_test_bus_interface(vif, WILC_S02_TEST_BYTE_FRAME_SIZE, 1);
	} else if (size == ALIGN((WILC_S02_TEST_BYTE_FRAME_SIZE + ETH_ETHERNET_HDR_OFFSET), 4)) {
		pr_info("BYTE LOOPBACK TEST:\n Test Packet Sent to host: size= :%d",
			WILC_S02_TEST_BYTE_FRAME_SIZE);
		wilc_check_result(wilc, buffer, size, WILC_S02_TEST_BYTE_FRAME_SIZE);
	} else {
		/* stop the bus interface test */
		ret = wilc->hif_func->hif_write_reg(wilc,
						    wilc->vmm_ctl.host_vmm_tx_ctl, 0x6);
		if (ret)
			pr_err("fail write reg host_vmm_ctl..");
	}
}
#endif
