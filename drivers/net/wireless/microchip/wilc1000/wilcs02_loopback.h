#ifndef WILC_S02_LOOP_BACK_H
#define WILC_S02_LOOP_BACK_H

#define WILC_S02_TEST_BUS_INTERFACE

//ETH_ETHERNET_HDR_OFFSET ~ 34

#define WILC_S02_TEST_FRAME_SIZE  (1580 - ETH_ETHERNET_HDR_OFFSET)
#define WILC_S02_TEST_2BLOCK_FRAME_SIZE  (1024 - ETH_ETHERNET_HDR_OFFSET)
#define WILC_S02_TEST_1BLOCK_FRAME_SIZE  (512 - ETH_ETHERNET_HDR_OFFSET)
#define WILC_S02_TEST_BYTE_FRAME_SIZE  (500 - ETH_ETHERNET_HDR_OFFSET)

#define  WILC_S02_TEST_FRAME_COUNT 5

extern u8 test_frame[WILC_S02_TEST_FRAME_COUNT][WILC_S02_TEST_FRAME_SIZE];
extern u32 test_in_progress;

void wilc_test_bus_interface(struct wilc_vif *vif, u32 frame_size, u32
			     frame_count);
void wilc_test_verify_result(struct wilc *wilc, u8 *buffer, int size);
#endif
