/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2026 Texas Instruments Incorporated - https://www.ti.com/
 *
 * This header defines the RPMSG message structures exchanged between
 * the Linux kernel (host) and the C7x DSP (remote) firmware for the
 * Thames DRM/accel driver.
 */

#ifndef _THAMES_IPC_H
#define _THAMES_IPC_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
typedef uint8_t __u8;
typedef uint16_t __u16;
typedef uint32_t __u32;
typedef uint64_t __u64;
#endif

#define THAMES_SERVICE_NAME "thames-service"

/**
 * @THAMES_MSG_TYPE: Simplified message type enumeration
 */
enum thames_msg_type {
	/* --- Host (Kernel) -> Remote (DSP) --- */
	THAMES_MSG_PING = 0x100, /* Ping message to test communication */
	THAMES_MSG_CONTEXT_OP, /* Create/destroy context */
	THAMES_MSG_BO_OP, /* Map/unmap buffer objects */
	THAMES_MSG_SUBMIT_JOB, /* Submit job for execution */

	/* --- Remote (DSP) -> Host (Kernel) --- */
	THAMES_MSG_PING_RESPONSE = 0x200,
	THAMES_MSG_CONTEXT_OP_RESPONSE,
	THAMES_MSG_BO_OP_RESPONSE,
	THAMES_MSG_SUBMIT_JOB_RESPONSE,
};

/**
 * @THAMES_CONTEXT_OP: Context operation types
 */
enum thames_context_op {
	THAMES_CONTEXT_CREATE = 0,
	THAMES_CONTEXT_DESTROY,
};

/**
 * @THAMES_BO_OP: Buffer Object operation types
 */
enum thames_bo_op {
	THAMES_BO_MAP = 0,
	THAMES_BO_UNMAP,
};

/**
 * @THAMES_RESP_STATUS: Response status codes
 */
enum thames_resp_status {
	THAMES_RESP_SUCCESS = 0,
	THAMES_RESP_ERR_GENERIC = 1,
	THAMES_RESP_ERR_NOMEM = 2,
	THAMES_RESP_ERR_INVAL = 3,
	THAMES_RESP_ERR_NO_CTX = 4,
	THAMES_RESP_ERR_MMU = 5,
	THAMES_RESP_ERR_JOB_TIMEOUT = 6,
};

/**
 * struct thames_msg_hdr - Common header for all RPMSG messages
 * @type: Message type from enum thames_msg_type
 * @seq:  Sequence number for request/response matching
 * @len:  Total message length including header
 */
struct thames_msg_hdr {
	__u32 type;
	__u32 seq;
	__u32 len;
	__u32 reserved;
};

/*
 * ===================================================================
 * Host (Kernel) -> Remote (DSP) Messages
 * ===================================================================
 */

/**
 * struct thames_msg_ping - Ping message to test communication
 * @hdr:        Common message header
 * @ping_data:  Optional ping data (timestamp, sequence, etc.)
 */
struct thames_msg_ping {
	struct thames_msg_hdr hdr;
	__u32 ping_data;
};

/**
 * struct thames_msg_context_op - Context create/destroy operations
 * @hdr:           Common message header
 * @op:            Operation type (CREATE/DESTROY)
 * @context_id:    Context ID
 */
struct thames_msg_context_op {
	struct thames_msg_hdr hdr;
	uint32_t op; /* enum thames_context_op */
	uint32_t context_id;
};

/**
 * struct thames_msg_bo_op - Buffer Object map/unmap operations
 * @hdr:        Common message header
 * @op:         Operation type (MAP/UNMAP)
 * @context_id: Context ID that this BO belongs to
 * @bo_id:      Buffer Object ID for tracking
 * @vaddr:      Virtual address where BO should be mapped on DSP
 * @paddr:      Physical address of the BO
 * @size:       Size of the BO in bytes
 */
struct thames_msg_bo_op {
	struct thames_msg_hdr hdr;
	uint32_t op; /* enum thames_bo_op */
	uint32_t context_id;
	uint32_t bo_id;
	uint64_t vaddr;
	uint64_t paddr;
	uint64_t size;
};

/**
 * struct thames_msg_submit_job - Submit job for execution
 * @hdr:         Common message header
 * @context_id:  Context to run job in
 * @job_id:      Host-generated job tracking ID
 * @kernel_iova: IOVA of kernel code BO (first byte = first instruction)
 * @kernel_size: Size of kernel code in bytes
 * @args_iova:   IOVA of arguments BO (array of uint64_t values)
 * @args_size:   Size of arguments BO in bytes
 */
struct thames_msg_submit_job {
	struct thames_msg_hdr hdr;
	uint32_t context_id;
	uint32_t job_id;
	uint64_t kernel_iova;
	uint64_t kernel_size;
	uint64_t args_iova;
	uint64_t args_size;
};

/*
 * ===================================================================
 * Remote (DSP) -> Host (Kernel) Messages
 * ===================================================================
 */

/**
 * struct thames_msg_response - Generic response to commands
 * @hdr:    Common message header (seq matches request)
 * @status: Status code from enum thames_resp_status
 * @data:   Optional response data (context-dependent)
 */
struct thames_msg_response {
	struct thames_msg_hdr hdr;
	uint32_t status;
	uint32_t data;
};

/*
 * ===================================================================
 * Buffer Size Calculations
 * ===================================================================
 */

/* Calculate the maximum message size by finding the largest structure */
#define THAMES_MSG_SIZE_PING sizeof(struct thames_msg_ping)
#define THAMES_MSG_SIZE_CONTEXT_OP sizeof(struct thames_msg_context_op)
#define THAMES_MSG_SIZE_BO_OP sizeof(struct thames_msg_bo_op)
#define THAMES_MSG_SIZE_SUBMIT_JOB sizeof(struct thames_msg_submit_job)
#define THAMES_MSG_SIZE_RESPONSE sizeof(struct thames_msg_response)

/* Helper macros to find maximum of multiple values */
#define THAMES_MAX2(a, b) ((a) > (b) ? (a) : (b))
#define THAMES_MAX3(a, b, c) THAMES_MAX2(THAMES_MAX2(a, b), c)
#define THAMES_MAX5(a, b, c, d, e) THAMES_MAX2(THAMES_MAX3(a, b, c), THAMES_MAX2(d, e))

/* Maximum size of any Thames IPC message */
#define THAMES_IPC_MAX_MSG_SIZE                                                              \
	THAMES_MAX5(THAMES_MSG_SIZE_PING, THAMES_MSG_SIZE_CONTEXT_OP, THAMES_MSG_SIZE_BO_OP, \
		    THAMES_MSG_SIZE_SUBMIT_JOB, THAMES_MSG_SIZE_RESPONSE)

/* RPMSG buffer size - should accommodate largest message + some padding */
#define THAMES_RPMSG_BUFFER_SIZE ((THAMES_IPC_MAX_MSG_SIZE + 15) & ~15) /* 16-byte aligned */

/* Compile-time size checks - use BUILD_BUG_ON in kernel code */
#ifdef __KERNEL__
#define THAMES_ASSERT_MSG_SIZE(msg_type) BUILD_BUG_ON(sizeof(struct msg_type) > 64)
#else
#define THAMES_ASSERT_MSG_SIZE(msg_type) \
	_Static_assert(sizeof(struct msg_type) <= 64, #msg_type " too large")
#endif

#endif /* _THAMES_IPC_H */
