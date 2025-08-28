// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments PRUETH hsr/prp Link Redundancy Entity (LRE) Driver.
 *
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com
 */

#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/spinlock_types.h>

#include "icssm_lre_firmware.h"
#include "icssm_prueth.h"
#include "icssm_prueth_lre.h"
#include "icssm_prueth_switch.h"

static void pru2host_mac(u8 *mac)
{
	swap(mac[0], mac[3]);
	swap(mac[1], mac[2]);
	swap(mac[4], mac[5]);
}

static u16 get_hash(u8 *mac, u16 hash_mask)
{
	int j;
	u16 hash;

	for (j = 0, hash = 0; j < ETH_ALEN; j++)
		hash ^= mac[j];
	hash = hash & hash_mask;

	return hash;
}

static void pru_spin_lock(struct node_tbl *nt)
{
	while (1) {
		nt->nt_info->arm_lock = 1;
		if (!nt->nt_info->fw_lock)
			break;
		nt->nt_info->arm_lock = 0;
	}
}

static inline void pru_spin_unlock(struct node_tbl *nt)
{
	nt->nt_info->arm_lock = 0;
}

int icssm_prueth_lre_nt_insert(struct prueth *prueth,
			       u8 *mac, int port, int sv_frame, int proto)
{
	struct nt_queue_t *q = prueth->mac_queue;
	unsigned long flags;
	int ret = LRE_OK;

	/* Will encounter a null mac_queue if we are in the middle of
	 * ndo_close. So check and return. Otherwise a kernel crash is
	 * seen when doing ifdown continuously.
	 */
	if (!q)
		return ret;

	spin_lock_irqsave(&prueth->nt_lock, flags);
	if (q->full) {
		ret = LRE_ERR;
	} else {
		memcpy(q->nt_queue[q->wr_ind].mac, mac, ETH_ALEN);
		q->nt_queue[q->wr_ind].sv_frame = sv_frame;
		q->nt_queue[q->wr_ind].port_id = port;
		q->nt_queue[q->wr_ind].proto = proto;

		q->wr_ind++;
		q->wr_ind &= (PRUETH_MAC_QUEUE_MAX - 1);
		if (q->wr_ind == q->rd_ind)
			q->full = true;
	}
	spin_unlock_irqrestore(&prueth->nt_lock, flags);

	return ret;
}

static inline bool node_expired(struct node_tbl *nt, u16 node, u16 forget_time)
{
	struct node_tbl_t nt_node = nt->nt_array->node_tbl[node];

	return ((nt_node.time_last_seen_s > forget_time ||
		 nt_node.status & ICSS_LRE_NT_REM_NODE_TYPE_SANAB) &&
		 nt_node.time_last_seen_a > forget_time &&
		 nt_node.time_last_seen_b > forget_time);
}

#define IND_BIN_NO(x)		nt->index_array->index_tbl[x].bin_no_entries
#define IND_BINOFS(x)		nt->index_array->index_tbl[x].bin_offset
#define BIN_NODEOFS(x)		nt->bin_array->bin_tbl[x].node_tbl_offset

static void _icssm_prueth_lre_init_node_table(struct prueth *prueth)
{
	struct nt_queue_t *q = prueth->mac_queue;
	struct node_tbl *nt = prueth->nt;
	int j;

	const struct prueth_fw_offsets *fw_offsets = prueth->fw_offsets;

	nt->nt_array = (struct nt_array_t *)((__force const void *)
			prueth->mem[fw_offsets->nt_array_loc].va +
			fw_offsets->nt_array_offset);
	memset_io((void __iomem *)nt->nt_array, 0, sizeof(struct node_tbl_t) *
		  fw_offsets->nt_array_max_entries);

	nt->bin_array = (struct bin_array_t *)((__force const void *)
			 prueth->mem[fw_offsets->bin_array_loc].va +
			 fw_offsets->bin_array_offset);
	memset_io((void __iomem *)nt->bin_array, 0, sizeof(struct bin_tbl_t) *
		  fw_offsets->bin_array_max_entries);

	nt->index_array = (struct index_array_t *)((__force const void *)
			   prueth->mem[fw_offsets->index_array_loc].va +
			   fw_offsets->index_array_offset);
	memset_io((void __iomem *)nt->index_array, 0,
		  sizeof(struct node_index_tbl_t) *
		  fw_offsets->index_array_max_entries);

	nt->nt_info = (struct node_tbl_info_t *)((__force const void *)
		       prueth->mem[fw_offsets->nt_array_loc].va +
		       fw_offsets->nt_array_offset +
		       (sizeof(struct node_tbl_t) *
		       fw_offsets->nt_array_max_entries));
	memset_io((void __iomem *)nt->nt_info, 0,
		  sizeof(struct node_tbl_info_t));

	nt->nt_lre_cnt =
		(struct node_tbl_lre_cnt_t *)((__force const void *)
		 prueth->mem[PRUETH_MEM_SHARED_RAM].va + ICSS_LRE_CNT_NODES);
	memset_io((void __iomem *)nt->nt_lre_cnt, 0,
		  sizeof(struct node_tbl_lre_cnt_t));

	nt->nt_array_max_entries = fw_offsets->nt_array_max_entries;
	nt->bin_array_max_entries = fw_offsets->bin_array_max_entries;
	nt->index_array_max_entries = fw_offsets->index_array_max_entries;
	nt->hash_mask = fw_offsets->hash_mask;

	for (j = 0; j < fw_offsets->index_array_max_entries; j++)
		IND_BINOFS(j) = fw_offsets->bin_array_max_entries;
	for (j = 0; j < fw_offsets->bin_array_max_entries; j++)
		BIN_NODEOFS(j) = fw_offsets->nt_array_max_entries;
	for (j = 0; j < fw_offsets->nt_array_max_entries; j++)
		nt->nt_array->node_tbl[j].entry_state = ICSS_LRE_NODE_FREE;

	q->rd_ind = 0;
	q->wr_ind = 0;
	q->full = false;
}

static u16 find_free_bin(struct node_tbl *nt)
{
	u16 j;

	for (j = 0; j < nt->bin_array_max_entries; j++)
		if (BIN_NODEOFS(j) == nt->nt_array_max_entries)
			break;

	return j;
}

/* find first free node table slot and write it to the next_free_slot */
static u16 next_free_slot_update(struct node_tbl *nt)
{
	int j;

	nt->nt_info->next_free_slot = nt->nt_array_max_entries;
	for (j = 0; j < nt->nt_array_max_entries; j++) {
		if (nt->nt_array->node_tbl[j].entry_state ==
				ICSS_LRE_NODE_FREE) {
			nt->nt_info->next_free_slot = j;
			break;
		}
	}

	return nt->nt_info->next_free_slot;
}

static void inc_time(u16 *t)
{
	*t += 1;
	if (*t > ICSS_LRE_MAX_FORGET_TIME)
		*t = ICSS_LRE_MAX_FORGET_TIME;
}

static void node_table_update_time(struct node_tbl *nt)
{
	int j;
	u16 ofs;
	struct nt_array_t *nt_arr = nt->nt_array;
	struct node_tbl_t *node;

	for (j = 0; j < nt->bin_array_max_entries; j++) {
		ofs = nt->bin_array->bin_tbl[j].node_tbl_offset;
		if (ofs < nt->nt_array_max_entries) {
			node = &nt_arr->node_tbl[ofs];
			inc_time(&node->time_last_seen_a);
			inc_time(&node->time_last_seen_b);
			/* increment time_last_seen_s if nod is not SAN */
			if ((node->status &
			     ICSS_LRE_NT_REM_NODE_TYPE_SANAB) == 0)
				inc_time(&node->time_last_seen_s);
		}
	}
}

static void write2node_slot(struct node_tbl *nt, u16 node, int port,
			    int sv_frame, int proto)
{
	memset(&nt->nt_array->node_tbl[node], 0, sizeof(struct node_tbl_t));
	nt->nt_array->node_tbl[node].entry_state = ICSS_LRE_NODE_TAKEN;

	if (port == 0x01) {
		nt->nt_array->node_tbl[node].status =
			ICSS_LRE_NT_REM_NODE_TYPE_SANA;
		nt->nt_array->node_tbl[node].cnt_ra = 1;
		if (sv_frame)
			nt->nt_array->node_tbl[node].cnt_rx_sup_a = 1;
	} else {
		nt->nt_array->node_tbl[node].status =
			ICSS_LRE_NT_REM_NODE_TYPE_SANB;
		nt->nt_array->node_tbl[node].cnt_rb = 1;
		if (sv_frame)
			nt->nt_array->node_tbl[node].cnt_rx_sup_b = 1;
	}

	if (sv_frame) {
		nt->nt_array->node_tbl[node].status = (proto == LRE_PROTO_PRP) ?
			ICSS_LRE_NT_REM_NODE_TYPE_DAN :
			ICSS_LRE_NT_REM_NODE_TYPE_DAN |
			ICSS_LRE_NT_REM_NODE_HSR_BIT;
	}
}

/* We assume that the _start_ cannot point to middle of a bin */
static void update_indexes(u16 start, u16 end, struct node_tbl *nt)
{
	u16 hash, hash_prev;

	hash_prev = 0xffff; /* invalid hash */
	for (; start <= end; start++) {
		hash = get_hash(nt->bin_array->bin_tbl[start].src_mac_id,
				nt->hash_mask);
		if (hash != hash_prev)
			IND_BINOFS(hash) = start;
		hash_prev = hash;
	}
}

/* start > end */
static void move_up(u16 start, u16 end, struct node_tbl *nt,
		    bool update)
{
	u16 j = end;

	pru_spin_lock(nt);

	for (; j < start; j++)
		memcpy(&nt->bin_array->bin_tbl[j],
		       &nt->bin_array->bin_tbl[j + 1],
		       sizeof(struct bin_tbl_t));

	BIN_NODEOFS(start) = nt->nt_array_max_entries;

	if (update)
		update_indexes(end, start + 1, nt);

	pru_spin_unlock(nt);
}

/* start < end */
static void move_down(u16 start, u16 end, struct node_tbl *nt,
		      bool update)
{
	u16 j = end;

	pru_spin_lock(nt);

	for (; j > start; j--)
		memcpy(&nt->bin_array->bin_tbl[j],
		       &nt->bin_array->bin_tbl[j - 1],
		       sizeof(struct bin_tbl_t));

	nt->bin_array->bin_tbl[start].node_tbl_offset =
					nt->nt_array_max_entries;

	if (update)
		update_indexes(start + 1, end, nt);

	pru_spin_unlock(nt);
}

static int node_table_insert_from_queue(struct node_tbl *nt,
					struct nt_queue_entry *entry)
{
	u8 macid[ETH_ALEN];
	u16 hash;
	u16 index;
	u16 free_node;
	bool not_found;
	u16 empty_slot;

	if (!nt)
		return LRE_ERR;

	memcpy(macid, entry->mac, ETH_ALEN);
	pru2host_mac(macid);

	hash = get_hash(macid, nt->hash_mask);

	not_found = 1;
	if (IND_BIN_NO(hash) == 0) {
		/* there is no bin for this hash, create one */
		index = find_free_bin(nt);
		if (index == nt->bin_array_max_entries)
			return LRE_ERR;

		IND_BINOFS(hash) = index;
	} else {
		for (index = IND_BINOFS(hash);
		     index < IND_BINOFS(hash) + IND_BIN_NO(hash); index++) {
			if ((memcmp(nt->bin_array->bin_tbl[index].src_mac_id,
				    macid, ETH_ALEN) == 0)) {
				not_found = 0;
				break;
			}
		}
	}

	if (not_found) {
		free_node = next_free_slot_update(nt);

		/* at this point we might create a new bin and set
		 * bin_offset at the index table. It was only possible
		 * if we found a free slot in the bin table.
		 * So, it also must be a free slot in the node table
		 * and we will not exit here in this case.
		 * So, be don't have to take care about fixing IND_BINOFS()
		 * on return LRE_ERR
		 */
		if (free_node >= nt->nt_array_max_entries)
			return LRE_ERR;

		/* if we are here, we have at least one empty slot in the bin
		 * table and one slot at the node table
		 */

		IND_BIN_NO(hash)++;

		/* look for an empty slot downwards */
		for (empty_slot = index;
		     (BIN_NODEOFS(empty_slot) != nt->nt_array_max_entries) &&
		     (empty_slot < nt->nt_array_max_entries);
		     empty_slot++)
			;

		/* if emptySlot != maxNodes => empty slot is found,
		 * else no space available downwards, look upwards
		 */
		if (empty_slot != nt->nt_array_max_entries) {
			move_down(index, empty_slot, nt, true);
		} else {
			for (empty_slot = index - 1;
			     (BIN_NODEOFS(empty_slot) !=
			     nt->nt_array_max_entries) &&
			     (empty_slot > 0);
			     empty_slot--)
				;
			/* we're sure to get a space here as nodetable
			 * has a empty slot, so no need to check for
			 * value of emptySlot
			 */
			move_up(index, empty_slot, nt, true);
		}

		/* space created, now populate the values*/
		BIN_NODEOFS(index) = free_node;
		memcpy(nt->bin_array->bin_tbl[index].src_mac_id, macid,
		       ETH_ALEN);
		write2node_slot(nt, free_node, entry->port_id, entry->sv_frame,
				entry->proto);

		nt->nt_lre_cnt->lre_cnt++;
	}

	return LRE_OK;
}

static void node_table_check_and_remove(struct node_tbl *nt, u16 forget_time)
{
	int j, end_bin;
	u16 node;
	u16 hash;

	/*loop to remove a node reaching NODE_FORGET_TIME*/
	for (j = 0; j < nt->bin_array_max_entries; j++) {
		node = BIN_NODEOFS(j);
		if (node >= nt->nt_array_max_entries)
			continue;

		if (node_expired(nt, node, forget_time)) {
			hash = get_hash(nt->bin_array->bin_tbl[j].src_mac_id,
					nt->hash_mask);

			/* remove entry from bin array */
			end_bin = IND_BINOFS(hash) + IND_BIN_NO(hash) - 1;

			move_up(end_bin, j, nt, false);
			(IND_BIN_NO(hash))--;

			if (!IND_BIN_NO(hash))
				IND_BINOFS(hash) = nt->bin_array_max_entries;

			nt->nt_array->node_tbl[node].entry_state =
							ICSS_LRE_NODE_FREE;
			BIN_NODEOFS(end_bin) = nt->nt_array_max_entries;

			nt->nt_lre_cnt->lre_cnt--;
		}
	}
}

static int pop_queue(struct prueth *prueth, spinlock_t *lock)
{
	unsigned long flags;
	struct node_tbl *nt = prueth->nt;
	struct nt_queue_t *q = prueth->mac_queue;
	struct nt_queue_entry one_mac;
	int ret = 0;

	spin_lock_irqsave(lock, flags);
	if (!q->full && q->wr_ind == q->rd_ind) { /* queue empty */
		ret = 1;
	} else {
		memcpy(&one_mac, &q->nt_queue[q->rd_ind],
		       sizeof(struct nt_queue_entry));
		spin_unlock_irqrestore(lock, flags);
		node_table_insert_from_queue(nt, &one_mac);
		spin_lock_irqsave(lock, flags);
		q->rd_ind++;
		q->rd_ind &= (PRUETH_MAC_QUEUE_MAX - 1);
		q->full = false;
	}
	spin_unlock_irqrestore(lock, flags);

	return ret;
}

static void pop_queue_process(struct prueth *prueth, spinlock_t *lock)
{
	while (pop_queue(prueth, lock) == 0)
		;
}

static void nt_updater(struct kthread_work *work)
{
	struct prueth *prueth = container_of(work, struct prueth, nt_work);

	pop_queue_process(prueth, &prueth->nt_lock);

	node_table_update_time(prueth->nt);
	if (++prueth->rem_cnt >= 100) {
		node_table_check_and_remove(prueth->nt,
					    ICSS_LRE_NODE_FORGET_TIME_60000_MS);
		prueth->rem_cnt = 0;
	}
}

int icssm_prueth_lre_init_node_table(struct prueth *prueth)
{
	/* HSR/PRP: initialize node table when first port is up */
	if (prueth->emac_configured)
		return 0;

	/* initialize for node table handling in driver for HSR/PRP */
	prueth->mac_queue = kmalloc(sizeof(*prueth->mac_queue), GFP_KERNEL);
	prueth->nt = kmalloc(sizeof(*prueth->nt), GFP_KERNEL);
	if (!prueth->mac_queue || !prueth->nt) {
		kfree(prueth->mac_queue);
		kfree(prueth->nt);
		prueth->mac_queue = NULL;
		prueth->nt = NULL;
		return -ENOMEM;
	}

	_icssm_prueth_lre_init_node_table(prueth);
	spin_lock_init(&prueth->nt_lock);
	kthread_init_work(&prueth->nt_work, nt_updater);
	prueth->nt_kworker = kthread_create_worker(0, "prueth_nt");

	return 0;
}
