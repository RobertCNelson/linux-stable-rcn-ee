// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments PRUETH Switch Driver
 *
 * Copyright (C) 2020-2021 Texas Instruments Incorporated - https://www.ti.com
 */
#include <linux/etherdevice.h>
#include <linux/kernel.h>
#include <linux/remoteproc.h>
#include <net/switchdev.h>
#include "icssm_prueth.h"
#include "icssm_prueth_switch.h"
#include "icssm_prueth_fdb_tbl.h"

#define FDB_IDX_TBL_ENTRY(n) \
	(&prueth->fdb_tbl->index_a->index_tbl_entry[n])

#define FDB_MAC_TBL_ENTRY(n) \
	(&prueth->fdb_tbl->mac_tbl_a->mac_tbl_entry[n])

#define FDB_LEARN  1
#define FDB_PURGE  3

struct icssm_prueth_sw_fdb_work {
	struct work_struct work;
	struct prueth_emac *emac;
	u8 addr[ETH_ALEN];
	int event;
};

void icssm_prueth_sw_free_fdb_table(struct prueth *prueth)
{
	if (prueth->emac_configured)
		return;

	kfree(prueth->fdb_tbl);
	prueth->fdb_tbl = NULL;
}

void icssm_prueth_sw_fdb_tbl_init(struct prueth *prueth)
{
	struct fdb_tbl *t = prueth->fdb_tbl;

	t->index_a = (struct fdb_index_array_t *)((__force const void *)
			prueth->mem[V2_1_FDB_TBL_LOC].va +
			V2_1_FDB_TBL_OFFSET);
	t->mac_tbl_a =
		(struct fdb_mac_tbl_array_t *)((__force const void *)
				t->index_a + FDB_INDEX_TBL_MAX_ENTRIES *
				sizeof(struct fdb_index_tbl_entry_t));
	t->port1_stp_cfg = (struct fdb_stp_config *)((__force const void *)
			t->mac_tbl_a + FDB_MAC_TBL_MAX_ENTRIES *
			sizeof(struct fdb_mac_tbl_entry_t));
	t->port2_stp_cfg = (struct fdb_stp_config *)((__force const void *)
			t->port1_stp_cfg + sizeof(struct fdb_stp_config));
	t->flood_enable_flags =
		(struct fdb_flood_config *)((__force const void *)
				t->port2_stp_cfg +
				sizeof(struct fdb_stp_config));
	t->locks = (struct fdb_arbitration *)((__force const void *)
			t->flood_enable_flags +
			sizeof(struct fdb_flood_config));

	t->flood_enable_flags->host_flood_enable  = 1;
	t->flood_enable_flags->port1_flood_enable = 1;
	t->flood_enable_flags->port2_flood_enable = 1;
	t->locks->host_lock                       = 0;
	t->total_entries                          = 0;
}

static void icssm_prueth_sw_fdb_spin_lock(struct fdb_tbl *fdb_tbl)
{
	/* Take the host lock */
	writeb(1, (u8 __iomem *)&fdb_tbl->locks->host_lock);

	/* Wait for the PRUs to release their locks */
	while (readb((u8 __iomem *)&fdb_tbl->locks->pru_locks))
		;
}

static inline void icssm_prueth_sw_fdb_spin_unlock(struct fdb_tbl *fdb_tbl)
{
	writeb(0, (u8 __iomem *)&fdb_tbl->locks->host_lock);
}

static void icssm_mac_copy(u8 *dst, const u8 *src)
{
	u8 i;

	for (i = 0; i < 6; i++) {
		*(dst) = *(src);
		dst++;
		src++;
	}
}

/* -1  mac_a <  mac_b
 *  0  mac_a == mac_b
 *  1  mac_a >  mac_b
 */
static s8 icssm_mac_cmp(const u8 *mac_a, const u8 *mac_b)
{
	s8  ret = 0, i;

	for (i = 0; i < 6; i++) {
		if (mac_a[i] == mac_b[i])
			continue;

		ret = mac_a[i] < mac_b[i] ? -1 : 1;
		break;
	}

	return ret;
}

static inline u8 icssm_prueth_sw_fdb_hash(const u8 *mac)
{
	return mac[0] ^ mac[1] ^ mac[2] ^ mac[3] ^ mac[4] ^ mac[5];
}

static s16
icssm_prueth_sw_fdb_search(struct fdb_mac_tbl_array_t *mac_tbl,
			   struct fdb_index_tbl_entry_t *bucket_info,
			   const u8 *mac)
{
	u8 mac_tbl_idx = bucket_info->bucket_idx;
	int i;

	for (i = 0; i < bucket_info->bucket_entries; i++, mac_tbl_idx++) {
		if (!icssm_mac_cmp(mac,
				   mac_tbl->mac_tbl_entry[mac_tbl_idx].mac))
			return mac_tbl_idx;
	}

	return -ENODATA;
}

static u16 icssm_prueth_sw_fdb_find_open_slot(struct fdb_tbl *fdb_tbl)
{
	u16 i;

	for (i = 0; i < FDB_MAC_TBL_MAX_ENTRIES; i++) {
		if (!fdb_tbl->mac_tbl_a->mac_tbl_entry[i].active)
			break;
	}

	return i;
}

/* port: 0 based: 0=port1, 1=port2 */
static s16
icssm_prueth_sw_fdb_find_bucket_insert_point(struct fdb_tbl *fdb,
					     struct fdb_index_tbl_entry_t
					     *bkt_info,
					     const u8 *mac, const u8 port)
{
	struct fdb_mac_tbl_array_t *mac_tbl = fdb->mac_tbl_a;
	struct fdb_mac_tbl_entry_t *e;
	u8 mac_tbl_idx;
	s8 cmp;
	int i;

	mac_tbl_idx = bkt_info->bucket_idx;

	for (i = 0; i < bkt_info->bucket_entries; i++, mac_tbl_idx++) {
		e = &mac_tbl->mac_tbl_entry[mac_tbl_idx];
		cmp = icssm_mac_cmp(mac, e->mac);
		if (cmp < 0) {
			return mac_tbl_idx;
		} else if (cmp == 0) {
			if (e->port != port) {
				/* mac is already in FDB, only port is
				 * different. So just update the port.
				 * Note: total_entries and bucket_entries
				 * remain the same.
				 */
				icssm_prueth_sw_fdb_spin_lock(fdb);
				e->port = port;
				icssm_prueth_sw_fdb_spin_unlock(fdb);
			}

			/* mac and port are the same, touch the fdb */
			e->age = 0;
			return -1;
		}
	}

	return mac_tbl_idx;
}

static s16
icssm_prueth_sw_fdb_check_empty_slot_left(struct fdb_mac_tbl_array_t *mac_tbl,
					  u8 mac_tbl_idx)
{
	s16 i;

	for (i = mac_tbl_idx - 1; i > -1; i--) {
		if (!mac_tbl->mac_tbl_entry[i].active)
			break;
	}

	return i;
}

static s16
icssm_prueth_sw_fdb_check_empty_slot_right(struct fdb_mac_tbl_array_t *mac_tbl,
					   u8 mac_tbl_idx)
{
	s16 i;

	for (i = mac_tbl_idx; i < FDB_MAC_TBL_MAX_ENTRIES; i++) {
		if (!mac_tbl->mac_tbl_entry[i].active)
			return i;
	}

	return -1;
}

static void icssm_prueth_sw_fdb_move_range_left(struct prueth *prueth,
						u16 left, u16 right)
{
	u8 *src, *dst;
	u32 sz = 0;
	u16 i;

	for (i = left; i < right; i++) {
		dst = (u8 *)FDB_MAC_TBL_ENTRY(i);
		src = (u8 *)FDB_MAC_TBL_ENTRY(i + 1);
		sz = sizeof(struct fdb_mac_tbl_entry_t);
		memcpy_toio((void __iomem *)dst, src, sz);
	}
}

static void icssm_prueth_sw_fdb_move_range_right(struct prueth *prueth,
						 u16 left, u16 right)
{
	u8 *src, *dst;
	u32 sz = 0;
	u16 i;

	for (i = right; i > left; i--) {
		dst = (u8 *)FDB_MAC_TBL_ENTRY(i);
		src = (u8 *)FDB_MAC_TBL_ENTRY(i - 1);
		sz = sizeof(struct fdb_mac_tbl_entry_t);
		memcpy_toio((void __iomem *)dst, src, sz);
	}
}

static void icssm_prueth_sw_fdb_update_index_tbl(struct prueth *prueth,
						 u16 left, u16 right)
{
	u8 hash, hash_prev;
	u16 i;

	/* To ensure we don't improperly update the
	 * bucket index, initialize with an invalid
	 * hash in case we are in leftmost slot
	 */
	hash_prev = 0xff;

	if (left > 0) {
		hash_prev =
			icssm_prueth_sw_fdb_hash
			(FDB_MAC_TBL_ENTRY(left - 1)->mac);
	}

	/* For each moved element, update the bucket index */
	for (i = left; i <= right; i++) {
		hash = icssm_prueth_sw_fdb_hash(FDB_MAC_TBL_ENTRY(i)->mac);

		/* Only need to update buckets once */
		if (hash != hash_prev)
			FDB_IDX_TBL_ENTRY(hash)->bucket_idx = i;

		hash_prev = hash;
	}
}

static struct fdb_mac_tbl_entry_t *
icssm_prueth_sw_get_empty_mac_tbl_entry(struct prueth *prueth,
					struct fdb_index_tbl_entry_t
					*bucket_info,
					u8 suggested_mac_tbl_idx,
					bool *update_indexes,
					const u8 *mac)
{
	s16 empty_slot_idx = 0, left = 0, right = 0;
	u8 mti = suggested_mac_tbl_idx;
	struct fdb_mac_tbl_array_t *mt;
	struct fdb_tbl *fdb;

	fdb = prueth->fdb_tbl;
	mt = fdb->mac_tbl_a;

	if (!FDB_MAC_TBL_ENTRY(mti)->active) {
		/* Claim the entry */
		FDB_MAC_TBL_ENTRY(mti)->active = 1;

		return FDB_MAC_TBL_ENTRY(mti);
	}

	if (fdb->total_entries == FDB_MAC_TBL_MAX_ENTRIES)
		return NULL;

	empty_slot_idx =
		icssm_prueth_sw_fdb_check_empty_slot_left(mt, mti);
	if (empty_slot_idx == -1) {
		/* Nothing available on the left. But table isn't full
		 * so there must be space to the right,
		 */
		empty_slot_idx =
			icssm_prueth_sw_fdb_check_empty_slot_right(mt, mti);

		/* Shift right */
		left = mti;
		right = empty_slot_idx;
		icssm_prueth_sw_fdb_move_range_right(prueth, left, right);

		/* Claim the entry */
		FDB_MAC_TBL_ENTRY(mti)->active = 1;

		icssm_mac_copy(FDB_MAC_TBL_ENTRY(mti)->mac, mac);

		/* There is a chance we moved something in a
		 * different bucket, update index table
		 */
		icssm_prueth_sw_fdb_update_index_tbl(prueth, left, right);

		return FDB_MAC_TBL_ENTRY(mti);
	}

	if (empty_slot_idx == mti - 1) {
		/* There is space immediately left of the open slot,
		 * which means the inserted MAC address.
		 * Must be the lowest-valued MAC address in bucket.
		 * Update bucket pointer accordingly.
		 */
		bucket_info->bucket_idx = empty_slot_idx;

		/* Claim the entry */
		FDB_MAC_TBL_ENTRY(empty_slot_idx)->active = 1;

		return FDB_MAC_TBL_ENTRY(empty_slot_idx);
	}

	/* There is empty space to the left, shift MAC table entries left */
	left = empty_slot_idx;
	right = mti - 1;
	icssm_prueth_sw_fdb_move_range_left(prueth, left, right);

	/* Claim the entry */
	FDB_MAC_TBL_ENTRY(mti - 1)->active = 1;

	icssm_mac_copy(FDB_MAC_TBL_ENTRY(mti - 1)->mac, mac);

	/* There is a chance we moved something in a
	 * different bucket, update index table
	 */
	icssm_prueth_sw_fdb_update_index_tbl(prueth, left, right);

	return FDB_MAC_TBL_ENTRY(mti - 1);
}

static int icssm_prueth_sw_insert_fdb_entry(struct prueth_emac *emac,
					    const u8 *mac, u8 is_static)
{
	struct fdb_index_tbl_entry_t *bucket_info;
	struct fdb_mac_tbl_entry_t *mac_info;
	struct prueth *prueth = emac->prueth;
	struct prueth_emac *other_emac;
	enum prueth_port other_port_id;
	u8 hash_val, mac_tbl_idx;
	struct fdb_tbl *fdb;
	s16 ret;

	fdb = prueth->fdb_tbl;
	other_port_id =
		(emac->port_id == PRUETH_PORT_MII0) ? PRUETH_PORT_MII1 :
						PRUETH_PORT_MII0;

	other_emac = prueth->emac[other_port_id - 1];

	if (fdb->total_entries == FDB_MAC_TBL_MAX_ENTRIES)
		return -ENOMEM;

	if (icssm_mac_cmp(mac, emac->mac_addr) == 0 ||
	    icssm_mac_cmp(mac, other_emac->mac_addr) == 0) {
		/* Don't insert fdb of own mac addr */
		return -EINVAL;
	}

	/* Empty mac table entries are available */

	/* Get the bucket that the mac belongs to */
	hash_val = icssm_prueth_sw_fdb_hash(mac);
	bucket_info = FDB_IDX_TBL_ENTRY(hash_val);

	if (!bucket_info->bucket_entries) {
		mac_tbl_idx = icssm_prueth_sw_fdb_find_open_slot(fdb);
		bucket_info->bucket_idx = mac_tbl_idx;
	}

	ret = icssm_prueth_sw_fdb_find_bucket_insert_point(fdb,
							   bucket_info, mac,
							   emac->port_id - 1);

	if (ret < 0)
		/* mac is already in fdb table */
		return 0;

	mac_tbl_idx = ret;

	icssm_prueth_sw_fdb_spin_lock(fdb);

	mac_info = icssm_prueth_sw_get_empty_mac_tbl_entry(prueth, bucket_info,
							   mac_tbl_idx, NULL,
							   mac);
	if (!mac_info) {
		/* Should not happen */
		dev_warn(prueth->dev, "OUT of MEM\n");
		return -ENOMEM;
	}

	icssm_mac_copy(mac_info->mac, mac);
	mac_info->active = 1;
	mac_info->age = 0;
	mac_info->port = emac->port_id - 1;
	mac_info->is_static = is_static;

	bucket_info->bucket_entries++;
	fdb->total_entries++;

	icssm_prueth_sw_fdb_spin_unlock(fdb);

	dev_dbg(prueth->dev, "added fdb: %pM port=%d total_entries=%u\n",
		mac, emac->port_id, fdb->total_entries);

	return 0;
}

static int icssm_prueth_sw_delete_fdb_entry(struct prueth_emac *emac,
					    const u8 *mac, u8 is_static)
{
	struct fdb_index_tbl_entry_t *bucket_info;
	struct fdb_mac_tbl_entry_t *mac_info;
	struct fdb_mac_tbl_array_t *mt;
	u8 hash_val, mac_tbl_idx;
	struct prueth *prueth;
	s16 ret, left, right;
	struct fdb_tbl *fdb;

	prueth = emac->prueth;
	fdb = prueth->fdb_tbl;
	mt = fdb->mac_tbl_a;

	if (fdb->total_entries == 0)
		return 0;

	/* Get the bucket that the mac belongs to */
	hash_val = icssm_prueth_sw_fdb_hash(mac);
	bucket_info = FDB_IDX_TBL_ENTRY(hash_val);

	ret = icssm_prueth_sw_fdb_search(mt, bucket_info, mac);
	if (ret < 0)
		return ret;

	mac_tbl_idx = ret;
	mac_info = FDB_MAC_TBL_ENTRY(mac_tbl_idx);

	icssm_prueth_sw_fdb_spin_lock(fdb);

	/* Shift all elements in bucket to the left. No need to
	 * update index table since only shifting within bucket.
	 */
	left = mac_tbl_idx;
	right = bucket_info->bucket_idx + bucket_info->bucket_entries - 1;
	icssm_prueth_sw_fdb_move_range_left(prueth, left, right);

	/* Remove end of bucket from table */
	mac_info = FDB_MAC_TBL_ENTRY(right);
	mac_info->active = 0;
	bucket_info->bucket_entries--;
	fdb->total_entries--;

	icssm_prueth_sw_fdb_spin_unlock(fdb);

	dev_dbg(prueth->dev, "del fdb: %pM total_entries=%u\n",
		mac, fdb->total_entries);

	return 0;
}

int icssm_prueth_sw_do_purge_fdb(struct prueth_emac *emac)
{
	struct fdb_index_tbl_entry_t *bucket_info;
	struct prueth *prueth = emac->prueth;
	struct fdb_tbl *fdb;
	u8 hash_val;
	s16 i;

	fdb = prueth->fdb_tbl;
	if (fdb->total_entries == 0)
		return 0;

	icssm_prueth_sw_fdb_spin_lock(fdb);

	for (i = 0; i < FDB_MAC_TBL_MAX_ENTRIES; i++) {
		if (fdb->mac_tbl_a->mac_tbl_entry[i].active) {
			if (!fdb->mac_tbl_a->mac_tbl_entry[i].is_static) {
				/* Get the bucket that the mac belongs to */
				hash_val = icssm_prueth_sw_fdb_hash
					(FDB_MAC_TBL_ENTRY(i)->mac);
				bucket_info = FDB_IDX_TBL_ENTRY(hash_val);
				fdb->mac_tbl_a->mac_tbl_entry[i].active = 0;
				bucket_info->bucket_entries--;
				fdb->total_entries--;
			}
		}
	}

	icssm_prueth_sw_fdb_spin_unlock(fdb);
	return 0;
}

int icssm_prueth_sw_init_fdb_table(struct prueth *prueth)
{
	if (prueth->emac_configured)
		return 0;

	prueth->fdb_tbl = kmalloc(sizeof(*prueth->fdb_tbl), GFP_KERNEL);
	if (!prueth->fdb_tbl)
		return -ENOMEM;

	icssm_prueth_sw_fdb_tbl_init(prueth);

	return 0;
}

/**
 * icssm_prueth_sw_fdb_add - insert fdb entry
 *
 * @emac: EMAC data structure
 * @fdb: fdb info
 *
 */
void icssm_prueth_sw_fdb_add(struct prueth_emac *emac,
			     struct switchdev_notifier_fdb_info *fdb)
{
	icssm_prueth_sw_insert_fdb_entry(emac, fdb->addr, 1);
}

void icssm_prueth_sw_fdb_del(struct prueth_emac *emac,
			     struct switchdev_notifier_fdb_info *fdb)
{
	icssm_prueth_sw_delete_fdb_entry(emac, fdb->addr, 1);
}

static void icssm_prueth_sw_fdb_work(struct work_struct *work)
{
	struct icssm_prueth_sw_fdb_work *fdb_work =
		container_of(work, struct icssm_prueth_sw_fdb_work, work);
	struct prueth_emac *emac = fdb_work->emac;

	rtnl_lock();

	/* Interface is not up */
	if (!emac->prueth->fdb_tbl) {
		rtnl_unlock();
		return;
	}

	switch (fdb_work->event) {
	case FDB_LEARN:
		icssm_prueth_sw_insert_fdb_entry(emac, fdb_work->addr, 0);
		break;
	case FDB_PURGE:
		icssm_prueth_sw_do_purge_fdb(emac);
		break;
	default:
		break;
	}
	rtnl_unlock();

	kfree(fdb_work);
	dev_put(emac->ndev);
}

int icssm_prueth_sw_learn_fdb(struct prueth_emac *emac, u8 *src_mac)
{
	struct icssm_prueth_sw_fdb_work *fdb_work;

	fdb_work = kzalloc(sizeof(*fdb_work), GFP_ATOMIC);
	if (WARN_ON(!fdb_work))
		return -ENOMEM;

	INIT_WORK(&fdb_work->work, icssm_prueth_sw_fdb_work);

	fdb_work->event = FDB_LEARN;
	fdb_work->emac  = emac;
	ether_addr_copy(fdb_work->addr, src_mac);

	dev_hold(emac->ndev);
	queue_work(system_long_wq, &fdb_work->work);
	return 0;
}

int icssm_prueth_sw_purge_fdb(struct prueth_emac *emac)
{
	struct icssm_prueth_sw_fdb_work *fdb_work;

	fdb_work = kzalloc(sizeof(*fdb_work), GFP_ATOMIC);
	if (WARN_ON(!fdb_work))
		return -ENOMEM;

	INIT_WORK(&fdb_work->work, icssm_prueth_sw_fdb_work);

	fdb_work->event = FDB_PURGE;
	fdb_work->emac  = emac;

	dev_hold(emac->ndev);
	queue_work(system_long_wq, &fdb_work->work);
	return 0;
}
