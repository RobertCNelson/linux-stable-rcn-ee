/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _BCACHEFS_BKEY_H
#define _BCACHEFS_BKEY_H

#include <linux/bug.h>
#include "bcachefs_format.h"
#include "bkey_types.h"
#include "btree_types.h"
#include "util.h"
#include "vstructs.h"

#if 0

/*
 * compiled unpack functions are disabled, pending a new interface for
 * dynamically allocating executable memory:
 */

#ifdef CONFIG_X86_64
#define HAVE_BCACHEFS_COMPILED_UNPACK	1
#endif
#endif

void bch2_bkey_packed_to_binary_text(struct printbuf *,
				     const struct bkey_format *,
				     const struct bkey_packed *);

enum bkey_lr_packed {
	BKEY_PACKED_BOTH,
	BKEY_PACKED_RIGHT,
	BKEY_PACKED_LEFT,
	BKEY_PACKED_NONE,
};

#define bkey_lr_packed(_l, _r)						\
	((_l)->format + ((_r)->format << 1))

static inline void bkey_p_copy(struct bkey_packed *dst, const struct bkey_packed *src)
{
	memcpy_u64s_small(dst, src, src->u64s);
}

static inline void bkey_copy(struct bkey_i *dst, const struct bkey_i *src)
{
	memcpy_u64s_small(dst, src, src->k.u64s);
}

struct btree;

__pure
unsigned bch2_bkey_greatest_differing_bit(const struct btree *,
					  const struct bkey_packed *,
					  const struct bkey_packed *);
__pure
unsigned bch2_bkey_ffs(const struct btree *, const struct bkey_packed *);

__pure
int __bch2_bkey_cmp_packed_format_checked(const struct bkey_packed *,
				     const struct bkey_packed *,
				     const struct btree *);

__pure
int __bch2_bkey_cmp_left_packed_format_checked(const struct btree *,
					  const struct bkey_packed *,
					  const struct bpos *);

__pure
int bch2_bkey_cmp_packed(const struct btree *,
			 const struct bkey_packed *,
			 const struct bkey_packed *);

__pure
int __bch2_bkey_cmp_left_packed(const struct btree *,
				const struct bkey_packed *,
				const struct bpos *);

static inline __pure
int bkey_cmp_left_packed(const struct btree *b,
			 const struct bkey_packed *l, const struct bpos *r)
{
	return __bch2_bkey_cmp_left_packed(b, l, r);
}

/*
 * The compiler generates better code when we pass bpos by ref, but it's often
 * enough terribly convenient to pass it by val... as much as I hate c++, const
 * ref would be nice here:
 */
__pure __flatten
static inline int bkey_cmp_left_packed_byval(const struct btree *b,
					     const struct bkey_packed *l,
					     struct bpos r)
{
	return bkey_cmp_left_packed(b, l, &r);
}

static __always_inline bool bpos_eq(struct bpos l, struct bpos r)
{
	return  !((l.inode	^ r.inode) |
		  (l.offset	^ r.offset) |
		  (l.snapshot	^ r.snapshot));
}

static __always_inline bool bpos_lt(struct bpos l, struct bpos r)
{
	return  l.inode	!= r.inode ? l.inode < r.inode :
		l.offset != r.offset ? l.offset < r.offset :
		l.snapshot != r.snapshot ? l.snapshot < r.snapshot : false;
}

static __always_inline bool bpos_le(struct bpos l, struct bpos r)
{
	return  l.inode	!= r.inode ? l.inode < r.inode :
		l.offset != r.offset ? l.offset < r.offset :
		l.snapshot != r.snapshot ? l.snapshot < r.snapshot : true;
}

static __always_inline bool bpos_gt(struct bpos l, struct bpos r)
{
	return bpos_lt(r, l);
}

static __always_inline bool bpos_ge(struct bpos l, struct bpos r)
{
	return bpos_le(r, l);
}

static __always_inline int bpos_cmp(struct bpos l, struct bpos r)
{
	return  cmp_int(l.inode,    r.inode) ?:
		cmp_int(l.offset,   r.offset) ?:
		cmp_int(l.snapshot, r.snapshot);
}

static inline struct bpos bpos_min(struct bpos l, struct bpos r)
{
	return bpos_lt(l, r) ? l : r;
}

static inline struct bpos bpos_max(struct bpos l, struct bpos r)
{
	return bpos_gt(l, r) ? l : r;
}

static __always_inline bool bkey_eq(struct bpos l, struct bpos r)
{
	return  !((l.inode	^ r.inode) |
		  (l.offset	^ r.offset));
}

static __always_inline bool bkey_lt(struct bpos l, struct bpos r)
{
	return  l.inode	!= r.inode
		? l.inode < r.inode
		: l.offset < r.offset;
}

static __always_inline bool bkey_le(struct bpos l, struct bpos r)
{
	return  l.inode	!= r.inode
		? l.inode < r.inode
		: l.offset <= r.offset;
}

static __always_inline bool bkey_gt(struct bpos l, struct bpos r)
{
	return bkey_lt(r, l);
}

static __always_inline bool bkey_ge(struct bpos l, struct bpos r)
{
	return bkey_le(r, l);
}

static __always_inline int bkey_cmp(struct bpos l, struct bpos r)
{
	return  cmp_int(l.inode,    r.inode) ?:
		cmp_int(l.offset,   r.offset);
}

static inline struct bpos bkey_min(struct bpos l, struct bpos r)
{
	return bkey_lt(l, r) ? l : r;
}

static inline struct bpos bkey_max(struct bpos l, struct bpos r)
{
	return bkey_gt(l, r) ? l : r;
}

static inline bool bkey_and_val_eq(struct bkey_s_c l, struct bkey_s_c r)
{
	return bpos_eq(l.k->p, r.k->p) &&
		l.k->size == r.k->size &&
		bkey_bytes(l.k) == bkey_bytes(r.k) &&
		!memcmp(l.v, r.v, bkey_val_bytes(l.k));
}

void bch2_bpos_swab(struct bpos *);
void bch2_bkey_swab_key(const struct bkey_format *, struct bkey_packed *);

static __always_inline int bversion_cmp(struct bversion l, struct bversion r)
{
	return  cmp_int(l.hi, r.hi) ?:
		cmp_int(l.lo, r.lo);
}

#define ZERO_VERSION	((struct bversion) { .hi = 0, .lo = 0 })
#define MAX_VERSION	((struct bversion) { .hi = ~0, .lo = ~0ULL })

static __always_inline bool bversion_zero(struct bversion v)
{
	return bversion_cmp(v, ZERO_VERSION) == 0;
}

#ifdef CONFIG_BCACHEFS_DEBUG
/* statement expressions confusing unlikely()? */
#define bkey_packed(_k)							\
	({ EBUG_ON((_k)->format > KEY_FORMAT_CURRENT);			\
	 (_k)->format != KEY_FORMAT_CURRENT; })
#else
#define bkey_packed(_k)		((_k)->format != KEY_FORMAT_CURRENT)
#endif

/*
 * It's safe to treat an unpacked bkey as a packed one, but not the reverse
 */
static inline struct bkey_packed *bkey_to_packed(struct bkey_i *k)
{
	return (struct bkey_packed *) k;
}

static inline const struct bkey_packed *bkey_to_packed_c(const struct bkey_i *k)
{
	return (const struct bkey_packed *) k;
}

static inline struct bkey_i *packed_to_bkey(struct bkey_packed *k)
{
	return bkey_packed(k) ? NULL : (struct bkey_i *) k;
}

static inline const struct bkey *packed_to_bkey_c(const struct bkey_packed *k)
{
	return bkey_packed(k) ? NULL : (const struct bkey *) k;
}

static inline unsigned bkey_format_key_bits(const struct bkey_format *format)
{
	return format->bits_per_field[BKEY_FIELD_INODE] +
		format->bits_per_field[BKEY_FIELD_OFFSET] +
		format->bits_per_field[BKEY_FIELD_SNAPSHOT];
}

static inline struct bpos bpos_successor(struct bpos p)
{
	if (!++p.snapshot &&
	    !++p.offset &&
	    !++p.inode)
		BUG();

	return p;
}

static inline struct bpos bpos_predecessor(struct bpos p)
{
	if (!p.snapshot-- &&
	    !p.offset-- &&
	    !p.inode--)
		BUG();

	return p;
}

static inline struct bpos bpos_nosnap_successor(struct bpos p)
{
	p.snapshot = 0;

	if (!++p.offset &&
	    !++p.inode)
		BUG();

	return p;
}

static inline struct bpos bpos_nosnap_predecessor(struct bpos p)
{
	p.snapshot = 0;

	if (!p.offset-- &&
	    !p.inode--)
		BUG();

	return p;
}

static inline u64 bkey_start_offset(const struct bkey *k)
{
	return k->p.offset - k->size;
}

static inline struct bpos bkey_start_pos(const struct bkey *k)
{
	return (struct bpos) {
		.inode		= k->p.inode,
		.offset		= bkey_start_offset(k),
		.snapshot	= k->p.snapshot,
	};
}

/* Packed helpers */

static inline unsigned bkeyp_key_u64s(const struct bkey_format *format,
				      const struct bkey_packed *k)
{
	return bkey_packed(k) ? format->key_u64s : BKEY_U64s;
}

static inline bool bkeyp_u64s_valid(const struct bkey_format *f,
				    const struct bkey_packed *k)
{
	return ((unsigned) k->u64s - bkeyp_key_u64s(f, k) <= U8_MAX - BKEY_U64s);
}

static inline unsigned bkeyp_key_bytes(const struct bkey_format *format,
				       const struct bkey_packed *k)
{
	return bkeyp_key_u64s(format, k) * sizeof(u64);
}

static inline unsigned bkeyp_val_u64s(const struct bkey_format *format,
				      const struct bkey_packed *k)
{
	return k->u64s - bkeyp_key_u64s(format, k);
}

static inline size_t bkeyp_val_bytes(const struct bkey_format *format,
				     const struct bkey_packed *k)
{
	return bkeyp_val_u64s(format, k) * sizeof(u64);
}

static inline void set_bkeyp_val_u64s(const struct bkey_format *format,
				      struct bkey_packed *k, unsigned val_u64s)
{
	k->u64s = bkeyp_key_u64s(format, k) + val_u64s;
}

#define bkeyp_val(_format, _k)						\
	 ((struct bch_val *) ((u64 *) (_k)->_data + bkeyp_key_u64s(_format, _k)))

extern const struct bkey_format bch2_bkey_format_current;

bool bch2_bkey_transform(const struct bkey_format *,
			 struct bkey_packed *,
			 const struct bkey_format *,
			 const struct bkey_packed *);

struct bkey __bch2_bkey_unpack_key(const struct bkey_format *,
				   const struct bkey_packed *);

#ifndef HAVE_BCACHEFS_COMPILED_UNPACK
struct bpos __bkey_unpack_pos(const struct bkey_format *,
			      const struct bkey_packed *);
#endif

bool bch2_bkey_pack_key(struct bkey_packed *, const struct bkey *,
		   const struct bkey_format *);

enum bkey_pack_pos_ret {
	BKEY_PACK_POS_EXACT,
	BKEY_PACK_POS_SMALLER,
	BKEY_PACK_POS_FAIL,
};

enum bkey_pack_pos_ret bch2_bkey_pack_pos_lossy(struct bkey_packed *, struct bpos,
					   const struct btree *);

static inline bool bkey_pack_pos(struct bkey_packed *out, struct bpos in,
				 const struct btree *b)
{
	return bch2_bkey_pack_pos_lossy(out, in, b) == BKEY_PACK_POS_EXACT;
}

void bch2_bkey_unpack(const struct btree *, struct bkey_i *,
		 const struct bkey_packed *);
bool bch2_bkey_pack(struct bkey_packed *, const struct bkey_i *,
	       const struct bkey_format *);

typedef void (*compiled_unpack_fn)(struct bkey *, const struct bkey_packed *);

static inline void
__bkey_unpack_key_format_checked(const struct btree *b,
			       struct bkey *dst,
			       const struct bkey_packed *src)
{
	if (IS_ENABLED(HAVE_BCACHEFS_COMPILED_UNPACK)) {
		compiled_unpack_fn unpack_fn = b->aux_data;
		unpack_fn(dst, src);

		if (static_branch_unlikely(&bch2_debug_check_bkey_unpack)) {
			struct bkey dst2 = __bch2_bkey_unpack_key(&b->format, src);

			BUG_ON(memcmp(dst, &dst2, sizeof(*dst)));
		}
	} else {
		*dst = __bch2_bkey_unpack_key(&b->format, src);
	}
}

static inline struct bkey
bkey_unpack_key_format_checked(const struct btree *b,
			       const struct bkey_packed *src)
{
	struct bkey dst;

	__bkey_unpack_key_format_checked(b, &dst, src);
	return dst;
}

static inline void __bkey_unpack_key(const struct btree *b,
				     struct bkey *dst,
				     const struct bkey_packed *src)
{
	if (likely(bkey_packed(src)))
		__bkey_unpack_key_format_checked(b, dst, src);
	else
		*dst = *packed_to_bkey_c(src);
}

/**
 * bkey_unpack_key -- unpack just the key, not the value
 */
static inline struct bkey bkey_unpack_key(const struct btree *b,
					  const struct bkey_packed *src)
{
	return likely(bkey_packed(src))
		? bkey_unpack_key_format_checked(b, src)
		: *packed_to_bkey_c(src);
}

static inline struct bpos
bkey_unpack_pos_format_checked(const struct btree *b,
			       const struct bkey_packed *src)
{
#ifdef HAVE_BCACHEFS_COMPILED_UNPACK
	return bkey_unpack_key_format_checked(b, src).p;
#else
	return __bkey_unpack_pos(&b->format, src);
#endif
}

static inline struct bpos bkey_unpack_pos(const struct btree *b,
					  const struct bkey_packed *src)
{
	return likely(bkey_packed(src))
		? bkey_unpack_pos_format_checked(b, src)
		: packed_to_bkey_c(src)->p;
}

/* Disassembled bkeys */

static inline struct bkey_s_c bkey_disassemble(const struct btree *b,
					       const struct bkey_packed *k,
					       struct bkey *u)
{
	__bkey_unpack_key(b, u, k);

	return (struct bkey_s_c) { u, bkeyp_val(&b->format, k), };
}

/* non const version: */
static inline struct bkey_s __bkey_disassemble(const struct btree *b,
					       struct bkey_packed *k,
					       struct bkey *u)
{
	__bkey_unpack_key(b, u, k);

	return (struct bkey_s) { .k = u, .v = bkeyp_val(&b->format, k), };
}

static inline u64 bkey_field_max(const struct bkey_format *f,
				 enum bch_bkey_fields nr)
{
	return f->bits_per_field[nr] < 64
		? (le64_to_cpu(f->field_offset[nr]) +
		   ~(~0ULL << f->bits_per_field[nr]))
		: U64_MAX;
}

#ifdef HAVE_BCACHEFS_COMPILED_UNPACK

int bch2_compile_bkey_format(const struct bkey_format *, void *);

#else

static inline int bch2_compile_bkey_format(const struct bkey_format *format,
					  void *out) { return 0; }

#endif

static inline void bkey_reassemble(struct bkey_i *dst,
				   struct bkey_s_c src)
{
	dst->k = *src.k;
	memcpy_u64s_small(&dst->v, src.v, bkey_val_u64s(src.k));
}

/* byte order helpers */

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__

static inline unsigned high_word_offset(const struct bkey_format *f)
{
	return f->key_u64s - 1;
}

#define high_bit_offset		0
#define nth_word(p, n)		((p) - (n))

#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__

static inline unsigned high_word_offset(const struct bkey_format *f)
{
	return 0;
}

#define high_bit_offset		KEY_PACKED_BITS_START
#define nth_word(p, n)		((p) + (n))

#else
#error edit for your odd byteorder.
#endif

#define high_word(f, k)		((u64 *) (k)->_data + high_word_offset(f))
#define next_word(p)		nth_word(p, 1)
#define prev_word(p)		nth_word(p, -1)

#ifdef CONFIG_BCACHEFS_DEBUG
void bch2_bkey_pack_test(void);
#else
static inline void bch2_bkey_pack_test(void) {}
#endif

#define bkey_fields()							\
	x(BKEY_FIELD_INODE,		p.inode)			\
	x(BKEY_FIELD_OFFSET,		p.offset)			\
	x(BKEY_FIELD_SNAPSHOT,		p.snapshot)			\
	x(BKEY_FIELD_SIZE,		size)				\
	x(BKEY_FIELD_VERSION_HI,	bversion.hi)			\
	x(BKEY_FIELD_VERSION_LO,	bversion.lo)

struct bkey_format_state {
	u64 field_min[BKEY_NR_FIELDS];
	u64 field_max[BKEY_NR_FIELDS];
};

void bch2_bkey_format_init(struct bkey_format_state *);

static inline void __bkey_format_add(struct bkey_format_state *s, unsigned field, u64 v)
{
	s->field_min[field] = min(s->field_min[field], v);
	s->field_max[field] = max(s->field_max[field], v);
}

/*
 * Changes @format so that @k can be successfully packed with @format
 */
static inline void bch2_bkey_format_add_key(struct bkey_format_state *s, const struct bkey *k)
{
#define x(id, field) __bkey_format_add(s, id, k->field);
	bkey_fields()
#undef x
}

void bch2_bkey_format_add_pos(struct bkey_format_state *, struct bpos);
struct bkey_format bch2_bkey_format_done(struct bkey_format_state *);

static inline bool bch2_bkey_format_field_overflows(struct bkey_format *f, unsigned i)
{
	unsigned f_bits = f->bits_per_field[i];
	unsigned unpacked_bits = bch2_bkey_format_current.bits_per_field[i];
	u64 unpacked_mask = ~((~0ULL << 1) << (unpacked_bits - 1));
	u64 field_offset = le64_to_cpu(f->field_offset[i]);

	if (f_bits > unpacked_bits)
		return true;

	if ((f_bits == unpacked_bits) && field_offset)
		return true;

	u64 f_mask = f_bits
		? ~((~0ULL << (f_bits - 1)) << 1)
		: 0;

	if (((field_offset + f_mask) & unpacked_mask) < field_offset)
		return true;
	return false;
}

int bch2_bkey_format_invalid(struct bch_fs *, struct bkey_format *,
			     enum bch_validate_flags, struct printbuf *);
void bch2_bkey_format_to_text(struct printbuf *, const struct bkey_format *);

#endif /* _BCACHEFS_BKEY_H */
