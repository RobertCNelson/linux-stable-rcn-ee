// SPDX-License-Identifier: GPL-2.0-only
/*
 *  linux/kernel/printk.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 * Modified to make sys_syslog() more flexible: added commands to
 * return the last 4k of kernel messages, regardless of whether
 * they've been read or not.  Added option to suppress kernel printk's
 * to the console.  Added hook for sending the console messages
 * elsewhere, in preparation for a serial line console (someday).
 * Ted Ts'o, 2/11/93.
 * Modified for sysctl support, 1/8/97, Chris Horn.
 * Fixed SMP synchronization, 08/08/99, Manfred Spraul
 *     manfred@colorfullife.com
 * Rewrote bits to get rid of console_lock
 *	01Mar01 Andrew Morton
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/nmi.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/security.h>
#include <linux/memblock.h>
#include <linux/syscalls.h>
#include <linux/crash_core.h>
#include <linux/kdb.h>
#include <linux/ratelimit.h>
#include <linux/kmsg_dump.h>
#include <linux/syslog.h>
#include <linux/cpu.h>
#include <linux/rculist.h>
#include <linux/poll.h>
#include <linux/irq_work.h>
#include <linux/ctype.h>
#include <linux/uio.h>
#include <linux/kthread.h>
#include <linux/clocksource.h>
#include <linux/printk_ringbuffer.h>
#include <linux/sched/clock.h>
#include <linux/sched/debug.h>
#include <linux/sched/task_stack.h>

#include <linux/uaccess.h>
#include <asm/sections.h>

#include <trace/events/initcall.h>
#define CREATE_TRACE_POINTS
#include <trace/events/printk.h>

#include "console_cmdline.h"
#include "braille.h"

int console_printk[5] = {
	CONSOLE_LOGLEVEL_DEFAULT,	/* console_loglevel */
	MESSAGE_LOGLEVEL_DEFAULT,	/* default_message_loglevel */
	CONSOLE_LOGLEVEL_MIN,		/* minimum_console_loglevel */
	CONSOLE_LOGLEVEL_DEFAULT,	/* default_console_loglevel */
	CONSOLE_LOGLEVEL_EMERGENCY,	/* emergency_console_loglevel */
};
EXPORT_SYMBOL_GPL(console_printk);

atomic_t ignore_console_lock_warning __read_mostly = ATOMIC_INIT(0);
EXPORT_SYMBOL(ignore_console_lock_warning);

/*
 * Low level drivers may need that to know if they can schedule in
 * their unblank() callback or not. So let's export it.
 */
int oops_in_progress;
EXPORT_SYMBOL(oops_in_progress);

/*
 * console_sem protects the console_drivers list, and also
 * provides serialisation for access to the entire console
 * driver system.
 */
static DEFINE_SEMAPHORE(console_sem);
struct console *console_drivers;
EXPORT_SYMBOL_GPL(console_drivers);

/*
 * System may need to suppress printk message under certain
 * circumstances, like after kernel panic happens.
 */
int __read_mostly suppress_printk;

#ifdef CONFIG_LOCKDEP
static struct lockdep_map console_lock_dep_map = {
	.name = "console_lock"
};
#endif

enum devkmsg_log_bits {
	__DEVKMSG_LOG_BIT_ON = 0,
	__DEVKMSG_LOG_BIT_OFF,
	__DEVKMSG_LOG_BIT_LOCK,
};

enum devkmsg_log_masks {
	DEVKMSG_LOG_MASK_ON             = BIT(__DEVKMSG_LOG_BIT_ON),
	DEVKMSG_LOG_MASK_OFF            = BIT(__DEVKMSG_LOG_BIT_OFF),
	DEVKMSG_LOG_MASK_LOCK           = BIT(__DEVKMSG_LOG_BIT_LOCK),
};

/* Keep both the 'on' and 'off' bits clear, i.e. ratelimit by default: */
#define DEVKMSG_LOG_MASK_DEFAULT	0

static unsigned int __read_mostly devkmsg_log = DEVKMSG_LOG_MASK_DEFAULT;

static int __control_devkmsg(char *str)
{
	size_t len;

	if (!str)
		return -EINVAL;

	len = str_has_prefix(str, "on");
	if (len) {
		devkmsg_log = DEVKMSG_LOG_MASK_ON;
		return len;
	}

	len = str_has_prefix(str, "off");
	if (len) {
		devkmsg_log = DEVKMSG_LOG_MASK_OFF;
		return len;
	}

	len = str_has_prefix(str, "ratelimit");
	if (len) {
		devkmsg_log = DEVKMSG_LOG_MASK_DEFAULT;
		return len;
	}

	return -EINVAL;
}

static int __init control_devkmsg(char *str)
{
	if (__control_devkmsg(str) < 0)
		return 1;

	/*
	 * Set sysctl string accordingly:
	 */
	if (devkmsg_log == DEVKMSG_LOG_MASK_ON)
		strcpy(devkmsg_log_str, "on");
	else if (devkmsg_log == DEVKMSG_LOG_MASK_OFF)
		strcpy(devkmsg_log_str, "off");
	/* else "ratelimit" which is set by default. */

	/*
	 * Sysctl cannot change it anymore. The kernel command line setting of
	 * this parameter is to force the setting to be permanent throughout the
	 * runtime of the system. This is a precation measure against userspace
	 * trying to be a smarta** and attempting to change it up on us.
	 */
	devkmsg_log |= DEVKMSG_LOG_MASK_LOCK;

	return 0;
}
__setup("printk.devkmsg=", control_devkmsg);

char devkmsg_log_str[DEVKMSG_STR_MAX_SIZE] = "ratelimit";

int devkmsg_sysctl_set_loglvl(struct ctl_table *table, int write,
			      void __user *buffer, size_t *lenp, loff_t *ppos)
{
	char old_str[DEVKMSG_STR_MAX_SIZE];
	unsigned int old;
	int err;

	if (write) {
		if (devkmsg_log & DEVKMSG_LOG_MASK_LOCK)
			return -EINVAL;

		old = devkmsg_log;
		strncpy(old_str, devkmsg_log_str, DEVKMSG_STR_MAX_SIZE);
	}

	err = proc_dostring(table, write, buffer, lenp, ppos);
	if (err)
		return err;

	if (write) {
		err = __control_devkmsg(devkmsg_log_str);

		/*
		 * Do not accept an unknown string OR a known string with
		 * trailing crap...
		 */
		if (err < 0 || (err + 1 != *lenp)) {

			/* ... and restore old setting. */
			devkmsg_log = old;
			strncpy(devkmsg_log_str, old_str, DEVKMSG_STR_MAX_SIZE);

			return -EINVAL;
		}
	}

	return 0;
}

/* Number of registered extended console drivers. */
static int nr_ext_console_drivers;

/*
 * Helper macros to handle lockdep when locking/unlocking console_sem. We use
 * macros instead of functions so that _RET_IP_ contains useful information.
 */
#define down_console_sem() do { \
	down(&console_sem);\
	mutex_acquire(&console_lock_dep_map, 0, 0, _RET_IP_);\
} while (0)

static int __down_trylock_console_sem(unsigned long ip)
{
	if (down_trylock(&console_sem))
		return 1;
	mutex_acquire(&console_lock_dep_map, 0, 1, ip);
	return 0;
}
#define down_trylock_console_sem() __down_trylock_console_sem(_RET_IP_)

static void __up_console_sem(unsigned long ip)
{
	mutex_release(&console_lock_dep_map, ip);

	up(&console_sem);
}
#define up_console_sem() __up_console_sem(_RET_IP_)

/*
 * This is used for debugging the mess that is the VT code by
 * keeping track if we have the console semaphore held. It's
 * definitely not the perfect debug tool (we don't know if _WE_
 * hold it and are racing, but it helps tracking those weird code
 * paths in the console code where we end up in places I want
 * locked without the console sempahore held).
 */
static int console_locked, console_suspended;

/*
 *	Array of consoles built from command line options (console=)
 */

#define MAX_CMDLINECONSOLES 8

static struct console_cmdline console_cmdline[MAX_CMDLINECONSOLES];

static int preferred_console = -1;
int console_set_on_cmdline;
EXPORT_SYMBOL(console_set_on_cmdline);

/* Flag: console code may call schedule() */
static int console_may_schedule;

enum con_msg_format_flags {
	MSG_FORMAT_DEFAULT	= 0,
	MSG_FORMAT_SYSLOG	= (1 << 0),
};

static int console_msg_format = MSG_FORMAT_DEFAULT;

/*
 * The printk log buffer consists of a chain of concatenated variable
 * length records. Every record starts with a record header, containing
 * the overall length of the record.
 *
 * The heads to the first and last entry in the buffer, as well as the
 * sequence numbers of these entries are maintained when messages are
 * stored.
 *
 * If the heads indicate available messages, the length in the header
 * tells the start next message. A length == 0 for the next message
 * indicates a wrap-around to the beginning of the buffer.
 *
 * Every record carries the monotonic timestamp in microseconds, as well as
 * the standard userspace syslog level and syslog facility. The usual
 * kernel messages use LOG_KERN; userspace-injected messages always carry
 * a matching syslog facility, by default LOG_USER. The origin of every
 * message can be reliably determined that way.
 *
 * The human readable log message directly follows the message header. The
 * length of the message text is stored in the header, the stored message
 * is not terminated.
 *
 * Optionally, a message can carry a dictionary of properties (key/value pairs),
 * to provide userspace with a machine-readable message context.
 *
 * Examples for well-defined, commonly used property names are:
 *   DEVICE=b12:8               device identifier
 *                                b12:8         block dev_t
 *                                c127:3        char dev_t
 *                                n8            netdev ifindex
 *                                +sound:card0  subsystem:devname
 *   SUBSYSTEM=pci              driver-core subsystem name
 *
 * Valid characters in property names are [a-zA-Z0-9.-_]. The plain text value
 * follows directly after a '=' character. Every property is terminated by
 * a '\0' character. The last property is not terminated.
 *
 * Example of a message structure:
 *   0000  ff 8f 00 00 00 00 00 00      monotonic time in nsec
 *   0008  34 00                        record is 52 bytes long
 *   000a        0b 00                  text is 11 bytes long
 *   000c              1f 00            dictionary is 23 bytes long
 *   000e                    03 00      LOG_KERN (facility) LOG_ERR (level)
 *   0010  69 74 27 73 20 61 20 6c      "it's a l"
 *         69 6e 65                     "ine"
 *   001b           44 45 56 49 43      "DEVIC"
 *         45 3d 62 38 3a 32 00 44      "E=b8:2\0D"
 *         52 49 56 45 52 3d 62 75      "RIVER=bu"
 *         67                           "g"
 *   0032     00 00 00                  padding to next message header
 *
 * The 'struct printk_log' buffer header must never be directly exported to
 * userspace, it is a kernel-private implementation detail that might
 * need to be changed in the future, when the requirements change.
 *
 * /dev/kmsg exports the structured data in the following line format:
 *   "<level>,<sequnum>,<timestamp>,<contflag>[,additional_values, ... ];<message text>\n"
 *
 * Users of the export format should ignore possible additional values
 * separated by ',', and find the message after the ';' character.
 *
 * The optional key/value pairs are attached as continuation lines starting
 * with a space character and terminated by a newline. All possible
 * non-prinatable characters are escaped in the "\xff" notation.
 */

enum log_flags {
	LOG_NEWLINE	= 2,	/* text ended with a newline */
	LOG_CONT	= 8,	/* text is a fragment of a continuation line */
};

struct printk_log {
	u64 ts_nsec;		/* timestamp in nanoseconds */
	u16 cpu;		/* cpu that generated record */
	u16 len;		/* length of entire record */
	u16 text_len;		/* length of text buffer */
	u16 dict_len;		/* length of dictionary buffer */
	u8 facility;		/* syslog facility */
	u8 flags:5;		/* internal record flags */
	u8 level:3;		/* syslog level */
#ifdef CONFIG_PRINTK_CALLER
	u32 caller_id;            /* thread id or processor id */
#endif
}
#ifdef CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS
__packed __aligned(4)
#endif
;

DECLARE_STATIC_PRINTKRB_CPULOCK(printk_cpulock);

#ifdef CONFIG_PRINTK
/* record buffer */
DECLARE_STATIC_PRINTKRB(printk_rb, CONFIG_LOG_BUF_SHIFT, &printk_cpulock);

static DEFINE_MUTEX(syslog_lock);
DECLARE_STATIC_PRINTKRB_ITER(syslog_iter, &printk_rb);

/* the last printk record to read by syslog(READ) or /proc/kmsg */
static u64 syslog_seq;
static size_t syslog_partial;
static bool syslog_time;

/* the next printk record to read after the last 'clear' command */
static u64 clear_seq;

#ifdef CONFIG_PRINTK_CALLER
#define PREFIX_MAX		48
#else
#define PREFIX_MAX		32
#endif
#define LOG_LINE_MAX		(1024 - PREFIX_MAX)

#define LOG_LEVEL(v)		((v) & 0x07)
#define LOG_FACILITY(v)		((v) >> 3 & 0xff)

/* Return log buffer address */
char *log_buf_addr_get(void)
{
	return printk_rb.buffer;
}

/* Return log buffer size */
u32 log_buf_len_get(void)
{
	return (1 << printk_rb.size_bits);
}

/* human readable text of the record */
static char *log_text(const struct printk_log *msg)
{
	return (char *)msg + sizeof(struct printk_log);
}

/* optional key/value pair dictionary attached to the record */
static char *log_dict(const struct printk_log *msg)
{
	return (char *)msg + sizeof(struct printk_log) + msg->text_len;
}

static void printk_emergency(char *buffer, int level, u64 ts_nsec, u16 cpu,
			     char *text, u16 text_len);

/* insert record into the buffer, discard old ones, update heads */
static int log_store(u32 caller_id, int facility, int level,
		     enum log_flags flags, u64 ts_nsec, u16 cpu,
		     const char *dict, u16 dict_len,
		     const char *text, u16 text_len)
{
	struct printk_log *msg;
	struct prb_handle h;
	char *rbuf;
	u32 size;

	size = sizeof(*msg) + text_len + dict_len;

	rbuf = prb_reserve(&h, &printk_rb, size);
	if (!rbuf) {
		/*
		 * An emergency message would have been printed, but
		 * it cannot be stored in the log.
		 */
		prb_inc_lost(&printk_rb);
		return 0;
	}

	/* fill message */
	msg = (struct printk_log *)rbuf;
	memcpy(log_text(msg), text, text_len);
	msg->text_len = text_len;
	memcpy(log_dict(msg), dict, dict_len);
	msg->dict_len = dict_len;
	msg->facility = facility;
	msg->level = level & 7;
	msg->flags = flags & 0x1f;
	msg->ts_nsec = ts_nsec;
#ifdef CONFIG_PRINTK_CALLER
	msg->caller_id = caller_id;
#endif
	msg->cpu = cpu;
	msg->len = size;

	/* insert message */
	prb_commit(&h);

	return msg->text_len;
}

int dmesg_restrict = IS_ENABLED(CONFIG_SECURITY_DMESG_RESTRICT);

static int syslog_action_restricted(int type)
{
	if (dmesg_restrict)
		return 1;
	/*
	 * Unless restricted, we allow "read all" and "get buffer size"
	 * for everybody.
	 */
	return type != SYSLOG_ACTION_READ_ALL &&
	       type != SYSLOG_ACTION_SIZE_BUFFER;
}

static int check_syslog_permissions(int type, int source)
{
	/*
	 * If this is from /proc/kmsg and we've already opened it, then we've
	 * already done the capabilities checks at open time.
	 */
	if (source == SYSLOG_FROM_PROC && type != SYSLOG_ACTION_OPEN)
		goto ok;

	if (syslog_action_restricted(type)) {
		if (capable(CAP_SYSLOG))
			goto ok;
		/*
		 * For historical reasons, accept CAP_SYS_ADMIN too, with
		 * a warning.
		 */
		if (capable(CAP_SYS_ADMIN)) {
			pr_warn_once("%s (%d): Attempt to access syslog with "
				     "CAP_SYS_ADMIN but no CAP_SYSLOG "
				     "(deprecated).\n",
				 current->comm, task_pid_nr(current));
			goto ok;
		}
		return -EPERM;
	}
ok:
	return security_syslog(type);
}

static void append_char(char **pp, char *e, char c)
{
	if (*pp < e)
		*(*pp)++ = c;
}

static ssize_t msg_print_ext_header(char *buf, size_t size,
				    struct printk_log *msg, u64 seq)
{
	u64 ts_usec = msg->ts_nsec;
	char caller[20];
#ifdef CONFIG_PRINTK_CALLER
	u32 id = msg->caller_id;

	snprintf(caller, sizeof(caller), ",caller=%c%u",
		 id & 0x80000000 ? 'C' : 'T', id & ~0x80000000);
#else
	caller[0] = '\0';
#endif

	do_div(ts_usec, 1000);

	return scnprintf(buf, size, "%u,%llu,%llu,%c%s,%hu;",
			 (msg->facility << 3) | msg->level, seq, ts_usec,
			 msg->flags & LOG_CONT ? 'c' : '-', caller, msg->cpu);
}

static ssize_t msg_print_ext_body(char *buf, size_t size,
				  char *dict, size_t dict_len,
				  char *text, size_t text_len)
{
	char *p = buf, *e = buf + size;
	size_t i;

	/* escape non-printable characters */
	for (i = 0; i < text_len; i++) {
		unsigned char c = text[i];

		if (c < ' ' || c >= 127 || c == '\\')
			p += scnprintf(p, e - p, "\\x%02x", c);
		else
			append_char(&p, e, c);
	}
	append_char(&p, e, '\n');

	if (dict_len) {
		bool line = true;

		for (i = 0; i < dict_len; i++) {
			unsigned char c = dict[i];

			if (line) {
				append_char(&p, e, ' ');
				line = false;
			}

			if (c == '\0') {
				append_char(&p, e, '\n');
				line = true;
				continue;
			}

			if (c < ' ' || c >= 127 || c == '\\') {
				p += scnprintf(p, e - p, "\\x%02x", c);
				continue;
			}

			append_char(&p, e, c);
		}
		append_char(&p, e, '\n');
	}

	return p - buf;
}

#define PRINTK_SPRINT_MAX (LOG_LINE_MAX + PREFIX_MAX)
#define PRINTK_RECORD_MAX (sizeof(struct printk_log) + \
				CONSOLE_EXT_LOG_MAX + PRINTK_SPRINT_MAX)

/* /dev/kmsg - userspace message inject/listen interface */
struct devkmsg_user {
	u64 seq;
	struct prb_iterator iter;
	struct ratelimit_state rs;
	struct mutex lock;
	char buf[CONSOLE_EXT_LOG_MAX];
	char msgbuf[PRINTK_RECORD_MAX];
};

static __printf(3, 4) __cold
int devkmsg_emit(int facility, int level, const char *fmt, ...)
{
	va_list args;
	int r;

	va_start(args, fmt);
	r = vprintk_emit(facility, level, NULL, 0, fmt, args);
	va_end(args);

	return r;
}

static ssize_t devkmsg_write(struct kiocb *iocb, struct iov_iter *from)
{
	char *buf, *line;
	int level = default_message_loglevel;
	int facility = 1;	/* LOG_USER */
	struct file *file = iocb->ki_filp;
	struct devkmsg_user *user = file->private_data;
	size_t len = iov_iter_count(from);
	ssize_t ret = len;

	if (!user || len > LOG_LINE_MAX)
		return -EINVAL;

	/* Ignore when user logging is disabled. */
	if (devkmsg_log & DEVKMSG_LOG_MASK_OFF)
		return len;

	/* Ratelimit when not explicitly enabled. */
	if (!(devkmsg_log & DEVKMSG_LOG_MASK_ON)) {
		if (!___ratelimit(&user->rs, current->comm))
			return ret;
	}

	buf = kmalloc(len+1, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	buf[len] = '\0';
	if (!copy_from_iter_full(buf, len, from)) {
		kfree(buf);
		return -EFAULT;
	}

	/*
	 * Extract and skip the syslog prefix <[0-9]*>. Coming from userspace
	 * the decimal value represents 32bit, the lower 3 bit are the log
	 * level, the rest are the log facility.
	 *
	 * If no prefix or no userspace facility is specified, we
	 * enforce LOG_USER, to be able to reliably distinguish
	 * kernel-generated messages from userspace-injected ones.
	 */
	line = buf;
	if (line[0] == '<') {
		char *endp = NULL;
		unsigned int u;

		u = simple_strtoul(line + 1, &endp, 10);
		if (endp && endp[0] == '>') {
			level = LOG_LEVEL(u);
			if (LOG_FACILITY(u) != 0)
				facility = LOG_FACILITY(u);
			endp++;
			len -= endp - line;
			line = endp;
		}
	}

	devkmsg_emit(facility, level, "%s", line);
	kfree(buf);
	return ret;
}

static ssize_t devkmsg_read(struct file *file, char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct devkmsg_user *user = file->private_data;
	struct prb_iterator backup_iter;
	struct printk_log *msg;
	ssize_t ret;
	size_t len;
	u64 seq;

	if (!user)
		return -EBADF;

	ret = mutex_lock_interruptible(&user->lock);
	if (ret)
		return ret;

	/* make a backup copy in case there is a problem */
	prb_iter_copy(&backup_iter, &user->iter);

	if (file->f_flags & O_NONBLOCK) {
		ret = prb_iter_next(&user->iter, &user->msgbuf[0],
				      sizeof(user->msgbuf), &seq);
	} else {
		ret = prb_iter_wait_next(&user->iter, &user->msgbuf[0],
					   sizeof(user->msgbuf), &seq);
	}
	if (ret == 0) {
		/* end of list */
		ret = -EAGAIN;
		goto out;
	} else if (ret == -EINVAL) {
		/* iterator invalid, return error and reset */
		ret = -EPIPE;
		prb_iter_init(&user->iter, &printk_rb, &user->seq);
		goto out;
	} else if (ret < 0) {
		/* interrupted by signal */
		goto out;
	}

	user->seq++;
	if (user->seq < seq) {
		ret = -EPIPE;
		goto restore_out;
	}

	msg = (struct printk_log *)&user->msgbuf[0];
	len = msg_print_ext_header(user->buf, sizeof(user->buf),
				   msg, user->seq);
	len += msg_print_ext_body(user->buf + len, sizeof(user->buf) - len,
				  log_dict(msg), msg->dict_len,
				  log_text(msg), msg->text_len);

	if (len > count) {
		ret = -EINVAL;
		goto restore_out;
	}

	if (copy_to_user(buf, user->buf, len)) {
		ret = -EFAULT;
		goto restore_out;
	}

	ret = len;
	goto out;
restore_out:
	/*
	 * There was an error, but this message should not be
	 * lost because of it. Restore the backup and setup
	 * seq so that it will work with the next read.
	 */
	prb_iter_copy(&user->iter, &backup_iter);
	user->seq = seq - 1;
out:
	mutex_unlock(&user->lock);
	return ret;
}

static loff_t devkmsg_llseek(struct file *file, loff_t offset, int whence)
{
	struct devkmsg_user *user = file->private_data;
	loff_t ret;
	u64 seq;

	if (!user)
		return -EBADF;
	if (offset)
		return -ESPIPE;

	ret = mutex_lock_interruptible(&user->lock);
	if (ret)
		return ret;

	switch (whence) {
	case SEEK_SET:
		/* the first record */
		prb_iter_init(&user->iter, &printk_rb, &user->seq);
		break;
	case SEEK_DATA:
		/*
		 * The first record after the last SYSLOG_ACTION_CLEAR,
		 * like issued by 'dmesg -c'. Reading /dev/kmsg itself
		 * changes no global state, and does not clear anything.
		 */
		for (;;) {
			prb_iter_init(&user->iter, &printk_rb, &seq);
			ret = prb_iter_seek(&user->iter, clear_seq);
			if (ret > 0) {
				/* seeked to clear seq */
				user->seq = clear_seq;
				break;
			} else if (ret == 0) {
				/*
				 * The end of the list was hit without
				 * ever seeing the clear seq. Just
				 * seek to the beginning of the list.
				 */
				prb_iter_init(&user->iter, &printk_rb,
						&user->seq);
				break;
			}
			/* iterator invalid, start over */

			/* reset clear_seq if it is no longer available */
			if (seq > clear_seq)
				clear_seq = 0;
		}
		ret = 0;
		break;
	case SEEK_END:
		/* after the last record */
		for (;;) {
			ret = prb_iter_next(&user->iter, NULL, 0, &user->seq);
			if (ret == 0)
				break;
			else if (ret > 0)
				continue;
			/* iterator invalid, start over */
			prb_iter_init(&user->iter, &printk_rb, &user->seq);
		}
		ret = 0;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&user->lock);
	return ret;
}

struct wait_queue_head *printk_wait_queue(void)
{
	/* FIXME: using prb internals! */
	return printk_rb.wq;
}

static __poll_t devkmsg_poll(struct file *file, poll_table *wait)
{
	struct devkmsg_user *user = file->private_data;
	struct prb_iterator iter;
	__poll_t ret = 0;
	int rbret;
	u64 seq;

	if (!user)
		return EPOLLERR|EPOLLNVAL;

	poll_wait(file, printk_wait_queue(), wait);

	mutex_lock(&user->lock);

	/* use copy so no actual iteration takes place */
	prb_iter_copy(&iter, &user->iter);

	rbret = prb_iter_next(&iter, &user->msgbuf[0],
				sizeof(user->msgbuf), &seq);
	if (rbret == 0)
		goto out;

	ret = EPOLLIN|EPOLLRDNORM;

	if (rbret < 0 || (seq - user->seq) != 1)
		ret |= EPOLLERR|EPOLLPRI;
out:
	mutex_unlock(&user->lock);

	return ret;
}

static int devkmsg_open(struct inode *inode, struct file *file)
{
	struct devkmsg_user *user;
	int err;

	if (devkmsg_log & DEVKMSG_LOG_MASK_OFF)
		return -EPERM;

	/* write-only does not need any file context */
	if ((file->f_flags & O_ACCMODE) != O_WRONLY) {
		err = check_syslog_permissions(SYSLOG_ACTION_READ_ALL,
					       SYSLOG_FROM_READER);
		if (err)
			return err;
	}

	user = kmalloc(sizeof(struct devkmsg_user), GFP_KERNEL);
	if (!user)
		return -ENOMEM;

	ratelimit_default_init(&user->rs);
	ratelimit_set_flags(&user->rs, RATELIMIT_MSG_ON_RELEASE);

	mutex_init(&user->lock);

	prb_iter_init(&user->iter, &printk_rb, &user->seq);

	file->private_data = user;
	return 0;
}

static int devkmsg_release(struct inode *inode, struct file *file)
{
	struct devkmsg_user *user = file->private_data;

	if (!user)
		return 0;

	ratelimit_state_exit(&user->rs);

	mutex_destroy(&user->lock);
	kfree(user);
	return 0;
}

const struct file_operations kmsg_fops = {
	.open = devkmsg_open,
	.read = devkmsg_read,
	.write_iter = devkmsg_write,
	.llseek = devkmsg_llseek,
	.poll = devkmsg_poll,
	.release = devkmsg_release,
};

#ifdef CONFIG_CRASH_CORE
/*
 * This appends the listed symbols to /proc/vmcore
 *
 * /proc/vmcore is used by various utilities, like crash and makedumpfile to
 * obtain access to symbols that are otherwise very difficult to locate.  These
 * symbols are specifically used so that utilities can access and extract the
 * dmesg log from a vmcore file after a crash.
 */
void log_buf_vmcoreinfo_setup(void)
{
	/*
	 * Export struct printk_log size and field offsets. User space tools can
	 * parse it and detect any changes to structure down the line.
	 */
	VMCOREINFO_STRUCT_SIZE(printk_log);
	VMCOREINFO_OFFSET(printk_log, ts_nsec);
	VMCOREINFO_OFFSET(printk_log, len);
	VMCOREINFO_OFFSET(printk_log, text_len);
	VMCOREINFO_OFFSET(printk_log, dict_len);
#ifdef CONFIG_PRINTK_CALLER
	VMCOREINFO_OFFSET(printk_log, caller_id);
#endif
}
#endif

/* FIXME: no support for buffer resizing */
#if 0
/* requested log_buf_len from kernel cmdline */
static unsigned long __initdata new_log_buf_len;

/* we practice scaling the ring buffer by powers of 2 */
static void __init log_buf_len_update(u64 size)
{
	if (size > (u64)LOG_BUF_LEN_MAX) {
		size = (u64)LOG_BUF_LEN_MAX;
		pr_err("log_buf over 2G is not supported.\n");
	}

	if (size)
		size = roundup_pow_of_two(size);
	if (size > log_buf_len)
		new_log_buf_len = (unsigned long)size;
}

/* save requested log_buf_len since it's too early to process it */
static int __init log_buf_len_setup(char *str)
{
	u64 size;

	if (!str)
		return -EINVAL;

	size = memparse(str, &str);

	log_buf_len_update(size);

	return 0;
}
early_param("log_buf_len", log_buf_len_setup);

#ifdef CONFIG_SMP
#define __LOG_CPU_MAX_BUF_LEN (1 << CONFIG_LOG_CPU_MAX_BUF_SHIFT)

static void __init log_buf_add_cpu(void)
{
	unsigned int cpu_extra;

	/*
	 * archs should set up cpu_possible_bits properly with
	 * set_cpu_possible() after setup_arch() but just in
	 * case lets ensure this is valid.
	 */
	if (num_possible_cpus() == 1)
		return;

	cpu_extra = (num_possible_cpus() - 1) * __LOG_CPU_MAX_BUF_LEN;

	/* by default this will only continue through for large > 64 CPUs */
	if (cpu_extra <= __LOG_BUF_LEN / 2)
		return;

	pr_info("log_buf_len individual max cpu contribution: %d bytes\n",
		__LOG_CPU_MAX_BUF_LEN);
	pr_info("log_buf_len total cpu_extra contributions: %d bytes\n",
		cpu_extra);
	pr_info("log_buf_len min size: %d bytes\n", __LOG_BUF_LEN);

	log_buf_len_update(cpu_extra + __LOG_BUF_LEN);
}
#else /* !CONFIG_SMP */
static inline void log_buf_add_cpu(void) {}
#endif /* CONFIG_SMP */
#endif /* 0 */

void __init setup_log_buf(int early)
{
/* FIXME: no support for buffer resizing */
#if 0
	unsigned long flags;
	char *new_log_buf;
	unsigned int free;

	if (log_buf != __log_buf)
		return;

	if (!early && !new_log_buf_len)
		log_buf_add_cpu();

	if (!new_log_buf_len)
		return;

	new_log_buf = memblock_alloc(new_log_buf_len, LOG_ALIGN);
	if (unlikely(!new_log_buf)) {
		pr_err("log_buf_len: %lu bytes not available\n",
			new_log_buf_len);
		return;
	}

	logbuf_lock_irqsave(flags);
	log_buf_len = new_log_buf_len;
	log_buf = new_log_buf;
	new_log_buf_len = 0;
	free = __LOG_BUF_LEN - log_next_idx;
	memcpy(log_buf, __log_buf, __LOG_BUF_LEN);
	logbuf_unlock_irqrestore(flags);

	pr_info("log_buf_len: %u bytes\n", log_buf_len);
	pr_info("early log buf free: %u(%u%%)\n",
		free, (free * 100) / __LOG_BUF_LEN);
#endif
}

static bool __read_mostly ignore_loglevel;

static int __init ignore_loglevel_setup(char *str)
{
	ignore_loglevel = true;
	pr_info("debug: ignoring loglevel setting.\n");

	return 0;
}

early_param("ignore_loglevel", ignore_loglevel_setup);
module_param(ignore_loglevel, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(ignore_loglevel,
		 "ignore loglevel setting (prints all kernel messages to the console)");

static bool suppress_message_printing(int level)
{
	return (level >= console_loglevel && !ignore_loglevel);
}

#ifdef CONFIG_BOOT_PRINTK_DELAY

static int boot_delay; /* msecs delay after each printk during bootup */
static unsigned long long loops_per_msec;	/* based on boot_delay */

static int __init boot_delay_setup(char *str)
{
	unsigned long lpj;

	lpj = preset_lpj ? preset_lpj : 1000000;	/* some guess */
	loops_per_msec = (unsigned long long)lpj / 1000 * HZ;

	get_option(&str, &boot_delay);
	if (boot_delay > 10 * 1000)
		boot_delay = 0;

	pr_debug("boot_delay: %u, preset_lpj: %ld, lpj: %lu, "
		"HZ: %d, loops_per_msec: %llu\n",
		boot_delay, preset_lpj, lpj, HZ, loops_per_msec);
	return 0;
}
early_param("boot_delay", boot_delay_setup);

static void boot_delay_msec(int level)
{
	unsigned long long k;
	unsigned long timeout;

	if ((boot_delay == 0 || system_state >= SYSTEM_RUNNING)
		|| suppress_message_printing(level)) {
		return;
	}

	k = (unsigned long long)loops_per_msec * boot_delay;

	timeout = jiffies + msecs_to_jiffies(boot_delay);
	while (k) {
		k--;
		cpu_relax();
		/*
		 * use (volatile) jiffies to prevent
		 * compiler reduction; loop termination via jiffies
		 * is secondary and may or may not happen.
		 */
		if (time_after(jiffies, timeout))
			break;
		touch_nmi_watchdog();
	}
}
#else
static inline void boot_delay_msec(int level)
{
}
#endif

static bool printk_time = IS_ENABLED(CONFIG_PRINTK_TIME);
module_param_named(time, printk_time, bool, S_IRUGO | S_IWUSR);

static size_t print_cpu(u16 cpu, char *buf)
{
	return sprintf(buf, "%03hu: ", cpu);
}

static size_t print_syslog(unsigned int level, char *buf)
{
	return sprintf(buf, "<%u>", level);
}

static size_t print_time(u64 ts, char *buf)
{
	unsigned long rem_nsec = do_div(ts, 1000000000);

	return sprintf(buf, "[%5lu.%06lu]",
		       (unsigned long)ts, rem_nsec / 1000);
}

#ifdef CONFIG_PRINTK_CALLER
static size_t print_caller(u32 id, char *buf)
{
	char caller[12];

	snprintf(caller, sizeof(caller), "%c%u",
		 id & 0x80000000 ? 'C' : 'T', id & ~0x80000000);
	return sprintf(buf, "[%6s]", caller);
}
#else
#define print_caller(id, buf) 0
#endif

static size_t print_prefix(const struct printk_log *msg, bool syslog,
			   bool time, char *buf)
{
	size_t len = 0;

	if (syslog)
		len = print_syslog((msg->facility << 3) | msg->level, buf);

	if (time)
		len += print_time(msg->ts_nsec, buf + len);

	len += print_caller(msg->caller_id, buf + len);

	if (IS_ENABLED(CONFIG_PRINTK_CALLER) || time) {
		buf[len++] = ' ';
		buf[len] = '\0';
	}
	len += print_cpu(msg->cpu, buf + len);

	return len;
}

static size_t msg_print_text(const struct printk_log *msg, bool syslog,
			     bool time, char *buf, size_t size)
{
	const char *text = log_text(msg);
	size_t text_size = msg->text_len;
	size_t len = 0;
	char prefix[PREFIX_MAX];
	const size_t prefix_len = print_prefix(msg, syslog, time, prefix);

	do {
		const char *next = memchr(text, '\n', text_size);
		size_t text_len;

		if (next) {
			text_len = next - text;
			next++;
			text_size -= next - text;
		} else {
			text_len = text_size;
		}

		if (buf) {
			if (prefix_len + text_len + 1 >= size - len)
				break;

			memcpy(buf + len, prefix, prefix_len);
			len += prefix_len;
			memcpy(buf + len, text, text_len);
			len += text_len;
			buf[len++] = '\n';
		} else {
			/* SYSLOG_ACTION_* buffer size only calculation */
			len += prefix_len + text_len + 1;
		}

		text = next;
	} while (text);

	return len;
}

static int syslog_print(char __user *buf, int size, char *text,
			char *msgbuf, int *locked)
{
	struct prb_iterator iter;
	struct printk_log *msg;
	int len = 0;
	u64 seq;
	int ret;

	while (size > 0) {
		size_t n;
		size_t skip;

		for (;;) {
			prb_iter_copy(&iter, &syslog_iter);
			ret = prb_iter_next(&iter, msgbuf,
					    PRINTK_RECORD_MAX, &seq);
			if (ret < 0) {
				/* messages are gone, move to first one */
				prb_iter_init(&syslog_iter, &printk_rb,
					      &syslog_seq);
				syslog_partial = 0;
				continue;
			}
			break;
		}
		if (ret == 0)
			break;

		/*
		 * If messages have been missed, the partial tracker
		 * is no longer valid and must be reset.
		 */
		if (syslog_seq > 0 && seq - 1 != syslog_seq) {
			syslog_seq = seq - 1;
			syslog_partial = 0;
		}

		/*
		 * To keep reading/counting partial line consistent,
		 * use printk_time value as of the beginning of a line.
		 */
		if (!syslog_partial)
			syslog_time = printk_time;

		msg = (struct printk_log *)msgbuf;

		skip = syslog_partial;
		n = msg_print_text(msg, true, syslog_time, text,
				   PRINTK_SPRINT_MAX);
		if (n - syslog_partial <= size) {
			/* message fits into buffer, move forward */
			prb_iter_next(&syslog_iter, NULL, 0, &syslog_seq);
			n -= syslog_partial;
			syslog_partial = 0;
		} else if (!len) {
			/* partial read(), remember position */
			n = size;
			syslog_partial += n;
		} else
			n = 0;

		if (!n)
			break;

		mutex_unlock(&syslog_lock);
		if (copy_to_user(buf, text + skip, n)) {
			if (!len)
				len = -EFAULT;
			*locked = 0;
			break;
		}
		ret = mutex_lock_interruptible(&syslog_lock);

		len += n;
		size -= n;
		buf += n;

		if (ret) {
			if (!len)
				len = ret;
			*locked = 0;
			break;
		}
	}

	return len;
}

static int count_remaining(struct prb_iterator *iter, u64 until_seq,
			   char *msgbuf, int size, bool records, bool time)
{
	struct prb_iterator local_iter;
	struct printk_log *msg;
	int len = 0;
	u64 seq;
	int ret;

	prb_iter_copy(&local_iter, iter);
	for (;;) {
		ret = prb_iter_next(&local_iter, msgbuf, size, &seq);
		if (ret == 0) {
			break;
		} else if (ret < 0) {
			/* the iter is invalid, restart from head */
			prb_iter_init(&local_iter, &printk_rb, NULL);
			len = 0;
			continue;
		}

		if (until_seq && seq >= until_seq)
			break;

		if (records) {
			len++;
		} else {
			msg = (struct printk_log *)msgbuf;
			len += msg_print_text(msg, true, time, NULL, 0);
		}
	}

	return len;
}

static void syslog_clear(void)
{
	struct prb_iterator iter;
	int ret;

	prb_iter_init(&iter, &printk_rb, &clear_seq);
	for (;;) {
		ret = prb_iter_next(&iter, NULL, 0, &clear_seq);
		if (ret == 0)
			break;
		else if (ret < 0)
			prb_iter_init(&iter, &printk_rb, &clear_seq);
	}
}

static int syslog_print_all(char __user *buf, int size, bool clear)
{
	struct prb_iterator iter;
	struct printk_log *msg;
	char *msgbuf = NULL;
	char *text = NULL;
	int textlen;
	u64 seq = 0;
	int len = 0;
	bool time;
	int ret;

	text = kmalloc(PRINTK_SPRINT_MAX, GFP_KERNEL);
	if (!text)
		return -ENOMEM;
	msgbuf = kmalloc(PRINTK_RECORD_MAX, GFP_KERNEL);
	if (!msgbuf) {
		kfree(text);
		return -ENOMEM;
	}

	time = printk_time;

	/*
	 * Setup iter to last event before clear. Clear may
	 * be lost, but keep going with a best effort.
	 */
	prb_iter_init(&iter, &printk_rb, NULL);
	prb_iter_seek(&iter, clear_seq);

	/* count the total bytes after clear */
	len = count_remaining(&iter, 0, msgbuf, PRINTK_RECORD_MAX,
			      false, time);

	/* move iter forward until length fits into the buffer */
	while (len > size) {
		ret = prb_iter_next(&iter, msgbuf,
				    PRINTK_RECORD_MAX, &seq);
		if (ret == 0) {
			break;
		} else if (ret < 0) {
			/*
			 * The iter is now invalid so clear will
			 * also be invalid. Restart from the head.
			 */
			prb_iter_init(&iter, &printk_rb, NULL);
			len = count_remaining(&iter, 0, msgbuf,
					      PRINTK_RECORD_MAX, false, time);
			continue;
		}

		msg = (struct printk_log *)msgbuf;
		len -= msg_print_text(msg, true, time, NULL, 0);

		if (clear)
			clear_seq = seq;
	}

	/* copy messages to buffer */
	len = 0;
	while (len >= 0 && len < size) {
		if (clear)
			clear_seq = seq;

		ret = prb_iter_next(&iter, msgbuf,
				    PRINTK_RECORD_MAX, &seq);
		if (ret == 0) {
			break;
		} else if (ret < 0) {
			/*
			 * The iter is now invalid. Make a best
			 * effort to grab the rest of the log
			 * from the new head.
			 */
			prb_iter_init(&iter, &printk_rb, NULL);
			continue;
		}

		msg = (struct printk_log *)msgbuf;
		textlen = msg_print_text(msg, true, time, text,
					 PRINTK_SPRINT_MAX);
		if (textlen < 0) {
			len = textlen;
			break;
		}

		if (len + textlen > size)
			break;

		if (copy_to_user(buf + len, text, textlen))
			len = -EFAULT;
		else
			len += textlen;
	}

	if (clear && !seq)
		syslog_clear();

	if (text)
		kfree(text);
	if (msgbuf)
		kfree(msgbuf);
	return len;
}

int do_syslog(int type, char __user *buf, int len, int source)
{
	bool clear = false;
	static int saved_console_loglevel = LOGLEVEL_DEFAULT;
	struct prb_iterator iter;
	char *msgbuf = NULL;
	char *text = NULL;
	int locked;
	int error;
	int ret;

	error = check_syslog_permissions(type, source);
	if (error)
		return error;

	switch (type) {
	case SYSLOG_ACTION_CLOSE:	/* Close log */
		break;
	case SYSLOG_ACTION_OPEN:	/* Open log */
		break;
	case SYSLOG_ACTION_READ:	/* Read from log */
		if (!buf || len < 0)
			return -EINVAL;
		if (!len)
			return 0;
		if (!access_ok(buf, len))
			return -EFAULT;

		text = kmalloc(PRINTK_SPRINT_MAX, GFP_KERNEL);
		msgbuf = kmalloc(PRINTK_RECORD_MAX, GFP_KERNEL);
		if (!text || !msgbuf) {
			error = -ENOMEM;
			goto out;
		}

		error = mutex_lock_interruptible(&syslog_lock);
		if (error)
			goto out;

		/*
		 * Wait until a first message is available. Use a copy
		 * because no iteration should occur for syslog now.
		 */
		for (;;) {
			prb_iter_copy(&iter, &syslog_iter);

			mutex_unlock(&syslog_lock);
			ret = prb_iter_wait_next(&iter, NULL, 0, NULL);
			if (ret == -ERESTARTSYS) {
				error = ret;
				goto out;
			}
			error = mutex_lock_interruptible(&syslog_lock);
			if (error)
				goto out;

			if (ret == -EINVAL) {
				prb_iter_init(&syslog_iter, &printk_rb,
					      &syslog_seq);
				syslog_partial = 0;
				continue;
			}
			break;
		}

		/* print as much as will fit in the user buffer */
		locked = 1;
		error = syslog_print(buf, len, text, msgbuf, &locked);
		if (locked)
			mutex_unlock(&syslog_lock);
		break;
	/* Read/clear last kernel messages */
	case SYSLOG_ACTION_READ_CLEAR:
		clear = true;
		/* FALL THRU */
	/* Read last kernel messages */
	case SYSLOG_ACTION_READ_ALL:
		if (!buf || len < 0)
			return -EINVAL;
		if (!len)
			return 0;
		if (!access_ok(buf, len))
			return -EFAULT;
		error = syslog_print_all(buf, len, clear);
		break;
	/* Clear ring buffer */
	case SYSLOG_ACTION_CLEAR:
		syslog_clear();
		break;
	/* Disable logging to console */
	case SYSLOG_ACTION_CONSOLE_OFF:
		if (saved_console_loglevel == LOGLEVEL_DEFAULT)
			saved_console_loglevel = console_loglevel;
		console_loglevel = minimum_console_loglevel;
		break;
	/* Enable logging to console */
	case SYSLOG_ACTION_CONSOLE_ON:
		if (saved_console_loglevel != LOGLEVEL_DEFAULT) {
			console_loglevel = saved_console_loglevel;
			saved_console_loglevel = LOGLEVEL_DEFAULT;
		}
		break;
	/* Set level of messages printed to console */
	case SYSLOG_ACTION_CONSOLE_LEVEL:
		if (len < 1 || len > 8)
			return -EINVAL;
		if (len < minimum_console_loglevel)
			len = minimum_console_loglevel;
		console_loglevel = len;
		/* Implicitly re-enable logging to console */
		saved_console_loglevel = LOGLEVEL_DEFAULT;
		break;
	/* Number of chars in the log buffer */
	case SYSLOG_ACTION_SIZE_UNREAD:
		msgbuf = kmalloc(PRINTK_RECORD_MAX, GFP_KERNEL);
		if (!msgbuf)
			return -ENOMEM;

		error = mutex_lock_interruptible(&syslog_lock);
		if (error)
			goto out;

		if (source == SYSLOG_FROM_PROC) {
			/*
			 * Short-cut for poll(/"proc/kmsg") which simply checks
			 * for pending data, not the size; return the count of
			 * records, not the length.
			 */
			error = count_remaining(&syslog_iter, 0, msgbuf,
						PRINTK_RECORD_MAX, true,
						printk_time);
		} else {
			error = count_remaining(&syslog_iter, 0, msgbuf,
						PRINTK_RECORD_MAX, false,
						printk_time);
			error -= syslog_partial;
		}

		mutex_unlock(&syslog_lock);
		break;
	/* Size of the log buffer */
	case SYSLOG_ACTION_SIZE_BUFFER:
		error = prb_buffer_size(&printk_rb);
		break;
	default:
		error = -EINVAL;
		break;
	}
out:
	if (msgbuf)
		kfree(msgbuf);
	if (text)
		kfree(text);
	return error;
}

SYSCALL_DEFINE3(syslog, int, type, char __user *, buf, int, len)
{
	return do_syslog(type, buf, len, SYSLOG_FROM_READER);
}

int printk_delay_msec __read_mostly;

static inline void printk_delay(int level)
{
	boot_delay_msec(level);
	if (unlikely(printk_delay_msec)) {
		int m = printk_delay_msec;

		while (m--) {
			mdelay(1);
			touch_nmi_watchdog();
		}
	}
}

static void print_console_dropped(struct console *con, u64 count)
{
	char text[64];
	int len;

	len = sprintf(text, "** %llu printk message%s dropped **\n",
		      count, count > 1 ? "s" : "");
	con->write(con, text, len);
}

static void format_text(struct printk_log *msg, u64 seq,
			char *ext_text, size_t *ext_len,
			char *text, size_t *len, bool time)
{
	if (suppress_message_printing(msg->level)) {
		/*
		 * Skip record that has level above the console
		 * loglevel and update each console's local seq.
		 */
		*len = 0;
		*ext_len = 0;
		return;
	}

	*len = msg_print_text(msg, console_msg_format & MSG_FORMAT_SYSLOG,
			      time, text, PRINTK_SPRINT_MAX);
	if (nr_ext_console_drivers) {
		*ext_len = msg_print_ext_header(ext_text, CONSOLE_EXT_LOG_MAX,
						msg, seq);
		*ext_len += msg_print_ext_body(ext_text + *ext_len,
					       CONSOLE_EXT_LOG_MAX - *ext_len,
					       log_dict(msg), msg->dict_len,
					       log_text(msg), msg->text_len);
	} else {
		*ext_len = 0;
	}
}

static void printk_write_history(struct console *con, u64 master_seq)
{
	struct prb_iterator iter;
	bool time = printk_time;
	static char *ext_text;
	static char *text;
	static char *buf;
	u64 seq;

	ext_text = kmalloc(CONSOLE_EXT_LOG_MAX, GFP_KERNEL);
	text = kmalloc(PRINTK_SPRINT_MAX, GFP_KERNEL);
	buf = kmalloc(PRINTK_RECORD_MAX, GFP_KERNEL);
	if (!ext_text || !text || !buf)
		return;

	if (!(con->flags & CON_ENABLED))
		goto out;

	if (!con->write)
		goto out;

	if (!cpu_online(raw_smp_processor_id()) &&
	    !(con->flags & CON_ANYTIME))
		goto out;

	prb_iter_init(&iter, &printk_rb, NULL);

	for (;;) {
		struct printk_log *msg;
		size_t ext_len;
		size_t len;
		int ret;

		ret = prb_iter_next(&iter, buf, PRINTK_RECORD_MAX, &seq);
		if (ret == 0) {
			break;
		} else if (ret < 0) {
			prb_iter_init(&iter, &printk_rb, NULL);
			continue;
		}

		if (seq > master_seq)
			break;

		con->printk_seq++;
		if (con->printk_seq < seq) {
			print_console_dropped(con, seq - con->printk_seq);
			con->printk_seq = seq;
		}

		msg = (struct printk_log *)buf;
		format_text(msg, master_seq, ext_text, &ext_len, text,
			    &len, time);

		if (len == 0 && ext_len == 0)
			continue;

		if (con->flags & CON_EXTENDED)
			con->write(con, ext_text, ext_len);
		else
			con->write(con, text, len);

		printk_delay(msg->level);
	}
out:
	con->wrote_history = 1;
	kfree(ext_text);
	kfree(text);
	kfree(buf);
}

/*
 * Call the console drivers, asking them to write out
 * log_buf[start] to log_buf[end - 1].
 * The console_lock must be held.
 */
static void call_console_drivers(u64 seq, const char *ext_text, size_t ext_len,
				 const char *text, size_t len, int level,
				 int facility)
{
	struct console *con;

	trace_console_rcuidle(text, len);

	if (!console_drivers)
		return;

	for_each_console(con) {
		if (!(con->flags & CON_ENABLED))
			continue;
		if (!con->wrote_history) {
			if (con->flags & CON_PRINTBUFFER) {
				printk_write_history(con, seq);
				continue;
			}
			con->wrote_history = 1;
			con->printk_seq = seq - 1;
		}
		if (con->flags & CON_BOOT && facility == 0) {
			/* skip boot messages, already printed */
			if (con->printk_seq < seq)
				con->printk_seq = seq;
			continue;
		}
		if (!con->write)
			continue;
		if (!cpu_online(raw_smp_processor_id()) &&
		    !(con->flags & CON_ANYTIME))
			continue;
		if (con->printk_seq >= seq)
			continue;

		con->printk_seq++;
		if (con->printk_seq < seq) {
			print_console_dropped(con, seq - con->printk_seq);
			con->printk_seq = seq;
		}

		/* for supressed messages, only seq is updated */
		if (len == 0 && ext_len == 0)
			continue;

		if (con->flags & CON_EXTENDED)
			con->write(con, ext_text, ext_len);
		else
			con->write(con, text, len);
	}
}

static inline u32 printk_caller_id(void)
{
	return in_task() ? task_pid_nr(current) :
		0x80000000 + raw_smp_processor_id();
}

/*
 * Continuation lines are buffered, and not committed to the record buffer
 * until the line is complete, or a race forces it. The line fragments
 * though, are printed immediately to the consoles to ensure everything has
 * reached the console in case of a kernel crash.
 */
static struct cont {
	char buf[LOG_LINE_MAX];
	size_t len;			/* length == 0 means unused buffer */
	u32 caller_id;			/* printk_caller_id() of first print */
	int cpu_owner;			/* cpu of first print */
	u64 ts_nsec;			/* time of first print */
	u8 level;			/* log level of first message */
	u8 facility;			/* log facility of first message */
	enum log_flags flags;		/* prefix, newline flags */
} cont[2];

static void cont_flush(int ctx)
{
	struct cont *c = &cont[ctx];

	if (c->len == 0)
		return;

	log_store(c->caller_id, c->facility, c->level, c->flags,
		  c->ts_nsec, c->cpu_owner, NULL, 0, c->buf, c->len);
	c->len = 0;
}

static void cont_add(int ctx, int cpu, u32 caller_id, int facility, int level,
		     enum log_flags flags, const char *text, size_t len)
{
	struct cont *c = &cont[ctx];

	if (cpu != c->cpu_owner || !(flags & LOG_CONT))
		cont_flush(ctx);

	/* If the line gets too long, split it up in separate records. */
	while (c->len + len > sizeof(c->buf))
		cont_flush(ctx);

	if (!c->len) {
		c->facility = facility;
		c->level = level;
		c->caller_id = caller_id;
		c->ts_nsec = local_clock();
		c->flags = flags;
		c->cpu_owner = cpu;
	}

	memcpy(c->buf + c->len, text, len);
	c->len += len;

	// The original flags come from the first line,
	// but later continuations can add a newline.
	if (flags & LOG_NEWLINE) {
		c->flags |= LOG_NEWLINE;
		cont_flush(ctx);
	}
}

/* ring buffer used as memory allocator for temporary sprint buffers */
DECLARE_STATIC_PRINTKRB(sprint_rb,
			ilog2(PRINTK_RECORD_MAX + sizeof(struct prb_entry) +
			      sizeof(long)) + 2, &printk_cpulock);

asmlinkage int vprintk_emit(int facility, int level,
			    const char *dict, size_t dictlen,
			    const char *fmt, va_list args)
{
	const u32 caller_id = printk_caller_id();
	int ctx = !!in_nmi();
	enum log_flags lflags = 0;
	int printed_len = 0;
	struct prb_handle h;
	size_t text_len;
	u64 ts_nsec;
	char *text;
	char *rbuf;
	int cpu;

	ts_nsec = local_clock();

	rbuf = prb_reserve(&h, &sprint_rb, PRINTK_SPRINT_MAX);
	if (!rbuf) {
		prb_inc_lost(&printk_rb);
		return printed_len;
	}

	cpu = raw_smp_processor_id();

	/*
	 * If this turns out to be an emergency message, there
	 * may need to be a prefix added. Leave room for it.
	 */
	text = rbuf + PREFIX_MAX;
	text_len = vscnprintf(text, PRINTK_SPRINT_MAX - PREFIX_MAX, fmt, args);

	/* strip and flag a trailing newline */
	if (text_len && text[text_len-1] == '\n') {
		text_len--;
		lflags |= LOG_NEWLINE;
	}

	/* strip kernel syslog prefix and extract log level or control flags */
	if (facility == 0) {
		int kern_level;

		while ((kern_level = printk_get_level(text)) != 0) {
			switch (kern_level) {
			case '0' ... '7':
				if (level == LOGLEVEL_DEFAULT)
					level = kern_level - '0';
				break;
			case 'c':	/* KERN_CONT */
				lflags |= LOG_CONT;
			}

			text_len -= 2;
			text += 2;
		}
	}

	if (level == LOGLEVEL_DEFAULT)
		level = default_message_loglevel;

	if (dict)
		lflags |= LOG_NEWLINE;

	/*
	 * NOTE:
	 * - rbuf points to beginning of allocated buffer
	 * - text points to beginning of text
	 * - there is room before text for prefix
	 */
	if (facility == 0) {
		/* only the kernel can create emergency messages */
		printk_emergency(rbuf, level & 7, ts_nsec, cpu, text, text_len);
	}

	if ((lflags & LOG_CONT) || !(lflags & LOG_NEWLINE)) {
		 cont_add(ctx, cpu, caller_id, facility, level, lflags, text, text_len);
		 printed_len = text_len;
	} else {
		if (cpu == cont[ctx].cpu_owner)
			cont_flush(ctx);
		printed_len = log_store(caller_id, facility, level, lflags, ts_nsec, cpu,
					dict, dictlen, text, text_len);
	}

	prb_commit(&h);
	return printed_len;
}
EXPORT_SYMBOL(vprintk_emit);

static __printf(1, 0) int vprintk_func(const char *fmt, va_list args)
{
	return vprintk_emit(0, LOGLEVEL_DEFAULT, NULL, 0, fmt, args);
}

asmlinkage int vprintk(const char *fmt, va_list args)
{
	return vprintk_func(fmt, args);
}
EXPORT_SYMBOL(vprintk);

int vprintk_default(const char *fmt, va_list args)
{
	int r;

#ifdef CONFIG_KGDB_KDB
	/* Allow to pass printk() to kdb but avoid a recursion. */
	if (unlikely(kdb_trap_printk && kdb_printf_cpu < 0)) {
		r = vkdb_printf(KDB_MSGSRC_PRINTK, fmt, args);
		return r;
	}
#endif
	r = vprintk_emit(0, LOGLEVEL_DEFAULT, NULL, 0, fmt, args);

	return r;
}
EXPORT_SYMBOL_GPL(vprintk_default);

/**
 * printk - print a kernel message
 * @fmt: format string
 *
 * This is printk(). It can be called from any context. We want it to work.
 *
 * We try to grab the console_lock. If we succeed, it's easy - we log the
 * output and call the console drivers.  If we fail to get the semaphore, we
 * place the output into the log buffer and return. The current holder of
 * the console_sem will notice the new output in console_unlock(); and will
 * send it to the consoles before releasing the lock.
 *
 * One effect of this deferred printing is that code which calls printk() and
 * then changes console_loglevel may break. This is because console_loglevel
 * is inspected when the actual printing occurs.
 *
 * See also:
 * printf(3)
 *
 * See the vsnprintf() documentation for format string extensions over C99.
 */
asmlinkage __visible int printk(const char *fmt, ...)
{
	va_list args;
	int r;

	va_start(args, fmt);
	r = vprintk_func(fmt, args);
	va_end(args);

	return r;
}
EXPORT_SYMBOL(printk);
#endif /* CONFIG_PRINTK */

#ifdef CONFIG_EARLY_PRINTK
struct console *early_console;

asmlinkage __visible void early_printk(const char *fmt, ...)
{
	va_list ap;
	char buf[512];
	int n;

	if (!early_console)
		return;

	va_start(ap, fmt);
	n = vscnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	early_console->write(early_console, buf, n);
}
#endif

static int __add_preferred_console(char *name, int idx, char *options,
				   char *brl_options)
{
	struct console_cmdline *c;
	int i;

	/*
	 *	See if this tty is not yet registered, and
	 *	if we have a slot free.
	 */
	for (i = 0, c = console_cmdline;
	     i < MAX_CMDLINECONSOLES && c->name[0];
	     i++, c++) {
		if (strcmp(c->name, name) == 0 && c->index == idx) {
			if (!brl_options)
				preferred_console = i;
			return 0;
		}
	}
	if (i == MAX_CMDLINECONSOLES)
		return -E2BIG;
	if (!brl_options)
		preferred_console = i;
	strlcpy(c->name, name, sizeof(c->name));
	c->options = options;
	braille_set_options(c, brl_options);

	c->index = idx;
	return 0;
}

static int __init console_msg_format_setup(char *str)
{
	if (!strcmp(str, "syslog"))
		console_msg_format = MSG_FORMAT_SYSLOG;
	if (!strcmp(str, "default"))
		console_msg_format = MSG_FORMAT_DEFAULT;
	return 1;
}
__setup("console_msg_format=", console_msg_format_setup);

/*
 * Set up a console.  Called via do_early_param() in init/main.c
 * for each "console=" parameter in the boot command line.
 */
static int __init console_setup(char *str)
{
	char buf[sizeof(console_cmdline[0].name) + 4]; /* 4 for "ttyS" */
	char *s, *options, *brl_options = NULL;
	int idx;

	if (_braille_console_setup(&str, &brl_options))
		return 1;

	/*
	 * Decode str into name, index, options.
	 */
	if (str[0] >= '0' && str[0] <= '9') {
		strcpy(buf, "ttyS");
		strncpy(buf + 4, str, sizeof(buf) - 5);
	} else {
		strncpy(buf, str, sizeof(buf) - 1);
	}
	buf[sizeof(buf) - 1] = 0;
	options = strchr(str, ',');
	if (options)
		*(options++) = 0;
#ifdef __sparc__
	if (!strcmp(str, "ttya"))
		strcpy(buf, "ttyS0");
	if (!strcmp(str, "ttyb"))
		strcpy(buf, "ttyS1");
#endif
	for (s = buf; *s; s++)
		if (isdigit(*s) || *s == ',')
			break;
	idx = simple_strtoul(s, NULL, 10);
	*s = 0;

	__add_preferred_console(buf, idx, options, brl_options);
	console_set_on_cmdline = 1;
	return 1;
}
__setup("console=", console_setup);

/**
 * add_preferred_console - add a device to the list of preferred consoles.
 * @name: device name
 * @idx: device index
 * @options: options for this console
 *
 * The last preferred console added will be used for kernel messages
 * and stdin/out/err for init.  Normally this is used by console_setup
 * above to handle user-supplied console arguments; however it can also
 * be used by arch-specific code either to override the user or more
 * commonly to provide a default console (ie from PROM variables) when
 * the user has not supplied one.
 */
int add_preferred_console(char *name, int idx, char *options)
{
	return __add_preferred_console(name, idx, options, NULL);
}

bool console_suspend_enabled = true;
EXPORT_SYMBOL(console_suspend_enabled);

static int __init console_suspend_disable(char *str)
{
	console_suspend_enabled = false;
	return 1;
}
__setup("no_console_suspend", console_suspend_disable);
module_param_named(console_suspend, console_suspend_enabled,
		bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(console_suspend, "suspend console during suspend"
	" and hibernate operations");

/**
 * suspend_console - suspend the console subsystem
 *
 * This disables printk() while we go into suspend states
 */
void suspend_console(void)
{
	if (!console_suspend_enabled)
		return;
	pr_info("Suspending console(s) (use no_console_suspend to debug)\n");
	console_lock();
	console_suspended = 1;
	up_console_sem();
}

void resume_console(void)
{
	if (!console_suspend_enabled)
		return;
	down_console_sem();
	console_suspended = 0;
	console_unlock();
}

/**
 * console_cpu_notify - print deferred console messages after CPU hotplug
 * @cpu: unused
 *
 * If printk() is called from a CPU that is not online yet, the messages
 * will be printed on the console only if there are CON_ANYTIME consoles.
 * This function is called when a new CPU comes online (or fails to come
 * up) or goes offline.
 */
static int console_cpu_notify(unsigned int cpu)
{
	if (!cpuhp_tasks_frozen) {
		/* If trylock fails, someone else is doing the printing */
		if (console_trylock())
			console_unlock();
	}
	return 0;
}

/**
 * console_lock - lock the console system for exclusive use.
 *
 * Acquires a lock which guarantees that the caller has
 * exclusive access to the console system and the console_drivers list.
 *
 * Can sleep, returns nothing.
 */
void console_lock(void)
{
	might_sleep();

	down_console_sem();
	if (console_suspended)
		return;
	console_locked = 1;
	console_may_schedule = 1;
}
EXPORT_SYMBOL(console_lock);

/**
 * console_trylock - try to lock the console system for exclusive use.
 *
 * Try to acquire a lock which guarantees that the caller has exclusive
 * access to the console system and the console_drivers list.
 *
 * returns 1 on success, and 0 on failure to acquire the lock.
 */
int console_trylock(void)
{
	if (down_trylock_console_sem())
		return 0;
	if (console_suspended) {
		up_console_sem();
		return 0;
	}
	console_locked = 1;
	console_may_schedule = 0;
	return 1;
}
EXPORT_SYMBOL(console_trylock);

int is_console_locked(void)
{
	return console_locked;
}
EXPORT_SYMBOL(is_console_locked);

/**
 * console_unlock - unlock the console system
 *
 * Releases the console_lock which the caller holds on the console system
 * and the console driver list.
 *
 * console_unlock(); may be called from any context.
 */
void console_unlock(void)
{
	if (console_suspended) {
		up_console_sem();
		return;
	}

	console_locked = 0;
	up_console_sem();
}
EXPORT_SYMBOL(console_unlock);

/**
 * console_conditional_schedule - yield the CPU if required
 *
 * If the console code is currently allowed to sleep, and
 * if this CPU should yield the CPU to another task, do
 * so here.
 *
 * Must be called within console_lock();.
 */
void __sched console_conditional_schedule(void)
{
	if (console_may_schedule)
		cond_resched();
}
EXPORT_SYMBOL(console_conditional_schedule);

void console_unblank(void)
{
	struct console *c;

	/*
	 * console_unblank can no longer be called in interrupt context unless
	 * oops_in_progress is set to 1..
	 */
	if (oops_in_progress) {
		if (down_trylock_console_sem() != 0)
			return;
	} else
		console_lock();

	console_locked = 1;
	console_may_schedule = 0;
	for_each_console(c)
		if ((c->flags & CON_ENABLED) && c->unblank)
			c->unblank();
	console_unlock();
}

/**
 * console_flush_on_panic - flush console content on panic
 * @mode: flush all messages in buffer or just the pending ones
 *
 * Immediately output all pending messages no matter what.
 */
void console_flush_on_panic(enum con_flush_mode mode)
{
	/*
	 * FIXME: This is currently a NOP. Emergency messages will have been
	 * printed, but what about if write_atomic is not available on the
	 * console? What if the printk kthread is still alive?
	 */
}

/*
 * Return the console tty driver structure and its associated index
 */
struct tty_driver *console_device(int *index)
{
	struct console *c;
	struct tty_driver *driver = NULL;

	console_lock();
	for_each_console(c) {
		if (!c->device)
			continue;
		driver = c->device(c, index);
		if (driver)
			break;
	}
	console_unlock();
	return driver;
}

/*
 * Prevent further output on the passed console device so that (for example)
 * serial drivers can disable console output before suspending a port, and can
 * re-enable output afterwards.
 */
void console_stop(struct console *console)
{
	console_lock();
	console->flags &= ~CON_ENABLED;
	console_unlock();
}
EXPORT_SYMBOL(console_stop);

void console_start(struct console *console)
{
	console_lock();
	console->flags |= CON_ENABLED;
	console_unlock();
}
EXPORT_SYMBOL(console_start);

static int __read_mostly keep_bootcon;

static int __init keep_bootcon_setup(char *str)
{
	keep_bootcon = 1;
	pr_info("debug: skip boot console de-registration.\n");

	return 0;
}

early_param("keep_bootcon", keep_bootcon_setup);

/*
 * The console driver calls this routine during kernel initialization
 * to register the console printing procedure with printk() and to
 * print any messages that were printed by the kernel before the
 * console driver was initialized.
 *
 * This can happen pretty early during the boot process (because of
 * early_printk) - sometimes before setup_arch() completes - be careful
 * of what kernel features are used - they may not be initialised yet.
 *
 * There are two types of consoles - bootconsoles (early_printk) and
 * "real" consoles (everything which is not a bootconsole) which are
 * handled differently.
 *  - Any number of bootconsoles can be registered at any time.
 *  - As soon as a "real" console is registered, all bootconsoles
 *    will be unregistered automatically.
 *  - Once a "real" console is registered, any attempt to register a
 *    bootconsoles will be rejected
 */
void register_console(struct console *newcon)
{
	int i;
	struct console *bcon = NULL;
	struct console_cmdline *c;
	static bool has_preferred;

	if (console_drivers)
		for_each_console(bcon)
			if (WARN(bcon == newcon,
					"console '%s%d' already registered\n",
					bcon->name, bcon->index))
				return;

	/*
	 * before we register a new CON_BOOT console, make sure we don't
	 * already have a valid console
	 */
	if (console_drivers && newcon->flags & CON_BOOT) {
		/* find the last or real console */
		for_each_console(bcon) {
			if (!(bcon->flags & CON_BOOT)) {
				pr_info("Too late to register bootconsole %s%d\n",
					newcon->name, newcon->index);
				return;
			}
		}
	}

	if (console_drivers && console_drivers->flags & CON_BOOT)
		bcon = console_drivers;

	if (!has_preferred || bcon || !console_drivers)
		has_preferred = preferred_console >= 0;

	/*
	 *	See if we want to use this console driver. If we
	 *	didn't select a console we take the first one
	 *	that registers here.
	 */
	if (!has_preferred) {
		if (newcon->index < 0)
			newcon->index = 0;
		if (newcon->setup == NULL ||
		    newcon->setup(newcon, NULL) == 0) {
			newcon->flags |= CON_ENABLED;
			if (newcon->device) {
				newcon->flags |= CON_CONSDEV;
				has_preferred = true;
			}
		}
	}

	/*
	 *	See if this console matches one we selected on
	 *	the command line.
	 */
	for (i = 0, c = console_cmdline;
	     i < MAX_CMDLINECONSOLES && c->name[0];
	     i++, c++) {
		if (!newcon->match ||
		    newcon->match(newcon, c->name, c->index, c->options) != 0) {
			/* default matching */
			BUILD_BUG_ON(sizeof(c->name) != sizeof(newcon->name));
			if (strcmp(c->name, newcon->name) != 0)
				continue;
			if (newcon->index >= 0 &&
			    newcon->index != c->index)
				continue;
			if (newcon->index < 0)
				newcon->index = c->index;

			if (_braille_register_console(newcon, c))
				return;

			if (newcon->setup &&
			    newcon->setup(newcon, c->options) != 0)
				break;
		}

		newcon->flags |= CON_ENABLED;
		if (i == preferred_console) {
			newcon->flags |= CON_CONSDEV;
			has_preferred = true;
		}
		break;
	}

	if (!(newcon->flags & CON_ENABLED))
		return;

	/*
	 * If we have a bootconsole, and are switching to a real console,
	 * don't print everything out again, since when the boot console, and
	 * the real console are the same physical device, it's annoying to
	 * see the beginning boot messages twice
	 */
	if (bcon && ((newcon->flags & (CON_CONSDEV | CON_BOOT)) == CON_CONSDEV))
		newcon->flags &= ~CON_PRINTBUFFER;

	/*
	 *	Put this console in the list - keep the
	 *	preferred driver at the head of the list.
	 */
	console_lock();
	if ((newcon->flags & CON_CONSDEV) || console_drivers == NULL) {
		newcon->next = console_drivers;
		console_drivers = newcon;
		if (newcon->next)
			newcon->next->flags &= ~CON_CONSDEV;
	} else {
		newcon->next = console_drivers->next;
		console_drivers->next = newcon;
	}

	if (newcon->flags & CON_EXTENDED)
		nr_ext_console_drivers++;

	console_unlock();
	console_sysfs_notify();

	/*
	 * By unregistering the bootconsoles after we enable the real console
	 * we get the "console xxx enabled" message on all the consoles -
	 * boot consoles, real consoles, etc - this is to ensure that end
	 * users know there might be something in the kernel's log buffer that
	 * went to the bootconsole (that they do not see on the real console)
	 *
	 * This message is also important because it will trigger the
	 * printk kthread to begin dumping the log buffer to the newly
	 * registered console.
	 */
	pr_info("%sconsole [%s%d] enabled\n",
		(newcon->flags & CON_BOOT) ? "boot" : "" ,
		newcon->name, newcon->index);
	if (bcon &&
	    ((newcon->flags & (CON_CONSDEV | CON_BOOT)) == CON_CONSDEV) &&
	    !keep_bootcon) {
		/* We need to iterate through all boot consoles, to make
		 * sure we print everything out, before we unregister them.
		 */
		for_each_console(bcon)
			if (bcon->flags & CON_BOOT)
				unregister_console(bcon);
	}
}
EXPORT_SYMBOL(register_console);

int unregister_console(struct console *console)
{
        struct console *a, *b;
	int res;

	pr_info("%sconsole [%s%d] disabled\n",
		(console->flags & CON_BOOT) ? "boot" : "" ,
		console->name, console->index);

	res = _braille_unregister_console(console);
	if (res)
		return res;

	res = 1;
	console_lock();
	if (console_drivers == console) {
		console_drivers=console->next;
		res = 0;
	} else if (console_drivers) {
		for (a=console_drivers->next, b=console_drivers ;
		     a; b=a, a=b->next) {
			if (a == console) {
				b->next = a->next;
				res = 0;
				break;
			}
		}
	}

	if (!res && (console->flags & CON_EXTENDED))
		nr_ext_console_drivers--;

	/*
	 * If this isn't the last console and it has CON_CONSDEV set, we
	 * need to set it on the next preferred console.
	 */
	if (console_drivers != NULL && console->flags & CON_CONSDEV)
		console_drivers->flags |= CON_CONSDEV;

	console->flags &= ~CON_ENABLED;
	console_unlock();
	console_sysfs_notify();
	return res;
}
EXPORT_SYMBOL(unregister_console);

/*
 * Initialize the console device. This is called *early*, so
 * we can't necessarily depend on lots of kernel help here.
 * Just do some early initializations, and do the complex setup
 * later.
 */
void __init console_init(void)
{
	int ret;
	initcall_t call;
	initcall_entry_t *ce;

	/* Setup the default TTY line discipline. */
	n_tty_init();

	/*
	 * set up the console device so that later boot sequences can
	 * inform about problems etc..
	 */
	ce = __con_initcall_start;
	trace_initcall_level("console");
	while (ce < __con_initcall_end) {
		call = initcall_from_entry(ce);
		trace_initcall_start(call);
		ret = call();
		trace_initcall_finish(call, ret);
		ce++;
	}
}

/*
 * Some boot consoles access data that is in the init section and which will
 * be discarded after the initcalls have been run. To make sure that no code
 * will access this data, unregister the boot consoles in a late initcall.
 *
 * If for some reason, such as deferred probe or the driver being a loadable
 * module, the real console hasn't registered yet at this point, there will
 * be a brief interval in which no messages are logged to the console, which
 * makes it difficult to diagnose problems that occur during this time.
 *
 * To mitigate this problem somewhat, only unregister consoles whose memory
 * intersects with the init section. Note that all other boot consoles will
 * get unregistred when the real preferred console is registered.
 */
static int __init printk_late_init(void)
{
	struct console *con;
	int ret;

	for_each_console(con) {
		if (!(con->flags & CON_BOOT))
			continue;

		/* Check addresses that might be used for enabled consoles. */
		if (init_section_intersects(con, sizeof(*con)) ||
		    init_section_contains(con->write, 0) ||
		    init_section_contains(con->read, 0) ||
		    init_section_contains(con->device, 0) ||
		    init_section_contains(con->unblank, 0) ||
		    init_section_contains(con->data, 0)) {
			/*
			 * Please, consider moving the reported consoles out
			 * of the init section.
			 */
			pr_warn("bootconsole [%s%d] uses init memory and must be disabled even before the real one is ready\n",
				con->name, con->index);
			unregister_console(con);
		}
	}
	ret = cpuhp_setup_state_nocalls(CPUHP_PRINTK_DEAD, "printk:dead", NULL,
					console_cpu_notify);
	WARN_ON(ret < 0);
	ret = cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN, "printk:online",
					console_cpu_notify, NULL);
	WARN_ON(ret < 0);
	return 0;
}
late_initcall(printk_late_init);

#if defined CONFIG_PRINTK
static int printk_kthread_func(void *data)
{
	struct prb_iterator iter;
	struct printk_log *msg;
	size_t ext_len;
	char *ext_text;
	u64 master_seq;
	size_t len;
	char *text;
	char *buf;
	int ret;

	ext_text = kmalloc(CONSOLE_EXT_LOG_MAX, GFP_KERNEL);
	text = kmalloc(PRINTK_SPRINT_MAX, GFP_KERNEL);
	buf = kmalloc(PRINTK_RECORD_MAX, GFP_KERNEL);
	if (!ext_text || !text || !buf)
		return -1;

	prb_iter_init(&iter, &printk_rb, NULL);

	/* the printk kthread never exits */
	for (;;) {
		ret = prb_iter_wait_next(&iter, buf,
					 PRINTK_RECORD_MAX, &master_seq);
		if (ret == -ERESTARTSYS) {
			continue;
		} else if (ret < 0) {
			/* iterator invalid, start over */
			prb_iter_init(&iter, &printk_rb, NULL);
			continue;
		}

		msg = (struct printk_log *)buf;
		format_text(msg, master_seq, ext_text, &ext_len, text,
			    &len, printk_time);

		console_lock();
		console_may_schedule = 0;
		call_console_drivers(master_seq, ext_text, ext_len, text, len,
				     msg->level, msg->facility);
		if (len > 0 || ext_len > 0)
			printk_delay(msg->level);
		console_unlock();
	}

	kfree(ext_text);
	kfree(text);
	kfree(buf);

	return 0;
}

static int __init init_printk_kthread(void)
{
	struct task_struct *thread;

	thread = kthread_run(printk_kthread_func, NULL, "printk");
	if (IS_ERR(thread)) {
		pr_err("printk: unable to create printing thread\n");
		return PTR_ERR(thread);
	}

	return 0;
}
late_initcall(init_printk_kthread);

static int vprintk_deferred(const char *fmt, va_list args)
{
	return vprintk_emit(0, LOGLEVEL_DEFAULT, NULL, 0, fmt, args);
}

int printk_deferred(const char *fmt, ...)
{
	va_list args;
	int r;

	va_start(args, fmt);
	r = vprintk_deferred(fmt, args);
	va_end(args);

	return r;
}

/*
 * printk rate limiting, lifted from the networking subsystem.
 *
 * This enforces a rate limit: not more than 10 kernel messages
 * every 5s to make a denial-of-service attack impossible.
 */
DEFINE_RATELIMIT_STATE(printk_ratelimit_state, 5 * HZ, 10);

int __printk_ratelimit(const char *func)
{
	return ___ratelimit(&printk_ratelimit_state, func);
}
EXPORT_SYMBOL(__printk_ratelimit);

/**
 * printk_timed_ratelimit - caller-controlled printk ratelimiting
 * @caller_jiffies: pointer to caller's state
 * @interval_msecs: minimum interval between prints
 *
 * printk_timed_ratelimit() returns true if more than @interval_msecs
 * milliseconds have elapsed since the last time printk_timed_ratelimit()
 * returned true.
 */
bool printk_timed_ratelimit(unsigned long *caller_jiffies,
			unsigned int interval_msecs)
{
	unsigned long elapsed = jiffies - *caller_jiffies;

	if (*caller_jiffies && elapsed <= msecs_to_jiffies(interval_msecs))
		return false;

	*caller_jiffies = jiffies;
	return true;
}
EXPORT_SYMBOL(printk_timed_ratelimit);

static DEFINE_SPINLOCK(dump_list_lock);
static LIST_HEAD(dump_list);

/**
 * kmsg_dump_register - register a kernel log dumper.
 * @dumper: pointer to the kmsg_dumper structure
 *
 * Adds a kernel log dumper to the system. The dump callback in the
 * structure will be called when the kernel oopses or panics and must be
 * set. Returns zero on success and %-EINVAL or %-EBUSY otherwise.
 */
int kmsg_dump_register(struct kmsg_dumper *dumper)
{
	unsigned long flags;
	int err = -EBUSY;

	/* The dump callback needs to be set */
	if (!dumper->dump)
		return -EINVAL;

	spin_lock_irqsave(&dump_list_lock, flags);
	/* Don't allow registering multiple times */
	if (!dumper->registered) {
		dumper->registered = 1;
		list_add_tail_rcu(&dumper->list, &dump_list);
		err = 0;
	}
	spin_unlock_irqrestore(&dump_list_lock, flags);

	return err;
}
EXPORT_SYMBOL_GPL(kmsg_dump_register);

/**
 * kmsg_dump_unregister - unregister a kmsg dumper.
 * @dumper: pointer to the kmsg_dumper structure
 *
 * Removes a dump device from the system. Returns zero on success and
 * %-EINVAL otherwise.
 */
int kmsg_dump_unregister(struct kmsg_dumper *dumper)
{
	unsigned long flags;
	int err = -EINVAL;

	spin_lock_irqsave(&dump_list_lock, flags);
	if (dumper->registered) {
		dumper->registered = 0;
		list_del_rcu(&dumper->list);
		err = 0;
	}
	spin_unlock_irqrestore(&dump_list_lock, flags);
	synchronize_rcu();

	return err;
}
EXPORT_SYMBOL_GPL(kmsg_dump_unregister);

static bool always_kmsg_dump;
module_param_named(always_kmsg_dump, always_kmsg_dump, bool, S_IRUGO | S_IWUSR);

/**
 * kmsg_dump - dump kernel log to kernel message dumpers.
 * @reason: the reason (oops, panic etc) for dumping
 *
 * Call each of the registered dumper's dump() callback, which can
 * retrieve the kmsg records with kmsg_dump_get_line() or
 * kmsg_dump_get_buffer().
 */
void kmsg_dump(enum kmsg_dump_reason reason)
{
	struct kmsg_dumper dumper_local;
	struct kmsg_dumper *dumper;

	if ((reason > KMSG_DUMP_OOPS) && !always_kmsg_dump)
		return;

	rcu_read_lock();
	list_for_each_entry_rcu(dumper, &dump_list, list) {
		if (dumper->max_reason && reason > dumper->max_reason)
			continue;

		/*
		 * use a local copy to avoid modifying the
		 * iterator used by any other cpus/contexts
		 */
		memcpy(&dumper_local, dumper, sizeof(dumper_local));

		/* initialize iterator with data about the stored records */
		dumper_local.active = true;
		kmsg_dump_rewind(&dumper_local);

		/* invoke dumper which will iterate over records */
		dumper_local.dump(&dumper_local, reason);
	}
	rcu_read_unlock();
}

/**
 * kmsg_dump_get_line_nolock - retrieve one kmsg log line (unlocked version)
 * @dumper: registered kmsg dumper
 * @syslog: include the "<4>" prefixes
 * @line: buffer to copy the line to
 * @size: maximum size of the buffer
 * @len: length of line placed into buffer
 *
 * Start at the beginning of the kmsg buffer, with the oldest kmsg
 * record, and copy one record into the provided buffer.
 *
 * Consecutive calls will return the next available record moving
 * towards the end of the buffer with the youngest messages.
 *
 * A return value of FALSE indicates that there are no more records to
 * read.
 *
 * The function is similar to kmsg_dump_get_line(), but grabs no locks.
 */
bool kmsg_dump_get_line_nolock(struct kmsg_dumper *dumper, bool syslog,
			       char *line, size_t size, size_t *len)
{
	struct prb_iterator iter;
	struct printk_log *msg;
	struct prb_handle h;
	bool cont = false;
	char *msgbuf;
	char *rbuf;
	size_t l;
	u64 seq;
	int ret;

	if (!dumper->active)
		return cont;

	rbuf = prb_reserve(&h, &sprint_rb, PRINTK_RECORD_MAX);
	if (!rbuf)
		return cont;
	msgbuf = rbuf;
retry:
	for (;;) {
		prb_iter_init(&iter, &printk_rb, &seq);

		if (dumper->line_seq == seq) {
			/* already where we want to be */
			break;
		} else if (dumper->line_seq < seq) {
			/* messages are gone, move to first available one */
			dumper->line_seq = seq;
			break;
		}

		ret = prb_iter_seek(&iter, dumper->line_seq);
		if (ret > 0) {
			/* seeked to line_seq */
			break;
		} else if (ret == 0) {
			/*
			 * The end of the list was hit without ever seeing
			 * line_seq. Reset it to the beginning of the list.
			 */
			prb_iter_init(&iter, &printk_rb, &dumper->line_seq);
			break;
		}
		/* iterator invalid, start over */
	}

	ret = prb_iter_next(&iter, msgbuf, PRINTK_RECORD_MAX,
			    &dumper->line_seq);
	if (ret == 0)
		goto out;
	else if (ret < 0)
		goto retry;

	msg = (struct printk_log *)msgbuf;
	l = msg_print_text(msg, syslog, printk_time, line, size);

	if (len)
		*len = l;
	cont = true;
out:
	prb_commit(&h);
	return cont;
}

/**
 * kmsg_dump_get_line - retrieve one kmsg log line
 * @dumper: registered kmsg dumper
 * @syslog: include the "<4>" prefixes
 * @line: buffer to copy the line to
 * @size: maximum size of the buffer
 * @len: length of line placed into buffer
 *
 * Start at the beginning of the kmsg buffer, with the oldest kmsg
 * record, and copy one record into the provided buffer.
 *
 * Consecutive calls will return the next available record moving
 * towards the end of the buffer with the youngest messages.
 *
 * A return value of FALSE indicates that there are no more records to
 * read.
 */
bool kmsg_dump_get_line(struct kmsg_dumper *dumper, bool syslog,
			char *line, size_t size, size_t *len)
{
	bool ret;

	ret = kmsg_dump_get_line_nolock(dumper, syslog, line, size, len);

	return ret;
}
EXPORT_SYMBOL_GPL(kmsg_dump_get_line);

/**
 * kmsg_dump_get_buffer - copy kmsg log lines
 * @dumper: registered kmsg dumper
 * @syslog: include the "<4>" prefixes
 * @buf: buffer to copy the line to
 * @size: maximum size of the buffer
 * @len: length of line placed into buffer
 *
 * Start at the end of the kmsg buffer and fill the provided buffer
 * with as many of the the *youngest* kmsg records that fit into it.
 * If the buffer is large enough, all available kmsg records will be
 * copied with a single call.
 *
 * Consecutive calls will fill the buffer with the next block of
 * available older records, not including the earlier retrieved ones.
 *
 * A return value of FALSE indicates that there are no more records to
 * read.
 */
bool kmsg_dump_get_buffer(struct kmsg_dumper *dumper, bool syslog,
			  char *buf, size_t size, size_t *len)
{
	struct prb_iterator iter;
	bool time = printk_time;
	struct printk_log *msg;
	u64 new_end_seq = 0;
	struct prb_handle h;
	bool cont = false;
	char *msgbuf;
	u64 end_seq;
	int textlen;
	u64 seq = 0;
	char *rbuf;
	int l = 0;
	int ret;

	if (!dumper->active)
		return cont;

	rbuf = prb_reserve(&h, &sprint_rb, PRINTK_RECORD_MAX);
	if (!rbuf)
		return cont;
	msgbuf = rbuf;

	prb_iter_init(&iter, &printk_rb, NULL);

	/*
	 * seek to the start record, which is set/modified
	 * by kmsg_dump_get_line_nolock()
	 */
	ret = prb_iter_seek(&iter, dumper->line_seq);
	if (ret <= 0)
		prb_iter_init(&iter, &printk_rb, &seq);

	/* work with a local end seq to have a constant value */
	end_seq = dumper->buffer_end_seq;
	if (!end_seq) {
		/* initialize end seq to "infinity" */
		end_seq = -1;
		dumper->buffer_end_seq = end_seq;
	}
retry:
	if (seq >= end_seq)
		goto out;

	/* count the total bytes after seq */
	textlen = count_remaining(&iter, end_seq, msgbuf,
				  PRINTK_RECORD_MAX, 0, time);

	/* move iter forward until length fits into the buffer */
	while (textlen > size) {
		ret = prb_iter_next(&iter, msgbuf, PRINTK_RECORD_MAX, &seq);
		if (ret == 0) {
			break;
		} else if (ret < 0 || seq >= end_seq) {
			prb_iter_init(&iter, &printk_rb, &seq);
			goto retry;
		}

		msg = (struct printk_log *)msgbuf;
		textlen -= msg_print_text(msg, true, time, NULL, 0);
	}

	/* save end seq for the next interation */
	new_end_seq = seq + 1;

	/* copy messages to buffer */
	while (l < size) {
		ret = prb_iter_next(&iter, msgbuf, PRINTK_RECORD_MAX, &seq);
		if (ret == 0) {
			break;
		} else if (ret < 0) {
			/*
			 * iterator (and thus also the start position)
			 * invalid, start over from beginning of list
			 */
			prb_iter_init(&iter, &printk_rb, NULL);
			continue;
		}

		if (seq >= end_seq)
			break;

		msg = (struct printk_log *)msgbuf;
		textlen = msg_print_text(msg, syslog, time, buf + l, size - l);
		if (textlen > 0)
			l += textlen;
		cont = true;
	}

	if (cont && len)
		*len = l;
out:
	prb_commit(&h);
	if (new_end_seq)
		dumper->buffer_end_seq = new_end_seq;
	return cont;
}
EXPORT_SYMBOL_GPL(kmsg_dump_get_buffer);

/**
 * kmsg_dump_rewind_nolock - reset the interator (unlocked version)
 * @dumper: registered kmsg dumper
 *
 * Reset the dumper's iterator so that kmsg_dump_get_line() and
 * kmsg_dump_get_buffer() can be called again and used multiple
 * times within the same dumper.dump() callback.
 *
 * The function is similar to kmsg_dump_rewind(), but grabs no locks.
 */
void kmsg_dump_rewind_nolock(struct kmsg_dumper *dumper)
{
	dumper->line_seq = 0;
	dumper->buffer_end_seq = 0;
}

/**
 * kmsg_dump_rewind - reset the interator
 * @dumper: registered kmsg dumper
 *
 * Reset the dumper's iterator so that kmsg_dump_get_line() and
 * kmsg_dump_get_buffer() can be called again and used multiple
 * times within the same dumper.dump() callback.
 */
void kmsg_dump_rewind(struct kmsg_dumper *dumper)
{
	kmsg_dump_rewind_nolock(dumper);
}
EXPORT_SYMBOL_GPL(kmsg_dump_rewind);

static bool console_can_emergency(int level)
{
	struct console *con;

	for_each_console(con) {
		if (!(con->flags & CON_ENABLED))
			continue;
		if (con->write_atomic && oops_in_progress)
			return true;
		if (con->write && (con->flags & CON_BOOT))
			return true;
	}
	return false;
}

static void call_emergency_console_drivers(int level, const char *text,
					   size_t text_len)
{
	struct console *con;

	for_each_console(con) {
		if (!(con->flags & CON_ENABLED))
			continue;
		if (con->write_atomic && oops_in_progress) {
			con->write_atomic(con, text, text_len);
			continue;
		}
		if (con->write && (con->flags & CON_BOOT)) {
			con->write(con, text, text_len);
			continue;
		}
	}
}

static void printk_emergency(char *buffer, int level, u64 ts_nsec, u16 cpu,
			     char *text, u16 text_len)
{
	struct printk_log msg;
	size_t prefix_len;

	if (!console_can_emergency(level))
		return;

	msg.level = level;
	msg.ts_nsec = ts_nsec;
	msg.cpu = cpu;
	msg.facility = 0;

	/* "text" must have PREFIX_MAX preceding bytes available */

	prefix_len = print_prefix(&msg,
				  console_msg_format & MSG_FORMAT_SYSLOG,
				  printk_time, buffer);
	/* move the prefix forward to the beginning of the message text */
	text -= prefix_len;
	memmove(text, buffer, prefix_len);
	text_len += prefix_len;

	text[text_len++] = '\n';

	call_emergency_console_drivers(level, text, text_len);

	touch_softlockup_watchdog_sync();
	clocksource_touch_watchdog();
	rcu_cpu_stall_reset();
	touch_nmi_watchdog();

	printk_delay(level);
}
#endif

void console_atomic_lock(unsigned int *flags)
{
	prb_lock(&printk_cpulock, flags);
}
EXPORT_SYMBOL(console_atomic_lock);

void console_atomic_unlock(unsigned int flags)
{
	prb_unlock(&printk_cpulock, flags);
}
EXPORT_SYMBOL(console_atomic_unlock);
