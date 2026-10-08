// SPDX-License-Identifier: GPL-2.0
/*
 * mipp_pen0 - HyperOS MiuiBluetooth stylus channel shim (/dev/mipp_pen0)
 *
 * When the mipp feature flag is on, HyperOS's libbluetooth_jni.so opens
 * /dev/mipp_pen0 (O_WRONLY) for every HID connection and writes stylus
 * pressure-feedback reports (report_id 5) into it. On kernels without
 * Xiaomi's mipp driver, open() fails (ENOENT) and the fallback path blocks
 * forever inside AServiceManager_waitForService("vendor.xiaomi.hw.touchfeature
 * .ITouchFeature/default"), stalling the BTIF thread until the 10s
 * uhid_ready_disconn_timer tears the HID connection down. Providing this node
 * makes open() succeed so the HID pipeline (UHID_CREATE2 -> UHID_OPEN)
 * proceeds normally.
 *
 * Stylus connection reporting: every BT HID connection (incl. the stylus)
 * opens this node, so the first open marks "stylus channel connected" and the
 * last close marks "disconnected". A KOBJ_CHANGE uevent carrying
 * PEN_STATE=connected/disconnected is emitted on both transitions; the
 * userspace stylus helper (mippd) sleeps on the uevent socket and only does
 * work while a stylus is connected.
 *
 * The userspace ABI is deliberately tiny: open(O_WRONLY) must succeed and
 * write() must never fail and never block. read/poll/ioctl are a debug
 * convenience; the Bluetooth stack does not use them.
 *
 * Debug ring wire format (read side): a stream of frames, each 4-byte
 * little-endian length prefix followed by that many raw bytes.
 */

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <asm/unaligned.h>

#define MIPP_BUF_SIZE	(64 * 1024)	/* must be a power of 2 */
#define MIPP_BUF_MASK	(MIPP_BUF_SIZE - 1)
#define MIPP_MAX_RECORD	4096		/* single-write cap = staging buffer */
#define MIPP_FRAME_LEN	4		/* u32 LE record length prefix */

#define MIIPP_IOC_FLUSH	_IO('M', 0x50)

static u8 mipp_buf[MIPP_BUF_SIZE];
static u8 mipp_stage[MIPP_MAX_RECORD];
static u32 mipp_head, mipp_tail;	/* monotonic byte counters */
static u32 mipp_bytes;			/* bytes currently buffered */
static u64 mipp_writes, mipp_drops;
static unsigned long mipp_last_log;
static atomic_t mipp_opens = ATOMIC_INIT(0);
static struct kobject *mipp_kobj;	/* set once misc_register() succeeded */
static DEFINE_SPINLOCK(mipp_lock);
static DECLARE_WAIT_QUEUE_HEAD(mipp_readq);
static DEFINE_MUTEX(mipp_stage_lock);

static inline u32 ring_pos(u32 pos)
{
	return pos & MIPP_BUF_MASK;
}

/* Caller holds mipp_lock and guarantees the data fits. */
static void ring_put(const u8 *data, u32 len)
{
	u32 h = ring_pos(mipp_head);
	u32 first = min_t(u32, len, MIPP_BUF_SIZE - h);

	memcpy(mipp_buf + h, data, first);
	memcpy(mipp_buf, data + first, len - first);
	mipp_head += len;
	mipp_bytes += len;
}

/*
 * Drop the oldest frame. Caller holds mipp_lock and guarantees at least
 * MIPP_FRAME_LEN bytes are buffered.
 */
static void ring_drop_oldest(void)
{
	u32 flen = get_unaligned_le32(mipp_buf + ring_pos(mipp_tail));

	if (unlikely(flen > MIPP_BUF_SIZE - MIPP_FRAME_LEN)) {
		/* corrupted frame header: reset the whole ring */
		mipp_head = mipp_tail = mipp_bytes = 0;
		return;
	}
	mipp_tail += MIPP_FRAME_LEN + flen;
	mipp_bytes -= MIPP_FRAME_LEN + flen;
}

static void ring_reset(void)
{
	mipp_head = mipp_tail = mipp_bytes = 0;
}

static void mipp_announce(const char *state)
{
	static char *envp[] = { NULL, NULL };
	char buf[32];

	scnprintf(buf, sizeof(buf), "PEN_STATE=%s", state);
	envp[0] = buf;
	if (mipp_kobj)
		kobject_uevent_env(mipp_kobj, KOBJ_CHANGE, envp);
	pr_info("stylus channel %s\n", state);
}

static int mipp_open(struct inode *inode, struct file *file)
{
	int n = atomic_inc_return(&mipp_opens);

	if (n == 1)
		mipp_announce("connected");
	pr_info("opened by pid %d (%s), open count=%d\n",
		current->pid, current->comm, n);
	return nonseekable_open(inode, file);
}

static int mipp_release(struct inode *inode, struct file *file)
{
	if (atomic_dec_and_test(&mipp_opens))
		mipp_announce("disconnected");
	pr_info("released by pid %d (%s)\n", current->pid, current->comm);
	return 0;
}

static ssize_t mipp_write(struct file *file, const char __user *ubuf,
			  size_t len, loff_t *ppos)
{
	unsigned long flags;
	__le32 hdr;
	u32 need;
	u8 report_id;
	bool stored = false;

	if (!len)
		return 0;
	if (len > MIPP_MAX_RECORD) {
		/*
		 * Too big for the debug ring. The Bluetooth stack ignores the
		 * write result and only cares that the fd exists, so eat it.
		 */
		pr_info_ratelimited("mipp_pen0: dropped oversized write %zu\n",
				    len);
		return len;
	}

	/*
	 * The stage mutex serializes copy_from_user(); writers come from the
	 * Bluetooth stack one report at a time, contention is nil.
	 */
	mutex_lock(&mipp_stage_lock);
	if (copy_from_user(mipp_stage, ubuf, len)) {
		mutex_unlock(&mipp_stage_lock);
		return -EFAULT;
	}
	report_id = mipp_stage[0];

	need = MIPP_FRAME_LEN + (u32)len;
	spin_lock_irqsave(&mipp_lock, flags);
	while (MIPP_BUF_SIZE - mipp_bytes < need) {
		if (mipp_bytes < MIPP_FRAME_LEN) {
			ring_reset();
			break;
		}
		ring_drop_oldest();
		mipp_drops++;
	}
	if (MIPP_BUF_SIZE - mipp_bytes >= need) {
		hdr = cpu_to_le32((u32)len);
		ring_put((u8 *)&hdr, MIPP_FRAME_LEN);
		ring_put(mipp_stage, (u32)len);
		stored = true;
	}
	spin_unlock_irqrestore(&mipp_lock, flags);
	mutex_unlock(&mipp_stage_lock);

	wake_up_interruptible(&mipp_readq);

	mipp_writes++;
	if (time_after(jiffies, READ_ONCE(mipp_last_log) + HZ)) {
		WRITE_ONCE(mipp_last_log, jiffies);
		pr_info("write len=%zu report_id=%u stored=%d writes=%llu drops=%llu\n",
			len, report_id, stored, mipp_writes, mipp_drops);
	}

	/* Always report success: the stack treats -EIO as a degraded pen. */
	return len;
}

static ssize_t mipp_read(struct file *file, char __user *ubuf,
			 size_t count, loff_t *ppos)
{
	unsigned long flags;
	u8 *tmp;
	u32 taken;
	ssize_t copied;

	if (!count)
		return 0;

	spin_lock_irqsave(&mipp_lock, flags);
	while (!mipp_bytes) {
		spin_unlock_irqrestore(&mipp_lock, flags);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(mipp_readq, READ_ONCE(mipp_bytes)))
			return -ERESTARTSYS;
		spin_lock_irqsave(&mipp_lock, flags);
	}
	taken = min_t(u32, count, mipp_bytes);
	spin_unlock_irqrestore(&mipp_lock, flags);

	tmp = kmalloc(taken, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	spin_lock_irqsave(&mipp_lock, flags);
	taken = min_t(u32, taken, mipp_bytes);
	{
		u32 t = ring_pos(mipp_tail);
		u32 first = min_t(u32, taken, MIPP_BUF_SIZE - t);

		memcpy(tmp, mipp_buf + t, first);
		memcpy(tmp + first, mipp_buf, taken - first);
		mipp_tail += taken;
		mipp_bytes -= taken;
	}
	spin_unlock_irqrestore(&mipp_lock, flags);

	copied = taken - copy_to_user(ubuf, tmp, taken);
	kfree(tmp);

	return copied;
}

static __poll_t mipp_poll(struct file *file, poll_table *wait)
{
	__poll_t mask = POLLOUT | POLLWRNORM;

	poll_wait(file, &mipp_readq, wait);
	if (READ_ONCE(mipp_bytes))
		mask |= POLLIN | POLLRDNORM;
	return mask;
}

static long mipp_unlocked_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	unsigned long flags;

	if (cmd != MIIPP_IOC_FLUSH)
		return -ENOTTY;

	spin_lock_irqsave(&mipp_lock, flags);
	ring_reset();
	spin_unlock_irqrestore(&mipp_lock, flags);
	return 0;
}

static const struct file_operations mipp_fops = {
	.owner = THIS_MODULE,
	.open = mipp_open,
	.release = mipp_release,
	.llseek = no_llseek,
	.read = mipp_read,
	.write = mipp_write,
	.poll = mipp_poll,
	.unlocked_ioctl = mipp_unlocked_ioctl,
};

static struct miscdevice mipp_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mipp_pen0",
	.fops = &mipp_fops,
	.mode = 0660,
};

static ssize_t mipp_state_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&mipp_opens) > 0);
}
static DEVICE_ATTR(state, 0444, mipp_state_show, NULL);

static int __init mipp_init(void)
{
	int ret;

	ret = misc_register(&mipp_miscdev);
	if (ret) {
		pr_err("failed to register misc device: %d\n", ret);
		return ret;
	}
	mipp_kobj = &mipp_miscdev.this_device->kobj;
	if (device_create_file(mipp_miscdev.this_device, &dev_attr_state))
		pr_warn("failed to create state attribute\n");
	pr_info("mipp channel ready: /dev/mipp_pen0\n");
	return 0;
}

static void __exit mipp_exit(void)
{
	if (mipp_miscdev.this_device)
		device_remove_file(mipp_miscdev.this_device, &dev_attr_state);
	misc_deregister(&mipp_miscdev);
	wake_up_interruptible_all(&mipp_readq);
}

module_init(mipp_init);
module_exit(mipp_exit);

MODULE_DESCRIPTION("HyperOS MiuiBluetooth mipp stylus channel shim (/dev/mipp_pen0)");
MODULE_AUTHOR("TB375FC port project");
MODULE_LICENSE("GPL");
