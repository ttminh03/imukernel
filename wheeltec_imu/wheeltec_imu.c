// SPDX-License-Identifier: GPL-2.0
/*
 * wheeltec_imu - USB driver for WheelTec N100 IMU (CP2102 USB-UART bridge)
 *
 * Borrows CP210x vendor-specific control transfer commands from
 * drivers/usb/serial/cp210x.c but does NOT inherit usb_serial framework.
 * Bypasses TTY layer entirely; parses N100 frames in kernel and exposes:
 *
 *   /dev/wheeltec_imuN_raw     read-only, raw IMU (gyro + accel + mag)
 *   /dev/wheeltec_imuN_filter  read-only, AHRS filtered (euler + quaternion)
 *   /dev/wheeltec_imuN_state   read-only, text status
 *   /dev/wheeltec_imuN_ctrl    write-only, control commands (e.g. "freboot")
 *
 * Targeted at kernel 6.12+, PREEMPT_RT-safe.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/seqlock.h>
#include <linux/wait.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/idr.h>
#include <linux/string.h>
#include <linux/workqueue.h>

/* ---------- Module parameters ---------- */

static unsigned int default_baud = 921600;
module_param(default_baud, uint, 0644);
MODULE_PARM_DESC(default_baud, "UART baud rate (default 921600)");

/* ---------- USB ID table ---------- */

#define WHEELTEC_VID		0x10c4
#define WHEELTEC_PID_N100	0xE100

static const struct usb_device_id wheeltec_id_table[] = {
	{ USB_DEVICE(WHEELTEC_VID, WHEELTEC_PID_N100) },
	{ }
};
MODULE_DEVICE_TABLE(usb, wheeltec_id_table);

/* ---------- CP210x vendor-specific control commands ---------- */

#define CP210X_IFC_ENABLE	0x00
#define CP210X_SET_LINE_CTL	0x03
#define CP210X_SET_MHS		0x07
#define CP210X_PURGE		0x12
#define CP210X_SET_BAUDRATE	0x1E

#define UART_ENABLE		0x0001
#define UART_DISABLE		0x0000
#define PURGE_ALL		0x000F
#define MHS_DTR_RTS_ON		0x0303
#define LINE_CTL_8N1		0x0800

#define CP210X_REQTYPE_HOST_TO_DEVICE	0x41

/* ---------- Frame protocol ---------- */

/* Renamed from FRAME_HEAD/FRAME_END to avoid collision with x86 asm/frame.h */
#define WT_FRAME_HEAD	0xFC
#define WT_FRAME_END	0xFD

#define TYPE_IMU	0x40
#define TYPE_AHRS	0x41
#define TYPE_INSGPS	0x42

#define IMU_LEN		0x38	/* 56 */
#define AHRS_LEN	0x30	/* 48 */
#define INSGPS_LEN	0x54	/* 84 */

#define MAX_PAYLOAD_LEN	INSGPS_LEN
#define MAX_FRAME_LEN	(8 + MAX_PAYLOAD_LEN)
#define FRAME_BUF_SIZE	(MAX_FRAME_LEN * 4)

/* ---------- ABI structs exposed via /dev ---------- */

/*
 * /dev/wheeltec_imuN_raw — from TYPE_IMU frame
 *
 * Maps IMUData_Packet_t (pack(1), 56 bytes):
 *   [0..11]  gyro_x/y/z        rad/s
 *   [12..23] accel_x/y/z       m/s²
 *   [24..35] mag_x/y/z         mG
 *   [36..47] imu_temp/pressure/press_temp  (dropped)
 *   [48..55] hw_timestamp      µs
 */
struct wt_raw_frame {
	__u64  ktime_ns;	/* ktime_get_real_ns() at frame end */
	__u32  seq;		/* monotonic frame counter */
	__u32  hz;		/* frame rate in Hz, computed from hw_timestamp */
	struct_group_attr(sensors, __packed,
		float  gyro_x;		/* rad/s */
		float  gyro_y;
		float  gyro_z;
		float  accel_x;		/* m/s² */
		float  accel_y;
		float  accel_z;
		float  mag_x;		/* mG */
		float  mag_y;
		float  mag_z;
	);
	__s64  hw_timestamp;	/* µs, from N100 firmware */
} __packed;
/* size = 8 + 4 + 4 + 36 + 8 = 60 bytes */

/*
 * /dev/wheeltec_imuN_filter — from TYPE_AHRS frame
 *
 * Maps AHRSData_Packet_t (pack(1), 48 bytes):
 *   [0..11]  roll/pitch/heading speed  rad/s
 *   [12..23] roll/pitch/heading        rad
 *   [24..39] qw/qx/qy/qz
 *   [40..47] hw_timestamp              µs
 */
struct wt_filter_frame {
	__u64  ktime_ns;	/* ktime_get_real_ns() at frame end */
	__u32  seq;		/* monotonic frame counter */
	__u32  hz;		/* frame rate in Hz, computed from hw_timestamp */
	struct_group_attr(data, __packed,
		float  roll_speed;	/* rad/s */
		float  pitch_speed;
		float  heading_speed;
		float  roll;		/* rad */
		float  pitch;
		float  heading;
		float  qw;
		float  qx;
		float  qy;
		float  qz;
		__s64  hw_timestamp;	/* µs, from N100 firmware */
	);
} __packed;
/* size = 8 + 40 + 4 + 8 = 60 bytes */

/* ---------- Per-device state ---------- */

struct wheeltec_dev {
	struct usb_device	*udev;
	struct usb_interface	*intf;

	/* Bulk IN */
	struct urb		*bulk_in_urb;
	unsigned char		*bulk_in_buf;
	size_t			bulk_in_size;
	__u8			bulk_in_addr;

	/* Bulk OUT */
	__u8			bulk_out_addr;

	/* Frame parser (single-producer: URB completion) */
	u8			fbuf[FRAME_BUF_SIZE];
	size_t			fbuf_len;

	/* Raw snapshot (TYPE_IMU) */
	seqcount_t		raw_seq;
	struct wt_raw_frame	raw_snap;
	atomic_t		has_raw;
	wait_queue_head_t	raw_wq;
	u32			raw_fseq;		/* frame counter for reader */
	s64			raw_last_hw_ts;		/* last hw_timestamp, µs */
	u64			raw_period_avg;		/* EMA of frame period, ns */

	/* Filter snapshot (TYPE_AHRS) */
	seqcount_t		filter_seq;
	struct wt_filter_frame	filter_snap;
	atomic_t		has_filter;
	wait_queue_head_t	filter_wq;
	u32			filter_fseq;		/* frame counter for reader */
	s64			filter_last_hw_ts;	/* last hw_timestamp, µs */
	u64			filter_period_avg;	/* EMA of frame period, ns */

	/* Stats */
	atomic64_t		raw_frames_ok;
	atomic64_t		filter_frames_ok;
	atomic64_t		frames_crc_err;
	atomic64_t		frames_dropped;

	/* Misc devices */
	struct miscdevice	mdev_raw;
	struct miscdevice	mdev_filter;
	struct miscdevice	mdev_state;
	struct miscdevice	mdev_ctrl;
	char			name_raw[32];
	char			name_filter[32];
	char			name_state[32];
	char			name_ctrl[32];

	int			id;

	/* URB error recovery */
	struct work_struct	urb_retry_work;
	atomic_t		urb_err_cnt;    /* consecutive transient errors */

	/* Firmware reboot (runs in system_wq to allow msleep) */
	struct work_struct	reboot_work;
	atomic_t		reboot_pending;
	/* 0=idle, 1=in-progress, 2=success, 3=warning(no stream) */
	atomic_t		reboot_status;
	/* Set while waiting for ASCII response after #fconfig */
	atomic_t		fconfig_pending;

	/* Firmware tare (runs in system_wq to allow msleep) */
	struct work_struct	tare_work;
	atomic_t		tare_pending;
	/* 0=idle, 1=in-progress, 2=success, 3=warning(no stream) */
	atomic_t		tare_status;
	/* tare type: 0=gyro, 1=accel, 2=level */
	int			tare_type;
	/* calibration wait time in ms (userspace-supplied) */
	int			tare_wait_ms;
	/* Set during entire tare command sequence to log all IMU responses */
	atomic_t		tare_cmd_pending;
	/* True after startup gyro-tare has run once — prevents re-tare on reconnect */
	bool			startup_tare_done;

	/* Manual gyro bias calibration.
	 * Collect 3s of samples in a workqueue (process context) to avoid
	 * float/SSE restrictions in softirq. Raw IEEE-754 bits are passed via
	 * atomic snapshot; soft-float decode (pure integer) computes mean bias.
	 * bias stored as s32 nrad/s (1e-9 rad/s).
	 */
	struct work_struct	calib_work;
	bool			gyro_calib_collecting;
	bool			gyro_calib_done;
	/* Raw float bits of latest gyro sample — written by softirq, read by calib_work */
	atomic_t		calib_gyro_x_bits;
	atomic_t		calib_gyro_y_bits;
	atomic_t		calib_gyro_z_bits;
	atomic_t		calib_sample_ready; /* new sample available */
	s64			gyro_sum_x;	/* nrad/s accumulator */
	s64			gyro_sum_y;
	s64			gyro_sum_z;
	u32			gyro_calib_count;
	s32			gyro_bias_x;	/* nrad/s, subtracted before publish */
	s32			gyro_bias_y;
	s32			gyro_bias_z;

	struct kref		kref;
	struct mutex		io_mutex;
	bool			disconnected;
};

#define to_wdev_raw(f)    container_of((f)->private_data, struct wheeltec_dev, mdev_raw)
#define to_wdev_filter(f) container_of((f)->private_data, struct wheeltec_dev, mdev_filter)
#define to_wdev_state(f)  container_of((f)->private_data, struct wheeltec_dev, mdev_state)
#define to_wdev_ctrl(f)   container_of((f)->private_data, struct wheeltec_dev, mdev_ctrl)

static DEFINE_IDA(wheeltec_ida);

/* ---------- CRC ---------- */

extern const u8  cp_crc8_table[256];
extern const u16 cp_crc16_table[256];

static u8 wheeltec_crc8(const u8 *data, size_t len)
{
	u8 crc = 0;
	size_t i;
	for (i = 0; i < len; i++)
		crc = cp_crc8_table[crc ^ data[i]];
	return crc;
}

static u16 wheeltec_crc16(const u8 *data, size_t len)
{
	u16 crc = 0;
	size_t i;
	for (i = 0; i < len; i++)
		crc = (crc << 8) ^ cp_crc16_table[((crc >> 8) ^ data[i]) & 0xff];
	return crc;
}

/* Forward declarations — defined later in this file */
static int cp210x_ctrl_out(struct usb_device *udev, u8 req, u16 val);
static int cp210x_setup(struct usb_device *udev, u32 baud);
static void wheeltec_read_complete(struct urb *urb);

/* ---------- Bulk OUT helper ---------- */

static int wt_bulk_write(struct wheeltec_dev *wd, const void *buf, size_t len)
{
	unsigned char *kbuf;
	int actual, ret;

	kbuf = kmemdup(buf, len, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	ret = usb_bulk_msg(wd->udev,
			   usb_sndbulkpipe(wd->udev, wd->bulk_out_addr),
			   kbuf, len, &actual, 2000);
	kfree(kbuf);
	return ret;
}

/* ---------- Firmware reboot sequence (workqueue context) ---------- */

/*
 * Mirrors ImuControl.RebootAsync() from C# (confirmed working):
 *
 *  Send("#fconfig")  → SendOn delay 300ms → await Delay(800ms)  → DiscardInBuffer
 *  Send("#freboot")  → SendOn delay 300ms → await Delay(1500ms) → DiscardInBuffer
 *  Send("y")         → SendOn delay 300ms → await Delay(800ms)
 *  _port.Close()
 *  await Delay(15s)
 *  OpenFreshPort (retry loop) → CountFrameHeads 12s
 *
 * Key insight: IMU reboots → USB disconnect → wheeltec_disconnect() fires →
 * wheeltec_probe() runs automatically and restores stream. reboot_work only
 * needs to send the commands then wait for disconnect. If disconnect happens,
 * probe handles everything — we just exit. If no disconnect after 20s, do
 * manual UART reinit as fallback.
 */
static void wt_reboot_work_fn(struct work_struct *work)
{
	struct wheeltec_dev *wd = container_of(work, struct wheeltec_dev, reboot_work);
	u64 frames_before, frames_after;
	int i, rc;

	atomic_set(&wd->reboot_status, 1);

	if (wd->disconnected)
		goto aborted;

	/* Stop binary stream before sending ASCII commands */
	usb_kill_urb(wd->bulk_in_urb);
	cp210x_ctrl_out(wd->udev, CP210X_IFC_ENABLE, UART_DISABLE);
	msleep(100);
	cp210x_ctrl_out(wd->udev, CP210X_PURGE, PURGE_ALL);
	msleep(50);
	wd->fbuf_len = 0;
	cp210x_ctrl_out(wd->udev, CP210X_IFC_ENABLE, UART_ENABLE);
	msleep(50);

	/* Send("#fconfig") → 300ms internal + 800ms delay → DiscardInBuffer */
	dev_info(&wd->intf->dev, "TX >> #fconfig\n");
	atomic_set(&wd->fconfig_pending, 1);
	rc = usb_submit_urb(wd->bulk_in_urb, GFP_KERNEL);
	if (rc && rc != -EPERM && !wd->disconnected)
		dev_warn(&wd->intf->dev, "freboot: URB submit rc=%d\n", rc);
	wt_bulk_write(wd, "#fconfig\r\n", 10);
	msleep(300 + 800); /* SendOn(300) + Delay(800) */
	atomic_set(&wd->fconfig_pending, 0);
	usb_kill_urb(wd->bulk_in_urb);
	cp210x_ctrl_out(wd->udev, CP210X_PURGE, PURGE_ALL);
	wd->fbuf_len = 0;

	if (wd->disconnected)
		goto aborted;

	/* Send("#freboot") → 300ms internal + 1500ms delay → DiscardInBuffer */
	dev_info(&wd->intf->dev, "TX >> #freboot\n");
	wt_bulk_write(wd, "#freboot\r\n", 10);
	msleep(300 + 1500); /* SendOn(300) + Delay(1500) */
	cp210x_ctrl_out(wd->udev, CP210X_PURGE, PURGE_ALL);

	if (wd->disconnected)
		goto aborted;

	/* Send("y") → 300ms internal + 800ms delay */
	dev_info(&wd->intf->dev, "TX >> y\n");
	wt_bulk_write(wd, "y\r\n", 3);
	msleep(300 + 800); /* SendOn(300) + Delay(800) */

	/* _port.Close() → disable UART */
	cp210x_ctrl_out(wd->udev, CP210X_IFC_ENABLE, UART_DISABLE);

	/*
	 * await Delay(15s): IMU reboots, USB disconnects, wheeltec_disconnect()
	 * fires → wd->disconnected = true. If that happens, probe() will
	 * re-bind automatically and restore the stream — we just exit.
	 */
	dev_info(&wd->intf->dev, "freboot: waiting up to 20s for USB disconnect/reconnect...\n");
	for (i = 0; i < 200; i++) {
		msleep(100);
		if (wd->disconnected) {
			dev_info(&wd->intf->dev, "freboot: USB disconnect detected — probe will restore stream\n");
			atomic_set(&wd->reboot_status, 2);
			atomic_set(&wd->reboot_pending, 0);
			return;
		}
	}

	/*
	 * No USB disconnect after 20s — IMU stayed connected (some firmware
	 * variants don't USB-disconnect on reboot). Do manual UART reinit
	 * as fallback (= OpenFreshPort in C#).
	 */
	dev_info(&wd->intf->dev, "freboot: no disconnect — manual UART reinit\n");
	rc = cp210x_setup(wd->udev, default_baud);
	if (rc)
		dev_warn(&wd->intf->dev, "freboot: cp210x_setup failed: %d\n", rc);

	wd->fbuf_len = 0;
	atomic_set(&wd->urb_err_cnt, 0);

	frames_before = atomic64_read(&wd->raw_frames_ok)
		      + atomic64_read(&wd->filter_frames_ok);

	atomic_set(&wd->reboot_pending, 0);

	rc = usb_submit_urb(wd->bulk_in_urb, GFP_KERNEL);
	if (rc && rc != -EPERM && !wd->disconnected) {
		dev_err(&wd->intf->dev, "freboot: URB submit failed: %d\n", rc);
		atomic_set(&wd->reboot_status, 3);
		return;
	}

	dev_info(&wd->intf->dev, "freboot: polling 12s for stream...\n");
	for (i = 0; i < 120 && !wd->disconnected; i++) {
		msleep(100);
		frames_after = atomic64_read(&wd->raw_frames_ok)
			     + atomic64_read(&wd->filter_frames_ok);
		if (frames_after > frames_before + 5) {
			dev_info(&wd->intf->dev, "freboot: SUCCESS — stream restored\n");
			atomic_set(&wd->reboot_status, 2);
			return;
		}
	}

	if (wd->disconnected)
		goto aborted;

	dev_warn(&wd->intf->dev, "freboot: WARNING — no stream after fallback reinit\n");
	atomic_set(&wd->reboot_status, 3);
	return;

	aborted:
		dev_info(&wd->intf->dev, "freboot: aborted (disconnected)\n");
		atomic_set(&wd->reboot_status, 0);
		atomic_set(&wd->reboot_pending, 0);
}

/* ---------- Tare sequence (workqueue context) ---------- */

/*
 * Mirrors RobotNet12 ImuControl.TareAsync() exactly.
 * SendOn() in C# adds 300ms internal delay, so each step timing = 300 + delay.
 *
 *  1. kill URB → disable UART → purge → flush → enable UART
 *  2. #fconfig    → 300+800ms  → arm URB → kill URB → purge
 *  3. #fimucal_*  → 300ms + tare_wait_ms
 *  4. #fsave      → 300+800ms
 *  5. #fconfig    → 300+800ms  → purge          (2nd time — ensure config mode)
 *  6. #freboot    → 300+1500ms → purge
 *  7. y           → 300+800ms
 *  8. disable UART → wait USB disconnect (probe restores stream)
 *     fallback: manual UART reinit → verify stream 15s
 */
static void wt_tare_work_fn(struct work_struct *work)
{
	struct wheeltec_dev *wd =
		container_of(work, struct wheeltec_dev, tare_work);
	u64 frames_before, frames_after;
	int i, rc;
	const char *cal_cmd;
	size_t cal_len;

	atomic_set(&wd->tare_status, 1); /* in-progress */
	atomic_set(&wd->tare_cmd_pending, 1);

	/* Tare changes sensor characteristics — old bias is no longer valid */
	wd->gyro_calib_done       = false;
	wd->gyro_calib_collecting = false;
	wd->gyro_bias_x = 0;
	wd->gyro_bias_y = 0;
	wd->gyro_bias_z = 0;

	if (wd->disconnected)
		goto aborted;

	switch (wd->tare_type) {
	case 0: cal_cmd = "#fimucal_gyro\r\n";  cal_len = 16; break;
	case 1: cal_cmd = "#fimucal_acce\r\n";  cal_len = 16; break;
	case 2: cal_cmd = "#fimucal_level\r\n"; cal_len = 17; break;
	default:
		dev_err(&wd->intf->dev, "tare: unknown type %d\n", wd->tare_type);
		goto aborted;
	}

	/* Stop binary stream: kill URB, disable UART, purge, re-enable */
	usb_kill_urb(wd->bulk_in_urb);
	cp210x_ctrl_out(wd->udev, CP210X_IFC_ENABLE, UART_DISABLE);
	msleep(100);
	cp210x_ctrl_out(wd->udev, CP210X_PURGE, PURGE_ALL);
	msleep(50);
	wd->fbuf_len = 0;
	cp210x_ctrl_out(wd->udev, CP210X_IFC_ENABLE, UART_ENABLE);
	msleep(50);

	if (wd->disconnected)
		goto aborted;

	/* Send("#fconfig") — arm URB to read ASCII response, then kill + purge */
	dev_info(&wd->intf->dev, "TX >> #fconfig\n");
	atomic_set(&wd->fconfig_pending, 1);
	rc = usb_submit_urb(wd->bulk_in_urb, GFP_KERNEL);
	if (rc && rc != -EPERM && !wd->disconnected)
		dev_warn(&wd->intf->dev, "tare: URB submit rc=%d\n", rc);
	wt_bulk_write(wd, "#fconfig\r\n", 10);
	msleep(300 + 800); /* SendOn(300) + Delay(800) */
	atomic_set(&wd->fconfig_pending, 0);
	usb_kill_urb(wd->bulk_in_urb);
	cp210x_ctrl_out(wd->udev, CP210X_PURGE, PURGE_ALL);
	wd->fbuf_len = 0;

	if (wd->disconnected)
		goto aborted;

	/* Send(calibCmd) */
	dev_info(&wd->intf->dev, "TX >> %s (wait %dms)\n", cal_cmd, wd->tare_wait_ms);
	wt_bulk_write(wd, cal_cmd, cal_len);
	msleep(300); /* SendOn internal */
	for (i = 0; i < wd->tare_wait_ms / 100 && !wd->disconnected; i++)
		msleep(100);

	if (wd->disconnected)
		goto aborted;

	/* Send("#fsave") */
	dev_info(&wd->intf->dev, "TX >> #fsave\n");
	wt_bulk_write(wd, "#fsave\r\n", 8);
	msleep(300 + 800); /* SendOn(300) + Delay(800) */

	if (wd->disconnected)
		goto aborted;

	/* Send("#fconfig") [2nd] — re-enter config mode before reboot */
	dev_info(&wd->intf->dev, "TX >> #fconfig (2nd)\n");
	wt_bulk_write(wd, "#fconfig\r\n", 10);
	msleep(300 + 800); /* SendOn(300) + Delay(800) */
	cp210x_ctrl_out(wd->udev, CP210X_PURGE, PURGE_ALL); /* DiscardInBuffer */
	wd->fbuf_len = 0;

	if (wd->disconnected)
		goto aborted;

	/* Send("#freboot") */
	dev_info(&wd->intf->dev, "TX >> #freboot\n");
	wt_bulk_write(wd, "#freboot\r\n", 10);
	msleep(300 + 1500); /* SendOn(300) + Delay(1500) */
	cp210x_ctrl_out(wd->udev, CP210X_PURGE, PURGE_ALL); /* DiscardInBuffer */

	if (wd->disconnected)
		goto aborted;

	/* Send("y") */
	dev_info(&wd->intf->dev, "TX >> y\n");
	wt_bulk_write(wd, "y\r\n", 3);
	msleep(300 + 800); /* SendOn(300) + Delay(800) */

	/* _port.Close() */
	cp210x_ctrl_out(wd->udev, CP210X_IFC_ENABLE, UART_DISABLE);

	/* Wait up to 15s for USB disconnect — probe() restores stream automatically */
	dev_info(&wd->intf->dev, "tare: waiting up to 15s for USB disconnect/reconnect...\n");
	for (i = 0; i < 150; i++) {
		msleep(100);
		if (wd->disconnected) {
			dev_info(&wd->intf->dev, "tare: USB disconnect detected — probe will restore stream\n");
			atomic_set(&wd->tare_cmd_pending, 0);
			atomic_set(&wd->tare_status, 2);
			atomic_set(&wd->tare_pending, 0);
			return;
		}
	}

	/* No USB disconnect — manual UART reinit (OpenFreshPort fallback) */
	dev_info(&wd->intf->dev, "tare: no disconnect — manual UART reinit\n");
	rc = cp210x_setup(wd->udev, default_baud);
	if (rc)
		dev_warn(&wd->intf->dev, "tare: cp210x_setup failed: %d\n", rc);

	wd->fbuf_len = 0;
	atomic_set(&wd->urb_err_cnt, 0);

	frames_before = atomic64_read(&wd->raw_frames_ok)
		      + atomic64_read(&wd->filter_frames_ok);

	atomic_set(&wd->tare_pending, 0);

	rc = usb_submit_urb(wd->bulk_in_urb, GFP_KERNEL);
	if (rc && rc != -EPERM && !wd->disconnected) {
		dev_err(&wd->intf->dev, "tare: URB submit failed: %d\n", rc);
		atomic_set(&wd->tare_cmd_pending, 0);
		atomic_set(&wd->tare_status, 3);
		return;
	}

	/* CountFrameHeads 15s */
	dev_info(&wd->intf->dev, "tare: polling 15s for stream...\n");
	for (i = 0; i < 150 && !wd->disconnected; i++) {
		msleep(100);
		frames_after = atomic64_read(&wd->raw_frames_ok)
			     + atomic64_read(&wd->filter_frames_ok);
		if (frames_after > frames_before + 5) {
			dev_info(&wd->intf->dev, "tare: SUCCESS — stream restored\n");
			atomic_set(&wd->tare_cmd_pending, 0);
			atomic_set(&wd->tare_status, 2);
			wd->gyro_calib_collecting = true;
			schedule_work(&wd->calib_work);
			return;
		}
	}

	if (wd->disconnected)
		goto aborted;

	dev_warn(&wd->intf->dev, "tare: WARNING — no stream after reinit\n");
	atomic_set(&wd->tare_cmd_pending, 0);
	atomic_set(&wd->tare_status, 3);
	return;

	aborted:
		dev_info(&wd->intf->dev, "tare: aborted (disconnected)\n");
		atomic_set(&wd->tare_cmd_pending, 0);
		atomic_set(&wd->tare_status, 0);
		atomic_set(&wd->tare_pending, 0);
}

/* ---------- CP210x init helpers ---------- */

static int cp210x_ctrl_out(struct usb_device *udev, u8 req, u16 val)
{
	return usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			       req, CP210X_REQTYPE_HOST_TO_DEVICE,
			       val, 0, NULL, 0, USB_CTRL_SET_TIMEOUT);
}

static int cp210x_set_baud(struct usb_device *udev, u32 baud)
{
	__le32 *le_baud;
	int ret;

	le_baud = kmalloc(sizeof(*le_baud), GFP_KERNEL);
	if (!le_baud)
		return -ENOMEM;
	*le_baud = cpu_to_le32(baud);
	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			      CP210X_SET_BAUDRATE, CP210X_REQTYPE_HOST_TO_DEVICE,
			      0, 0, le_baud, sizeof(*le_baud),
			      USB_CTRL_SET_TIMEOUT);
	kfree(le_baud);
	return ret < 0 ? ret : 0;
}

static int cp210x_setup(struct usb_device *udev, u32 baud)
{
	int ret;

	ret = cp210x_ctrl_out(udev, CP210X_IFC_ENABLE, UART_ENABLE);
	if (ret)
		return ret;

	ret = cp210x_set_baud(udev, baud);
	if (ret)
		goto err;

	ret = cp210x_ctrl_out(udev, CP210X_SET_LINE_CTL, LINE_CTL_8N1);
	if (ret)
		goto err;

	ret = cp210x_ctrl_out(udev, CP210X_SET_MHS, MHS_DTR_RTS_ON);
	if (ret)
		goto err;

	ret = cp210x_ctrl_out(udev, CP210X_PURGE, PURGE_ALL);
	if (ret)
		goto err;

	return 0;
err:
	cp210x_ctrl_out(udev, CP210X_IFC_ENABLE, UART_DISABLE);
	return ret;
}

/* ---------- Frame parser ---------- */

static u8 expected_payload_len(u8 type)
{
	switch (type) {
	case TYPE_IMU:    return IMU_LEN;
	case TYPE_AHRS:   return AHRS_LEN;
	case TYPE_INSGPS: return INSGPS_LEN;
	default:          return 0;
	}
}


/* ---------- Soft-float helpers (pure integer, no SSE) ---------- */

/*
 * Decode IEEE 754 single-precision float bits → nrad/s (s32, 1e-9 rad/s).
 * Handles normal numbers only; subnormals/inf/nan treated as 0.
 * Precision: ~1 nrad/s (gyro values are typically < 0.1 rad/s at rest).
 */
static s32 f32bits_to_nrads(u32 bits)
{
	u32 sign     = bits >> 31;
	u32 exp      = (bits >> 23) & 0xFF;
	u32 mantissa = bits & 0x7FFFFF;
	s64 val;
	int shift;

	if (exp == 0 || exp == 0xFF)
		return 0; /* subnormal / inf / nan → treat as 0 */

	/* val = (1 + mantissa/2^23) × 2^(exp-127) × 1e9 */
	/* = (2^23 + mantissa) × 2^(exp-127-23) × 1e9    */
	val = (s64)(0x800000 | mantissa); /* 1.mantissa in Q23 */

	shift = (int)exp - 127 - 23; /* net power of 2 */

	/* Scale by 1e9 first (stays in s64 for gyro values < 100 rad/s) */
	val *= 1000000000LL;

	if (shift >= 0) {
		if (shift < 40)
			val <<= shift;
		else
			return sign ? S32_MIN : S32_MAX;
	} else {
		shift = -shift;
		if (shift < 63)
			val >>= shift;
		else
			val = 0;
	}

	return sign ? (s32)(-val) : (s32)(val);
}

/* Disabled — used only by bias correction which is currently off. */
#if 0
static u32 f32bits_sub_nrads(u32 bits, s32 bias_nrads)
{
	s32 val_nrads = f32bits_to_nrads(bits);
	s32 corrected = val_nrads - bias_nrads;
	u32 sign;
	u32 abs_val;
	u32 exp;
	u32 mantissa;
	int lz;

	if (corrected == 0)
		return 0;

	sign    = (corrected < 0) ? 1u : 0u;
	abs_val = (u32)(corrected < 0 ? -corrected : corrected);

	/* Normalize: find highest set bit */
	lz = __builtin_clz(abs_val);
	exp = 127 + (31 - lz); /* exponent bias 127 */

	/* Mantissa: shift abs_val to Q23 */
	if (31 - lz >= 23)
		mantissa = abs_val >> ((31 - lz) - 23);
	else
		mantissa = abs_val << (23 - (31 - lz));
	mantissa &= 0x7FFFFF; /* strip implicit leading 1 */

	/* Scale back: corrected is in nrad/s (×1e9), result should be rad/s */
	/* We need to divide by 1e9 in float representation — adjust exponent */
	/* log2(1e9) ≈ 29.897, so subtract 30 from exponent and compensate */
	/* Simpler: just rebuild from corrected nrad/s by re-encoding */
	(void)exp; (void)mantissa; /* unused — use direct re-encode below */

	/*
	 * Re-encode corrected nrad/s → IEEE 754 rad/s directly.
	 * corrected is in units of 1e-9 rad/s.
	 * We want float = corrected × 1e-9.
	 * log2(1e-9) ≈ -29.897, so exp = 127 + exponent_of_corrected - 30.
	 */
	lz = __builtin_clz(abs_val);
	{
		int exp_val = 127 + (31 - lz) - 30; /* -30 ≈ log2(1e-9) */
		u32 mant;

		if (exp_val <= 0 || exp_val >= 255)
			return 0; /* underflow/overflow → 0 */

		if (31 - lz >= 23)
			mant = abs_val >> ((31 - lz) - 23);
		else
			mant = abs_val << (23 - (31 - lz));
		mant &= 0x7FFFFF;

		return (sign << 31) | ((u32)exp_val << 23) | mant;
	}
}
#endif

/* ---------- Gyro bias calibration workqueue ---------- */

static void gyro_calib_work_fn(struct work_struct *work)
{
	struct wheeltec_dev *wd = container_of(work, struct wheeltec_dev, calib_work);
	ktime_t deadline = ktime_add_ms(ktime_get(), 5000);
	u32 bx, by, bz;

	dev_info(&wd->intf->dev, "gyro calib: collecting bias for 5s (~2000 samples)...\n");

	wd->gyro_sum_x = 0;
	wd->gyro_sum_y = 0;
	wd->gyro_sum_z = 0;
	wd->gyro_calib_count = 0;

	while (ktime_before(ktime_get(), deadline) && !wd->disconnected) {
		if (!atomic_xchg(&wd->calib_sample_ready, 0)) {
			usleep_range(500, 1000); /* wait ~1ms between polls */
			continue;
		}
		bx = (u32)atomic_read(&wd->calib_gyro_x_bits);
		by = (u32)atomic_read(&wd->calib_gyro_y_bits);
		bz = (u32)atomic_read(&wd->calib_gyro_z_bits);
		wd->gyro_sum_x += f32bits_to_nrads(bx);
		wd->gyro_sum_y += f32bits_to_nrads(by);
		wd->gyro_sum_z += f32bits_to_nrads(bz);
		wd->gyro_calib_count++;
	}

	if (wd->disconnected || wd->gyro_calib_count == 0)
		return;

	wd->gyro_bias_x = (s32)(wd->gyro_sum_x / wd->gyro_calib_count);
	wd->gyro_bias_y = (s32)(wd->gyro_sum_y / wd->gyro_calib_count);
	wd->gyro_bias_z = (s32)(wd->gyro_sum_z / wd->gyro_calib_count);

	/* Publish bias before setting done flag */
	smp_wmb();
	wd->gyro_calib_collecting = false;
	wd->gyro_calib_done = true;

	dev_info(&wd->intf->dev,
		"gyro calib done: bias=(%d %d %d) nrad/s, n=%u\n",
		wd->gyro_bias_x, wd->gyro_bias_y, wd->gyro_bias_z,
		wd->gyro_calib_count);
}

static void publish_raw(struct wheeltec_dev *wd, const u8 *payload)
{
	struct wt_raw_frame snap;
	s64 hw_ts;

	snap.ktime_ns = ktime_get_real_ns();
	memcpy(&snap.sensors,      payload + 0,  sizeof(snap.sensors)); /* gyro + accel + mag */
	memcpy(&snap.hw_timestamp, payload + 48, sizeof(snap.hw_timestamp)); /* skip temp/pressure */

	hw_ts = snap.hw_timestamp;
	if (wd->raw_last_hw_ts && hw_ts > wd->raw_last_hw_ts) {
		u64 dt_ns = (u64)(hw_ts - wd->raw_last_hw_ts) * 1000; /* µs → ns */
		if (wd->raw_period_avg == 0)
			wd->raw_period_avg = dt_ns;
		else
			wd->raw_period_avg = (wd->raw_period_avg * 7 + dt_ns) / 8;
		snap.hz = (u32)(1000000000ULL / wd->raw_period_avg);
	} else {
		snap.hz = 0;
	}
	wd->raw_last_hw_ts = hw_ts;
	snap.seq = ++wd->raw_fseq;

	/* Gyro bias calibration disabled — raw data passed through unmodified.
	 * if (wd->gyro_calib_collecting) {
	 *     u32 bx, by, bz;
	 *     memcpy(&bx, &snap.gyro_x, 4); memcpy(&by, &snap.gyro_y, 4); memcpy(&bz, &snap.gyro_z, 4);
	 *     atomic_set(&wd->calib_gyro_x_bits, (int)bx);
	 *     atomic_set(&wd->calib_gyro_y_bits, (int)by);
	 *     atomic_set(&wd->calib_gyro_z_bits, (int)bz);
	 *     atomic_set(&wd->calib_sample_ready, 1);
	 * }
	 * if (wd->gyro_calib_done) {
	 *     u32 bx, by, bz;
	 *     memcpy(&bx, &snap.gyro_x, 4); memcpy(&by, &snap.gyro_y, 4); memcpy(&bz, &snap.gyro_z, 4);
	 *     bx = f32bits_sub_nrads(bx, wd->gyro_bias_x);
	 *     by = f32bits_sub_nrads(by, wd->gyro_bias_y);
	 *     bz = f32bits_sub_nrads(bz, wd->gyro_bias_z);
	 *     memcpy(&snap.gyro_x, &bx, 4); memcpy(&snap.gyro_y, &by, 4); memcpy(&snap.gyro_z, &bz, 4);
	 * }
	 */

	write_seqcount_begin(&wd->raw_seq);
	wd->raw_snap = snap;
	write_seqcount_end(&wd->raw_seq);

	atomic_set(&wd->has_raw, 1);
	atomic64_inc(&wd->raw_frames_ok);
	wake_up_interruptible(&wd->raw_wq);
}

static void publish_filter(struct wheeltec_dev *wd, const u8 *payload)
{
	struct wt_filter_frame snap;
	s64 hw_ts;

	snap.ktime_ns = ktime_get_real_ns();
	memcpy(&snap.data, payload, sizeof(snap.data)); /* speeds + euler + quat + hw_timestamp */

	hw_ts = snap.hw_timestamp;
	if (wd->filter_last_hw_ts && hw_ts > wd->filter_last_hw_ts) {
		u64 dt_ns = (u64)(hw_ts - wd->filter_last_hw_ts) * 1000;
		if (wd->filter_period_avg == 0)
			wd->filter_period_avg = dt_ns;
		else
			wd->filter_period_avg = (wd->filter_period_avg * 7 + dt_ns) / 8;
		snap.hz = (u32)(1000000000ULL / wd->filter_period_avg);
	} else {
		snap.hz = 0;
	}
	wd->filter_last_hw_ts = hw_ts;
	snap.seq = ++wd->filter_fseq;

	write_seqcount_begin(&wd->filter_seq);
	wd->filter_snap = snap;
	write_seqcount_end(&wd->filter_seq);

	atomic_set(&wd->has_filter, 1);
	atomic64_inc(&wd->filter_frames_ok);
	wake_up_interruptible(&wd->filter_wq);
}

/* Drain as many complete frames from fbuf as possible. */
static void parse_buffer(struct wheeltec_dev *wd)
{
	while (wd->fbuf_len > 0) {
		size_t i;
		u8 type, plen, exp_len;
		u16 head_crc16, calc_crc16;
		u8 calc_crc8;
		size_t total_len;

		/* Find WT_FRAME_HEAD */
		for (i = 0; i < wd->fbuf_len; i++)
			if (wd->fbuf[i] == WT_FRAME_HEAD)
				break;

		if (i > 0) {
			memmove(wd->fbuf, wd->fbuf + i, wd->fbuf_len - i);
			wd->fbuf_len -= i;
			atomic64_add(i, &wd->frames_dropped);
		}
		if (wd->fbuf_len < 8)
			return;

		type    = wd->fbuf[1];
		plen    = wd->fbuf[2];
		exp_len = expected_payload_len(type);

		if (exp_len == 0 || plen != exp_len) {
			memmove(wd->fbuf, wd->fbuf + 1, wd->fbuf_len - 1);
			wd->fbuf_len--;
			atomic64_inc(&wd->frames_dropped);
			continue;
		}

		total_len = 8 + plen;
		if (wd->fbuf_len < total_len)
			return;

		if (wd->fbuf[7 + plen] != WT_FRAME_END) {
			memmove(wd->fbuf, wd->fbuf + 1, wd->fbuf_len - 1);
			wd->fbuf_len--;
			atomic64_inc(&wd->frames_dropped);
			continue;
		}

		/* CRC8 over header bytes [0..3] */
		calc_crc8 = wheeltec_crc8(&wd->fbuf[0], 4);
		if (calc_crc8 != wd->fbuf[4]) {
			memmove(wd->fbuf, wd->fbuf + 1, wd->fbuf_len - 1);
			wd->fbuf_len--;
			atomic64_inc(&wd->frames_crc_err);
			continue;
		}

		/* CRC16 over payload */
		head_crc16 = ((u16)wd->fbuf[5] << 8) | wd->fbuf[6];
		calc_crc16 = wheeltec_crc16(&wd->fbuf[7], plen);
		if (calc_crc16 != head_crc16) {
			memmove(wd->fbuf, wd->fbuf + 1, wd->fbuf_len - 1);
			wd->fbuf_len--;
			atomic64_inc(&wd->frames_crc_err);
			continue;
		}

		/* Dispatch by type */
		if (type == TYPE_IMU)
			publish_raw(wd, &wd->fbuf[7]);
		else if (type == TYPE_AHRS)
			publish_filter(wd, &wd->fbuf[7]);
		/* TYPE_INSGPS: ignored */

		memmove(wd->fbuf, wd->fbuf + total_len, wd->fbuf_len - total_len);
		wd->fbuf_len -= total_len;
	}
}

/* ---------- URB completion ---------- */

#define URB_MAX_CONSECUTIVE_ERR 10

/*
 * Lỗi transient (EPROTO, EILSEQ, EOVERFLOW, ENOSR) xảy ra do nhiễu USB hoặc
 * firmware glitch. Retry từ process context để tránh busy-loop trong softirq.
 */
static void urb_retry_work_fn(struct work_struct *work)
{
	struct wheeltec_dev *wd =
		container_of(work, struct wheeltec_dev, urb_retry_work);
	int rc;

	if (wd->disconnected)
		return;

	rc = usb_submit_urb(wd->bulk_in_urb, GFP_KERNEL);
	if (rc && rc != -EPERM && rc != -ENODEV)
		dev_err(&wd->intf->dev, "URB retry failed: %d\n", rc);
	else if (rc == 0)
		atomic_set(&wd->urb_err_cnt, 0);
}

static void wheeltec_read_complete(struct urb *urb)
{
	struct wheeltec_dev *wd = urb->context;
	int rc;

	if (urb->status) {
		/* Lỗi khi kill/disconnect — dừng hẳn, không retry */
		if (urb->status == -ENOENT   ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN)
			return;

		/* Lỗi transient — thử lại từ workqueue */
		if (atomic_inc_return(&wd->urb_err_cnt) <= URB_MAX_CONSECUTIVE_ERR) {
			dev_warn_ratelimited(&wd->intf->dev,
				"bulk-in URB status=%d, scheduling retry (%d/%d)\n",
				urb->status,
				atomic_read(&wd->urb_err_cnt),
				URB_MAX_CONSECUTIVE_ERR);
			schedule_work(&wd->urb_retry_work);
		} else {
			dev_err(&wd->intf->dev,
				"URB failed %d times consecutively, giving up\n",
				URB_MAX_CONSECUTIVE_ERR);
		}
		return;
	}

	atomic_set(&wd->urb_err_cnt, 0);

	if (urb->actual_length > 0) {
		size_t copy_len  = urb->actual_length;
		size_t free_space = FRAME_BUF_SIZE - wd->fbuf_len;

		/*
		 * Log IMU ASCII responses (*#OK, *#ERROR, etc).
		 * Binary frames có byte 0xFC — nếu buffer bắt đầu bằng 0xFC
		 * thì đây là binary stream, không log để tránh garbage.
		 * Chỉ log khi có ký tự '*' hoặc '#' trong buffer (IMU response).
		 */
		if (atomic_read(&wd->fconfig_pending) || atomic_read(&wd->tare_cmd_pending)) {
			u8 *buf = urb->transfer_buffer;
			size_t len = urb->actual_length;
			size_t i;
			bool has_imu_resp = false;

			/* Kiểm tra buffer có chứa IMU ASCII response không */
			for (i = 0; i + 1 < len; i++) {
				if (buf[i] == '*' && buf[i+1] == '#') {
					has_imu_resp = true;
					break;
				}
			}

			if (has_imu_resp) {
				char tmp[65];
				size_t out = 0;
				for (i = 0; i < len; i++) {
					u8 b = buf[i];
					if (b == '\r')
						continue;
					if (b == '\n') {
						if (out > 0) {
							tmp[out] = '\0';
							dev_info(&wd->intf->dev, "IMU << %s\n", tmp);
							out = 0;
						}
						continue;
					}
					if (b >= 32 && b < 127) {
						tmp[out++] = b;
						if (out == 64) {
							tmp[out] = '\0';
							dev_info(&wd->intf->dev, "IMU << %s\n", tmp);
							out = 0;
						}
					}
				}
				if (out > 0) {
					tmp[out] = '\0';
					dev_info(&wd->intf->dev, "IMU << %s\n", tmp);
				}
			}

			/* tare: vẫn parse để detect khi stream binary quay lại */
			if (!atomic_read(&wd->tare_cmd_pending))
				goto resubmit;
		}

		if (copy_len > free_space) {
			wd->fbuf_len  = 0;
			free_space    = FRAME_BUF_SIZE;
			copy_len      = min(copy_len, free_space);
		}
		memcpy(wd->fbuf + wd->fbuf_len, urb->transfer_buffer, copy_len);
		wd->fbuf_len += copy_len;
		parse_buffer(wd);
	}

resubmit:

	rc = usb_submit_urb(urb, GFP_ATOMIC);
	if (rc && rc != -EPERM)
		dev_err(&wd->intf->dev, "resubmit bulk-in failed: %d\n", rc);
}

/* ---------- Misc device: RAW ---------- */

static int raw_open(struct inode *ino, struct file *f) { return 0; }
static int raw_release(struct inode *ino, struct file *f) { return 0; }

static ssize_t raw_read(struct file *f, char __user *ubuf, size_t len, loff_t *pos)
{
	struct wheeltec_dev *wd = to_wdev_raw(f);
	struct wt_raw_frame snap;
	unsigned int seq;

	if (len < sizeof(snap))
		return -EINVAL;

	if (!atomic_read(&wd->has_raw)) {
		if (f->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(wd->raw_wq,
					     atomic_read(&wd->has_raw) || wd->disconnected))
			return -ERESTARTSYS;
		if (wd->disconnected)
			return -ENODEV;
	}

	do {
		seq = read_seqcount_begin(&wd->raw_seq);
		memcpy(&snap, &wd->raw_snap, sizeof(snap));
	} while (read_seqcount_retry(&wd->raw_seq, seq));

	if (copy_to_user(ubuf, &snap, sizeof(snap)))
		return -EFAULT;

	return sizeof(snap);
}

static __poll_t raw_poll(struct file *f, poll_table *pt)
{
	struct wheeltec_dev *wd = to_wdev_raw(f);
	__poll_t mask = 0;

	poll_wait(f, &wd->raw_wq, pt);
	if (atomic_read(&wd->has_raw))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (wd->disconnected)
		mask |= EPOLLHUP;
	return mask;
}

static const struct file_operations raw_fops = {
	.owner   = THIS_MODULE,
	.open    = raw_open,
	.release = raw_release,
	.read    = raw_read,
	.poll    = raw_poll,
};

/* ---------- Misc device: FILTER ---------- */

static int filter_open(struct inode *ino, struct file *f) { return 0; }
static int filter_release(struct inode *ino, struct file *f) { return 0; }

static ssize_t filter_read(struct file *f, char __user *ubuf, size_t len, loff_t *pos)
{
	struct wheeltec_dev *wd = to_wdev_filter(f);
	struct wt_filter_frame snap;
	unsigned int seq;

	if (len < sizeof(snap))
		return -EINVAL;

	if (!atomic_read(&wd->has_filter)) {
		if (f->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(wd->filter_wq,
					     atomic_read(&wd->has_filter) || wd->disconnected))
			return -ERESTARTSYS;
		if (wd->disconnected)
			return -ENODEV;
	}

	do {
		seq = read_seqcount_begin(&wd->filter_seq);
		memcpy(&snap, &wd->filter_snap, sizeof(snap));
	} while (read_seqcount_retry(&wd->filter_seq, seq));

	if (copy_to_user(ubuf, &snap, sizeof(snap)))
		return -EFAULT;

	return sizeof(snap);
}

static __poll_t filter_poll(struct file *f, poll_table *pt)
{
	struct wheeltec_dev *wd = to_wdev_filter(f);
	__poll_t mask = 0;

	poll_wait(f, &wd->filter_wq, pt);
	if (atomic_read(&wd->has_filter))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (wd->disconnected)
		mask |= EPOLLHUP;
	return mask;
}

static const struct file_operations filter_fops = {
	.owner   = THIS_MODULE,
	.open    = filter_open,
	.release = filter_release,
	.read    = filter_read,
	.poll    = filter_poll,
};

/* ---------- Misc device: STATE ---------- */

static ssize_t state_read(struct file *f, char __user *ubuf, size_t len, loff_t *pos)
{
	struct wheeltec_dev *wd = to_wdev_state(f);
	char buf[256];
	int n;

	if (*pos > 0)
		return 0;

	static const char * const reboot_status_str[] = {
		"idle", "in-progress", "success", "warning"
	};
	int rs = atomic_read(&wd->reboot_status);

	int ts = atomic_read(&wd->tare_status);

	n = scnprintf(buf, sizeof(buf),
		"connected=%d\nraw_frames_ok=%lld\nfilter_frames_ok=%lld\nframes_crc_err=%lld\nframes_dropped=%lld\nreboot_status=%s\ntare_status=%s\ngyro_calib=%s\ngyro_bias_x_urad=%d\ngyro_bias_y_urad=%d\ngyro_bias_z_urad=%d\n",
		!wd->disconnected,
		(long long)atomic64_read(&wd->raw_frames_ok),
		(long long)atomic64_read(&wd->filter_frames_ok),
		(long long)atomic64_read(&wd->frames_crc_err),
		(long long)atomic64_read(&wd->frames_dropped),
		reboot_status_str[rs < 4 ? rs : 0],
		reboot_status_str[ts < 4 ? ts : 0],
		wd->gyro_calib_done ? "done" : (wd->gyro_calib_collecting ? "collecting" : "pending"),
		wd->gyro_bias_x,
		wd->gyro_bias_y,
		wd->gyro_bias_z);

	if (len < (size_t)n)
		n = len;
	if (copy_to_user(ubuf, buf, n))
		return -EFAULT;
	*pos += n;
	return n;
}

static const struct file_operations state_fops = {
	.owner  = THIS_MODULE,
	.read   = state_read,
};

/* ---------- Misc device: CTRL ---------- */

static ssize_t ctrl_write(struct file *f, const char __user *ubuf,
			  size_t len, loff_t *pos)
{
	struct wheeltec_dev *wd = to_wdev_ctrl(f);
	char cmd[32];
	size_t copy_len;

	if (wd->disconnected)
		return -ENODEV;

	copy_len = min(len, sizeof(cmd) - 1);
	if (copy_from_user(cmd, ubuf, copy_len))
		return -EFAULT;
	cmd[copy_len] = '\0';

	/* Strip trailing newline from shell echo */
	copy_len = strcspn(cmd, "\r\n");
	cmd[copy_len] = '\0';

	if (!strcmp(cmd, "freboot")) {
		if (atomic_cmpxchg(&wd->reboot_pending, 0, 1) != 0)
			return -EBUSY;
		schedule_work(&wd->reboot_work);
		return len;
	}

	if (!strcmp(cmd, "recalib")) {
		wd->gyro_calib_done       = false;
		wd->gyro_calib_collecting = false;
		wd->gyro_bias_x = 0;
		wd->gyro_bias_y = 0;
		wd->gyro_bias_z = 0;
		dev_info(&wd->intf->dev, "gyro calib: reset, will recollect on next frame\n");
		return len;
	}

	if (!strncmp(cmd, "tare:", 5)) {
		char type_str[16]; int wait_ms = 2500; int tare_type;
		if (sscanf(cmd + 5, "%15[^:]:%d", type_str, &wait_ms) < 1) return -EINVAL;
		if (!strcmp(type_str, "gyro"))       tare_type = 0;
		else if (!strcmp(type_str, "accel")) tare_type = 1;
		else if (!strcmp(type_str, "level")) tare_type = 2;
		else return -EINVAL;
		if (wait_ms < 100 || wait_ms > 60000) return -EINVAL;
		if (atomic_cmpxchg(&wd->tare_pending, 0, 1) != 0) return -EBUSY;
		wd->tare_type = tare_type; wd->tare_wait_ms = wait_ms;
		schedule_work(&wd->tare_work); return len;
	}

	return -EINVAL;
}

static const struct file_operations ctrl_fops = {
	.owner  = THIS_MODULE,
	.write  = ctrl_write,
};

/* ---------- USB probe / disconnect ---------- */

static int wheeltec_register_misc(struct wheeltec_dev *wd)
{
	int rc;

	snprintf(wd->name_raw,    sizeof(wd->name_raw),    "wheeltec_imu%d_raw",    wd->id);
	snprintf(wd->name_filter, sizeof(wd->name_filter), "wheeltec_imu%d_filter", wd->id);
	snprintf(wd->name_state,  sizeof(wd->name_state),  "wheeltec_imu%d_state",  wd->id);
	snprintf(wd->name_ctrl,   sizeof(wd->name_ctrl),   "wheeltec_imu%d_ctrl",   wd->id);

	wd->mdev_raw.minor  = MISC_DYNAMIC_MINOR;
	wd->mdev_raw.name   = wd->name_raw;
	wd->mdev_raw.fops   = &raw_fops;
	wd->mdev_raw.parent = &wd->intf->dev;

	wd->mdev_filter.minor  = MISC_DYNAMIC_MINOR;
	wd->mdev_filter.name   = wd->name_filter;
	wd->mdev_filter.fops   = &filter_fops;
	wd->mdev_filter.parent = &wd->intf->dev;

	wd->mdev_state.minor  = MISC_DYNAMIC_MINOR;
	wd->mdev_state.name   = wd->name_state;
	wd->mdev_state.fops   = &state_fops;
	wd->mdev_state.parent = &wd->intf->dev;

	wd->mdev_ctrl.minor  = MISC_DYNAMIC_MINOR;
	wd->mdev_ctrl.name   = wd->name_ctrl;
	wd->mdev_ctrl.fops   = &ctrl_fops;
	wd->mdev_ctrl.parent = &wd->intf->dev;

	rc = misc_register(&wd->mdev_raw);
	if (rc)
		return rc;
	rc = misc_register(&wd->mdev_filter);
	if (rc)
		goto err_filter;
	rc = misc_register(&wd->mdev_state);
	if (rc)
		goto err_state;
	rc = misc_register(&wd->mdev_ctrl);
	if (rc)
		goto err_ctrl;
	return 0;

err_ctrl:
	misc_deregister(&wd->mdev_state);
err_state:
	misc_deregister(&wd->mdev_filter);
err_filter:
	misc_deregister(&wd->mdev_raw);
	return rc;
}

static void wheeltec_unregister_misc(struct wheeltec_dev *wd)
{
	misc_deregister(&wd->mdev_ctrl);
	misc_deregister(&wd->mdev_state);
	misc_deregister(&wd->mdev_filter);
	misc_deregister(&wd->mdev_raw);
}

static int wheeltec_probe(struct usb_interface *intf,
			  const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_endpoint_descriptor *bulk_in, *bulk_out;
	struct wheeltec_dev *wd;
	int rc;

	rc = usb_find_common_endpoints(intf->cur_altsetting,
				       &bulk_in, &bulk_out, NULL, NULL);
	if (rc) {
		dev_err(&intf->dev, "no bulk-in/out endpoints\n");
		return rc;
	}

	wd = kzalloc(sizeof(*wd), GFP_KERNEL);
	if (!wd)
		return -ENOMEM;

	kref_init(&wd->kref);
	mutex_init(&wd->io_mutex);
	seqcount_init(&wd->raw_seq);
	seqcount_init(&wd->filter_seq);
	init_waitqueue_head(&wd->raw_wq);
	init_waitqueue_head(&wd->filter_wq);
	atomic64_set(&wd->raw_frames_ok,    0);
	atomic64_set(&wd->filter_frames_ok, 0);
	atomic64_set(&wd->frames_crc_err,   0);
	atomic64_set(&wd->frames_dropped,   0);
	atomic_set(&wd->has_raw,    0);
	atomic_set(&wd->has_filter, 0);
	atomic_set(&wd->urb_err_cnt, 0);
	atomic_set(&wd->reboot_pending,  0);
	atomic_set(&wd->reboot_status,   0);
	atomic_set(&wd->fconfig_pending, 0);
	atomic_set(&wd->tare_pending,    0);
	atomic_set(&wd->tare_cmd_pending, 0);
	atomic_set(&wd->tare_status,    0);
	wd->tare_type    = 0;
	wd->tare_wait_ms = 2500;
	INIT_WORK(&wd->urb_retry_work, urb_retry_work_fn);
	INIT_WORK(&wd->reboot_work, wt_reboot_work_fn);
	INIT_WORK(&wd->tare_work, wt_tare_work_fn);
	INIT_WORK(&wd->calib_work, gyro_calib_work_fn);

	wd->udev = usb_get_dev(udev);
	wd->intf = intf;
	wd->bulk_in_addr  = bulk_in->bEndpointAddress;
	wd->bulk_out_addr = bulk_out->bEndpointAddress;
	wd->bulk_in_size  = usb_endpoint_maxp(bulk_in);
	if (wd->bulk_in_size < 64)
		wd->bulk_in_size = 64;

	wd->id = ida_alloc(&wheeltec_ida, GFP_KERNEL);
	if (wd->id < 0) { rc = wd->id; goto err_free; }

	wd->bulk_in_buf = kmalloc(wd->bulk_in_size, GFP_KERNEL);
	if (!wd->bulk_in_buf) { rc = -ENOMEM; goto err_ida; }

	wd->bulk_in_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!wd->bulk_in_urb) { rc = -ENOMEM; goto err_buf; }

	usb_fill_bulk_urb(wd->bulk_in_urb, udev,
			  usb_rcvbulkpipe(udev, wd->bulk_in_addr),
			  wd->bulk_in_buf, wd->bulk_in_size,
			  wheeltec_read_complete, wd);

	rc = cp210x_setup(udev, default_baud);
	if (rc) {
		dev_err(&intf->dev, "cp210x_setup failed: %d\n", rc);
		goto err_urb;
	}

	rc = wheeltec_register_misc(wd);
	if (rc)
		goto err_uart_off;

	usb_set_intfdata(intf, wd);

	if (!wd->startup_tare_done) {
		dev_info(&intf->dev, "wheeltec_imu%d: running startup gyro tare...\n", wd->id);
		wd->tare_type = 0;
		wd->tare_wait_ms = 2500;
		atomic_set(&wd->tare_pending, 1);
		wd->startup_tare_done = true;
		schedule_work(&wd->tare_work);
	} else {
		rc = usb_submit_urb(wd->bulk_in_urb, GFP_KERNEL);
		if (rc) {
			dev_err(&intf->dev, "submit bulk-in urb failed: %d\n", rc);
			goto err_misc;
		}
	}

	dev_info(&intf->dev,
		 "wheeltec_imu%d bound (serial=%s, baud=%u, max=%zu)\n",
		 wd->id, udev->serial ? udev->serial : "?",
		 default_baud, wd->bulk_in_size);
	return 0;

err_misc:
	wheeltec_unregister_misc(wd);
err_uart_off:
	cp210x_ctrl_out(udev, CP210X_IFC_ENABLE, UART_DISABLE);
err_urb:
	usb_free_urb(wd->bulk_in_urb);
err_buf:
	kfree(wd->bulk_in_buf);
err_ida:
	ida_free(&wheeltec_ida, wd->id);
err_free:
	usb_put_dev(wd->udev);
	kfree(wd);
	return rc;
}

static void wheeltec_disconnect(struct usb_interface *intf)
{
	struct wheeltec_dev *wd = usb_get_intfdata(intf);

	if (!wd)
		return;

	wd->disconnected = true;
	wake_up_interruptible_all(&wd->raw_wq);
	wake_up_interruptible_all(&wd->filter_wq);

	usb_kill_urb(wd->bulk_in_urb);
	cancel_work_sync(&wd->urb_retry_work);
	cancel_work_sync(&wd->reboot_work);
	cancel_work_sync(&wd->tare_work);
	cancel_work_sync(&wd->calib_work);

	wheeltec_unregister_misc(wd);

	cp210x_ctrl_out(wd->udev, CP210X_IFC_ENABLE, UART_DISABLE);

	usb_free_urb(wd->bulk_in_urb);
	kfree(wd->bulk_in_buf);
	ida_free(&wheeltec_ida, wd->id);
	usb_put_dev(wd->udev);
	usb_set_intfdata(intf, NULL);
	kfree(wd);
}

static int wheeltec_pre_reset(struct usb_interface *intf)
{
	struct wheeltec_dev *wd = usb_get_intfdata(intf);

	if (!wd)
		return 0;

	usb_kill_urb(wd->bulk_in_urb);
	cancel_work_sync(&wd->urb_retry_work);
	cancel_work_sync(&wd->reboot_work);
	cancel_work_sync(&wd->tare_work);
	cancel_work_sync(&wd->calib_work);
	return 0;
}

static int wheeltec_post_reset(struct usb_interface *intf)
{
	struct wheeltec_dev *wd = usb_get_intfdata(intf);
	int rc;

	if (!wd || wd->disconnected)
		return 0;

	/* Tái cấu hình UART sau reset */
	rc = cp210x_setup(wd->udev, default_baud);
	if (rc) {
		dev_err(&intf->dev, "cp210x_setup after reset failed: %d\n", rc);
		return rc;
	}

	/* Xoá buffer parser để tránh dữ liệu cũ còn sót */
	wd->fbuf_len = 0;
	atomic_set(&wd->urb_err_cnt, 0);

	rc = usb_submit_urb(wd->bulk_in_urb, GFP_KERNEL);
	if (rc)
		dev_err(&intf->dev, "URB resubmit after reset failed: %d\n", rc);

	return rc;
}

static struct usb_driver wheeltec_driver = {
	.name		= "wheeltec_imu",
	.id_table	= wheeltec_id_table,
	.probe		= wheeltec_probe,
	.disconnect	= wheeltec_disconnect,
	.pre_reset	= wheeltec_pre_reset,
	.post_reset	= wheeltec_post_reset,
};

module_usb_driver(wheeltec_driver);

MODULE_AUTHOR("Robotnet");
MODULE_DESCRIPTION("WheelTec N100 IMU USB driver (CP2102 bypass)");
MODULE_LICENSE("GPL v2");
