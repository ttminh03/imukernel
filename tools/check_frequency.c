/*
 * check_frequency.c — đo tần số frame từ /dev/imu/0/wheeltec_imu0_raw và _filter
 *
 * Build: gcc -O2 -o check_frequency check_frequency.c
 * Run:   sudo ./check_frequency [device_index]   (mặc định index=0)
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <time.h>
#include <signal.h>
#include <errno.h>

/* ---- ABI structs phải khớp với kernel __packed ---- */

struct wt_raw_frame {
	uint64_t ktime_ns;
	uint32_t seq;
	uint32_t hz;
	float    gyro_x, gyro_y, gyro_z;
	float    accel_x, accel_y, accel_z;
	float    mag_x, mag_y, mag_z;
	int64_t  hw_timestamp;
} __attribute__((packed));

struct wt_filter_frame {
	uint64_t ktime_ns;
	uint32_t seq;
	uint32_t hz;
	float    roll_speed, pitch_speed, heading_speed;
	float    roll, pitch, heading;
	float    qw, qx, qy, qz;
	int64_t  hw_timestamp;
} __attribute__((packed));

/* ---- Per-channel stats ---- */

struct chan_stat {
	const char *name;
	int         fd;

	uint64_t    win_start;
	uint32_t    win_new;
	uint32_t    meas_hz;
	uint32_t    kern_hz;

	uint32_t    last_seq;
	int         seq_valid;

	uint64_t    total_new;
	uint64_t    total_dup;
	uint64_t    total_missed;

	uint32_t    win_dup;
	uint32_t    win_missed;
};

static volatile int running = 1;

static void sig_handler(int s) { (void)s; running = 0; }

static uint64_t now_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

static void chan_disconnect(struct chan_stat *c)
{
	if (c->fd >= 0) {
		close(c->fd);
		c->fd = -1;
	}
	/* Reset trạng thái seq để không tính missed khi reconnect */
	c->seq_valid = 0;
	c->meas_hz   = 0;
	c->kern_hz   = 0;
	c->win_new   = 0;
	c->win_dup   = 0;
	c->win_missed = 0;
}

/* Trả về 1 nếu mở được, 0 nếu chưa */
static int chan_try_open(struct chan_stat *c)
{
	if (c->fd >= 0)
		return 1;

	c->fd = open(c->name, O_RDONLY | O_NONBLOCK);
	if (c->fd < 0)
		return 0;

	c->win_start = now_ns();
	printf("  [reconnect] %s\n", c->name);
	return 1;
}

static void process_seq(struct chan_stat *c, uint32_t seq, uint32_t kern_hz)
{
	c->kern_hz = kern_hz;

	if (!c->seq_valid) {
		c->last_seq  = seq;
		c->seq_valid = 1;
		c->win_start = now_ns();
		c->win_new   = 1;
		c->total_new = 1;
		return;
	}

	if (seq == c->last_seq) {
		c->win_dup++;
		c->total_dup++;
		return;
	}

	uint32_t gap    = seq - c->last_seq;
	uint32_t missed = gap - 1;

	c->win_new++;
	c->total_new++;
	c->win_missed   += missed;
	c->total_missed += missed;
	c->last_seq      = seq;
}

static void print_chan(struct chan_stat *c, uint64_t t)
{
	if (c->fd < 0) {
		printf("  %-36s  [ngắt kết nối — đang chờ reconnect...]\n", c->name);
		return;
	}

	if (c->seq_valid) {
		uint64_t dt = t - c->win_start;
		if (dt > 0)
			c->meas_hz = (uint32_t)(c->win_new * 1000000000ULL / dt);
		c->win_start = t;
		c->win_new   = 0;
	}

	printf("  %-36s  Hz(đo)=%3u  Hz(kern)=%3u  seq=%-8u\n"
	       "    new=%-10llu  dup=%-8llu  missed=%-8llu"
	       "  (cửa sổ: dup=%-4u missed=%-4u)\n",
	       c->name,
	       c->meas_hz, c->kern_hz, c->last_seq,
	       (unsigned long long)c->total_new,
	       (unsigned long long)c->total_dup,
	       (unsigned long long)c->total_missed,
	       c->win_dup, c->win_missed);

	c->win_dup    = 0;
	c->win_missed = 0;
}

/* ---- Main ---- */

int main(int argc, char *argv[])
{
	int idx = (argc > 1) ? atoi(argv[1]) : 0;
	char path_raw[64], path_filter[64];
	struct chan_stat raw = {0}, filt = {0};
	struct pollfd pfds[2];
	uint64_t next_print;
	uint64_t next_retry = 0;

	snprintf(path_raw,    sizeof(path_raw),    "/dev/imu/0/wheeltec_imu%d_raw",    idx);
	snprintf(path_filter, sizeof(path_filter), "/dev/imu/0/wheeltec_imu%d_filter", idx);
	raw.name   = path_raw;
	filt.name  = path_filter;
	raw.fd     = -1;
	filt.fd    = -1;

	signal(SIGINT,  sig_handler);
	signal(SIGTERM, sig_handler);

	printf("Đo tần số frame — nhấn Ctrl+C để dừng\n\n");
	next_print = now_ns() + 1000000000ULL;

	while (running) {
		uint64_t t = now_ns();

		/* Thử mở lại device mỗi 1 giây nếu đang ngắt */
		if (t >= next_retry) {
			next_retry = t + 1000000000ULL;
			chan_try_open(&raw);
			chan_try_open(&filt);
		}

		/* Cập nhật pfds — fd=-1 sẽ bị poll bỏ qua (events=0) */
		pfds[0].fd      = raw.fd;
		pfds[0].events  = (raw.fd  >= 0) ? POLLIN : 0;
		pfds[0].revents = 0;
		pfds[1].fd      = filt.fd;
		pfds[1].events  = (filt.fd >= 0) ? POLLIN : 0;
		pfds[1].revents = 0;

		int ret = poll(pfds, 2, 200);
		t = now_ns();

		if (ret < 0) {
			if (errno == EINTR)
				continue;
			break;
		}

		/* Đọc raw */
		if (raw.fd >= 0) {
			if (pfds[0].revents & (POLLHUP | POLLERR)) {
				printf("  [disconnect] %s\n", raw.name);
				chan_disconnect(&raw);
			} else if (pfds[0].revents & POLLIN) {
				struct wt_raw_frame f;
				ssize_t n = read(raw.fd, &f, sizeof(f));
				if (n == (ssize_t)sizeof(f)) {
					process_seq(&raw, f.seq, f.hz);
				} else if (n < 0 && (errno == ENODEV || errno == ENOENT || errno == EIO)) {
					printf("  [disconnect] %s (errno=%d)\n", raw.name, errno);
					chan_disconnect(&raw);
				}
			}
		}

		/* Đọc filter */
		if (filt.fd >= 0) {
			if (pfds[1].revents & (POLLHUP | POLLERR)) {
				printf("  [disconnect] %s\n", filt.name);
				chan_disconnect(&filt);
			} else if (pfds[1].revents & POLLIN) {
				struct wt_filter_frame f;
				ssize_t n = read(filt.fd, &f, sizeof(f));
				if (n == (ssize_t)sizeof(f)) {
					process_seq(&filt, f.seq, f.hz);
				} else if (n < 0 && (errno == ENODEV || errno == ENOENT || errno == EIO)) {
					printf("  [disconnect] %s (errno=%d)\n", filt.name, errno);
					chan_disconnect(&filt);
				}
			}
		}

		if (t >= next_print) {
			next_print = t + 1000000000ULL;
			printf("─────────────────────────────────────────────────────────────\n");
			print_chan(&raw,  t);
			print_chan(&filt, t);
		}
	}

	printf("\n=== Tổng kết ===\n");
	printf("%-36s  new=%-10llu  dup=%-8llu  missed=%llu\n",
	       raw.name,
	       (unsigned long long)raw.total_new,
	       (unsigned long long)raw.total_dup,
	       (unsigned long long)raw.total_missed);
	printf("%-36s  new=%-10llu  dup=%-8llu  missed=%llu\n",
	       filt.name,
	       (unsigned long long)filt.total_new,
	       (unsigned long long)filt.total_dup,
	       (unsigned long long)filt.total_missed);

	if (raw.fd  >= 0) close(raw.fd);
	if (filt.fd >= 0) close(filt.fd);
	return 0;
}
