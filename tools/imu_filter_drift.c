/*
 * imu_filter_drift.c — đo gyro drift từ /dev/wheeltec_imuN_filter
 *
 * Đọc roll_speed / pitch_speed / heading_speed (rad/s) từ FilterFrame,
 * tích phân theo thời gian thực → in bias, noise, góc trôi tích luỹ.
 *
 * Build:  gcc -O2 -o imu_filter_drift imu_filter_drift.c -lm
 * Dùng:   sudo ./imu_filter_drift [-d /dev/wheeltec_imuN_filter] [-t <giây>]
 */

#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <math.h>

/* Phải khớp chính xác với wt_filter_frame trong kernel */
struct filter_frame {
    uint64_t ktime_ns;       /* 8 */
    uint32_t seq;            /* 4 */
    uint32_t hz;             /* 4 */
    float    roll_speed;     /* rad/s — 4 */
    float    pitch_speed;    /* rad/s — 4 */
    float    heading_speed;  /* rad/s — 4 */
    float    roll;           /* rad  — 4 */
    float    pitch;          /* rad  — 4 */
    float    heading;        /* rad  — 4 */
    float    qw, qx, qy, qz;/* —       16 */
    int64_t  hw_timestamp;   /* µs  — 8 */
} __attribute__((packed));   /* tổng: 64 bytes */

static volatile int running = 1;
static void on_sigint(int sig) { (void)sig; running = 0; }

static double now_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

int main(int argc, char *argv[])
{
    const char *path = "/dev/wheeltec_imu0_filter";
    int duration_sec  = 60;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-d") && i + 1 < argc) {
            path = argv[++i];
        } else if (!strcmp(argv[i], "-t") && i + 1 < argc) {
            duration_sec = atoi(argv[++i]);
        } else if (!strncmp(argv[i], "-t", 2) && argv[i][2]) {
            duration_sec = atoi(argv[i] + 2);
        }
    }
    if (duration_sec <= 0) duration_sec = 1;

    /* Kiểm tra kích thước struct */
    if (sizeof(struct filter_frame) != 64) {
        fprintf(stderr, "Lỗi: sizeof(filter_frame) = %zu, cần 64!\n",
                sizeof(struct filter_frame));
        return 1;
    }

    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "Không mở được %s: %s\n", path, strerror(errno));
        return 1;
    }

    signal(SIGINT, on_sigint);

    printf("=== IMU FILTER DRIFT METER ===\n");
    printf("\033[1;33mGIỮ IMU ĐỨNG YÊN HOÀN TOÀN TRONG SUỐT QUÁ TRÌNH ĐO!\033[0m\n");
    printf("Device   : %s\n", path);
    printf("Thời gian: %d giây  (Ctrl+C để dừng sớm)\n\n", duration_sec);

    struct filter_frame f;
    uint32_t prev_seq   = UINT32_MAX;
    uint64_t count      = 0;
    uint32_t drops      = 0;

    /* Accumulators cho bias (mean) */
    double sum_rs = 0, sum_ps = 0, sum_hs = 0;   /* roll/pitch/heading speed */

    /* Accumulators cho variance */
    double sum_sq_rs = 0, sum_sq_ps = 0, sum_sq_hs = 0;

    /* Min/max */
    float max_rs = -1e9f, max_ps = -1e9f, max_hs = -1e9f;
    float min_rs =  1e9f, min_ps =  1e9f, min_hs =  1e9f;

    /* Tích phân thực sự theo ktime (không dùng count*dt) */
    double integ_roll    = 0;  /* rad */
    double integ_pitch   = 0;
    double integ_heading = 0;
    uint64_t prev_ktime  = 0;   /* ns */

    double t_start = now_sec();
    double t_end   = t_start + duration_sec;
    double t_last_print = t_start;

    while (running && now_sec() < t_end) {
        ssize_t n = read(fd, &f, sizeof(f));
        if (n < 0) {
            if (errno == EINTR) break;
            fprintf(stderr, "read error: %s\n", strerror(errno));
            break;
        }
        if (n != (ssize_t)sizeof(f)) {
            fprintf(stderr, "short read: %zd bytes (cần %zu)\n", n, sizeof(f));
            break;
        }

        /* Bỏ qua frame trùng */
        if (prev_seq != UINT32_MAX) {
            if (f.seq == prev_seq)
                continue;
            if (f.seq > prev_seq + 1)
                drops += f.seq - prev_seq - 1;
        }
        prev_seq = f.seq;

        /* Tích phân góc dùng dt từ ktime_ns */
        if (prev_ktime != 0 && f.ktime_ns > prev_ktime) {
            double dt = (double)(f.ktime_ns - prev_ktime) * 1e-9; /* giây */
            integ_roll    += f.roll_speed    * dt;
            integ_pitch   += f.pitch_speed   * dt;
            integ_heading += f.heading_speed * dt;
        }
        prev_ktime = f.ktime_ns;

        /* Tích luỹ thống kê */
        sum_rs    += f.roll_speed;
        sum_ps    += f.pitch_speed;
        sum_hs    += f.heading_speed;
        sum_sq_rs += (double)f.roll_speed    * f.roll_speed;
        sum_sq_ps += (double)f.pitch_speed   * f.pitch_speed;
        sum_sq_hs += (double)f.heading_speed * f.heading_speed;

        if (f.roll_speed    > max_rs) max_rs = f.roll_speed;
        if (f.pitch_speed   > max_ps) max_ps = f.pitch_speed;
        if (f.heading_speed > max_hs) max_hs = f.heading_speed;
        if (f.roll_speed    < min_rs) min_rs = f.roll_speed;
        if (f.pitch_speed   < min_ps) min_ps = f.pitch_speed;
        if (f.heading_speed < min_hs) min_hs = f.heading_speed;

        count++;

        /* In tiến độ mỗi 2 giây */
        double now = now_sec();
        if (now - t_last_print >= 2.0) {
            t_last_print = now;
            double elapsed = now - t_start;
            double roll_deg  = integ_roll    * (180.0 / M_PI);
            double pitch_deg = integ_pitch   * (180.0 / M_PI);
            double head_deg  = integ_heading * (180.0 / M_PI);
            printf("\r[%5.1f/%ds] %lu mẫu | hz=%u | "
                   "trôi: roll=%+.4f° pitch=%+.4f° heading=%+.4f°   ",
                   elapsed, duration_sec, (unsigned long)count, f.hz,
                   roll_deg, pitch_deg, head_deg);
            fflush(stdout);
        }
    }

    printf("\n\n");
    close(fd);

    if (count == 0) {
        printf("Lỗi: Không thu thập được dữ liệu nào!\n");
        return 1;
    }

    double actual_time = now_sec() - t_start;
    double sample_rate = count / actual_time;

    double mean_rs = sum_rs / count;
    double mean_ps = sum_ps / count;
    double mean_hs = sum_hs / count;

    double std_rs = sqrt(fmax(0, sum_sq_rs / count - mean_rs * mean_rs));
    double std_ps = sqrt(fmax(0, sum_sq_ps / count - mean_ps * mean_ps));
    double std_hs = sqrt(fmax(0, sum_sq_hs / count - mean_hs * mean_hs));

    /* Góc trôi tích phân (tích phân chính xác theo ktime) */
    double drift_roll_deg    = integ_roll    * (180.0 / M_PI);
    double drift_pitch_deg   = integ_pitch   * (180.0 / M_PI);
    double drift_heading_deg = integ_heading * (180.0 / M_PI);

    /* Tốc độ trôi deg/h từ bias */
    double dph_roll    = mean_rs * 3600.0 * (180.0 / M_PI);
    double dph_pitch   = mean_ps * 3600.0 * (180.0 / M_PI);
    double dph_heading = mean_hs * 3600.0 * (180.0 / M_PI);

    /* Allan variance proxy: noise density (deg/sqrt(h)) = std * sqrt(1/sample_rate) * (180/pi) * sqrt(3600) */
    double ard_roll    = std_rs * (180.0 / M_PI) * sqrt(3600.0 / sample_rate);
    double ard_pitch   = std_ps * (180.0 / M_PI) * sqrt(3600.0 / sample_rate);
    double ard_heading = std_hs * (180.0 / M_PI) * sqrt(3600.0 / sample_rate);

    printf("==================== KẾT QUẢ GYRO DRIFT (FILTER) ====================\n");
    printf("Samples: %lu  |  Drops: %u  |  Thời gian: %.2f s  |  Rate: %.1f Hz\n\n",
           (unsigned long)count, drops, actual_time, sample_rate);

    printf("%-26s | %14s | %14s | %14s\n",
           "Thông số", "Roll", "Pitch", "Heading");
    printf("---------------------------|----------------|----------------|----------------\n");
    printf("%-26s | %+14.7f | %+14.7f | %+14.7f\n",
           "Bias / Mean (rad/s)", mean_rs, mean_ps, mean_hs);
    printf("%-26s | %+14.7f | %+14.7f | %+14.7f\n",
           "Std Dev / Noise (rad/s)", std_rs, std_ps, std_hs);
    printf("%-26s | %+14.7f | %+14.7f | %+14.7f\n",
           "Max (rad/s)", (double)max_rs, (double)max_ps, (double)max_hs);
    printf("%-26s | %+14.7f | %+14.7f | %+14.7f\n",
           "Min (rad/s)", (double)min_rs, (double)min_ps, (double)min_hs);
    printf("---------------------------|----------------|----------------|----------------\n");
    printf("%-26s | %+14.4f | %+14.4f | %+14.4f\n",
           "Góc trôi tích phân (°)", drift_roll_deg, drift_pitch_deg, drift_heading_deg);
    printf("%-26s | %+14.2f | %+14.2f | %+14.2f\n",
           "Tốc độ trôi (°/h)", dph_roll, dph_pitch, dph_heading);
    printf("%-26s | %14.4f | %14.4f | %14.4f\n",
           "Noise density (°/√h)", ard_roll, ard_pitch, ard_heading);
    printf("======================================================================\n\n");

    printf("[GIẢI THÍCH]\n");
    printf("  Bias (Mean)        : Độ lệch tĩnh khi đứng yên — lý tưởng = 0. Giá trị này\n");
    printf("                       gây ra trôi góc tuyến tính theo thời gian.\n");
    printf("  Std Dev (Noise)    : Nhiễu ngẫu nhiên. Nhỏ hơn thì tốt hơn.\n");
    printf("  Góc trôi tích phân : Tổng góc đã bị trôi trong %.0f giây đo thực tế.\n", actual_time);
    printf("  Tốc độ trôi (°/h)  : Ước lượng tuyến tính — IMU sẽ trôi bao nhiêu độ trong 1 giờ.\n");
    printf("  Noise density      : Ước tính ARW (Angle Random Walk) — °/√h.\n");
    printf("                       Cảm biến tốt: < 1 °/√h; IMU MEMS tiêu dùng: 1–10 °/√h.\n");

    return 0;
}
