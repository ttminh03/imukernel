/*
 * imu_drift_compare.c
 *
 * Đọc song song raw (gyro_z) và filter (heading_speed), tích phân cả hai,
 * in bảng so sánh drift cuối cùng.
 *
 * Build:  gcc -O2 -pthread -o imu_drift_compare imu_drift_compare.c -lm
 * Dùng:   ./imu_drift_compare [-t <giây>]
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
#include <pthread.h>

/* ---------- structs khớp kernel __packed ---------- */

struct raw_frame {
    uint64_t ktime_ns;
    uint32_t seq;
    uint32_t hz;
    float gyro_x, gyro_y, gyro_z;   /* rad/s */
    float accel_x, accel_y, accel_z;
    float mag_x, mag_y, mag_z;
    int64_t hw_timestamp;            /* µs */
} __attribute__((packed));           /* 60 bytes */

struct filter_frame {
    uint64_t ktime_ns;
    uint32_t seq;
    uint32_t hz;
    float roll_speed, pitch_speed, heading_speed; /* rad/s */
    float roll, pitch, heading;                   /* rad  */
    float qw, qx, qy, qz;
    int64_t hw_timestamp;                         /* µs  */
} __attribute__((packed));                        /* 64 bytes */

/* ---------- shared state ---------- */

static volatile int running = 1;
static void on_sigint(int sig) { (void)sig; running = 0; }

static double now_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* kết quả mỗi luồng ghi vào đây */
struct result {
    const char *label;       /* "RAW gyro_z" hoặc "FILTER heading_speed" */
    uint64_t    count;
    uint32_t    drops;
    double      mean;        /* rad/s  */
    double      std;         /* rad/s  */
    double      integ_rad;   /* tích phân rad — cập nhật liên tục trong vòng lặp */
    double      actual_time; /* giây thực */
    double      sample_rate; /* Hz */
    volatile int live_ready; /* 1 khi đã có mẫu đầu tiên */
};

/* ---------- thread raw ---------- */

struct thread_arg {
    const char  *path;
    int          duration_sec;
    struct result *out;
};

static void *thread_raw(void *arg)
{
    struct thread_arg *a = arg;
    struct result     *r = a->out;
    r->label = "RAW  gyro_z";

    int fd = open(a->path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "[raw ] open %s: %s\n", a->path, strerror(errno));
        return NULL;
    }

    struct raw_frame f;
    uint32_t prev_seq  = UINT32_MAX;
    uint64_t prev_ktime = 0;
    double sum = 0, sum_sq = 0;
    double integ = 0;
    uint64_t count = 0;
    uint32_t drops = 0;

    double t_end = now_sec() + a->duration_sec;

    while (running && now_sec() < t_end) {
        ssize_t n = read(fd, &f, sizeof(f));
        if (n < 0) { if (errno == EINTR) break; break; }
        if (n != (ssize_t)sizeof(f)) break;

        if (prev_seq != UINT32_MAX) {
            if (f.seq == prev_seq) continue;
            if (f.seq > prev_seq + 1) drops += f.seq - prev_seq - 1;
        }
        prev_seq = f.seq;

        if (prev_ktime && f.ktime_ns > prev_ktime)
            integ += f.gyro_z * (double)(f.ktime_ns - prev_ktime) * 1e-9;
        prev_ktime = f.ktime_ns;

        sum    += f.gyro_z;
        sum_sq += (double)f.gyro_z * f.gyro_z;
        count++;
        r->integ_rad  = integ;  /* cập nhật để main thread đọc tiến độ */
        r->live_ready = 1;
    }
    close(fd);

    double actual = now_sec() - (t_end - a->duration_sec);
    r->count       = count;
    r->drops       = drops;
    r->mean        = count ? sum / count : 0;
    r->std         = count ? sqrt(fmax(0, sum_sq/count - r->mean*r->mean)) : 0;
    r->integ_rad   = integ;
    r->actual_time = actual;
    r->sample_rate = count ? count / actual : 0;
    return NULL;
}

/* ---------- thread filter ---------- */

static void *thread_filter(void *arg)
{
    struct thread_arg *a = arg;
    struct result     *r = a->out;
    r->label = "FILTER heading_speed";

    int fd = open(a->path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "[flt ] open %s: %s\n", a->path, strerror(errno));
        return NULL;
    }

    struct filter_frame f;
    uint32_t prev_seq   = UINT32_MAX;
    uint64_t prev_ktime = 0;
    double sum = 0, sum_sq = 0;
    double integ = 0;
    uint64_t count = 0;
    uint32_t drops = 0;

    double t_end = now_sec() + a->duration_sec;

    while (running && now_sec() < t_end) {
        ssize_t n = read(fd, &f, sizeof(f));
        if (n < 0) { if (errno == EINTR) break; break; }
        if (n != (ssize_t)sizeof(f)) break;

        if (prev_seq != UINT32_MAX) {
            if (f.seq == prev_seq) continue;
            if (f.seq > prev_seq + 1) drops += f.seq - prev_seq - 1;
        }
        prev_seq = f.seq;

        if (prev_ktime && f.ktime_ns > prev_ktime)
            integ += f.heading_speed * (double)(f.ktime_ns - prev_ktime) * 1e-9;
        prev_ktime = f.ktime_ns;

        sum    += f.heading_speed;
        sum_sq += (double)f.heading_speed * f.heading_speed;
        count++;
        r->integ_rad  = integ;  /* cập nhật để main thread đọc tiến độ */
        r->live_ready = 1;
    }
    close(fd);

    double actual = now_sec() - (t_end - a->duration_sec);
    r->count       = count;
    r->drops       = drops;
    r->mean        = count ? sum / count : 0;
    r->std         = count ? sqrt(fmax(0, sum_sq/count - r->mean*r->mean)) : 0;
    r->integ_rad   = integ;
    r->actual_time = actual;
    r->sample_rate = count ? count / actual : 0;
    return NULL;
}

/* ---------- in tiến độ trong main thread ---------- */

static void print_progress(double elapsed, int total,
                           double raw_integ, double flt_integ)
{
    printf("\r[%5.1f/%ds]  "
           "RAW  gyro_z: %+.4f°   "
           "FILTER heading: %+.4f°   ",
           elapsed, total,
           raw_integ * (180.0 / M_PI),
           flt_integ * (180.0 / M_PI));
    fflush(stdout);
}

/* ---------- main ---------- */

int main(int argc, char *argv[])
{
    int duration_sec = 60;
    const char *raw_path    = "/dev/wheeltec_imu0_raw";
    const char *filter_path = "/dev/wheeltec_imu0_filter";

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-t") && i + 1 < argc)
            duration_sec = atoi(argv[++i]);
        else if (!strncmp(argv[i], "-t", 2) && argv[i][2])
            duration_sec = atoi(argv[i] + 2);
    }
    if (duration_sec <= 0) duration_sec = 1;

    if (sizeof(struct raw_frame) != 60) {
        fprintf(stderr, "Lỗi: sizeof(raw_frame)=%zu cần 60\n", sizeof(struct raw_frame));
        return 1;
    }
    if (sizeof(struct filter_frame) != 64) {
        fprintf(stderr, "Lỗi: sizeof(filter_frame)=%zu cần 64\n", sizeof(struct filter_frame));
        return 1;
    }

    signal(SIGINT, on_sigint);

    printf("=== IMU DRIFT COMPARE: RAW vs FILTER ===\n");
    printf("\033[1;33mGIỮ IMU ĐỨNG YÊN HOÀN TOÀN!\033[0m\n");
    printf("Thời gian: %d giây  (Ctrl+C dừng sớm)\n\n", duration_sec);

    struct result res_raw = {0}, res_flt = {0};

    struct thread_arg arg_raw = { raw_path,    duration_sec, &res_raw };
    struct thread_arg arg_flt = { filter_path, duration_sec, &res_flt };

    pthread_t tid_raw, tid_flt;
    pthread_create(&tid_raw, NULL, thread_raw,    &arg_raw);
    pthread_create(&tid_flt, NULL, thread_filter, &arg_flt);

    /* tiến độ: đọc integ từ hai thread qua shared result (race ok — double write) */
    double t_start = now_sec();
    double t_end   = t_start + duration_sec;
    double t_last  = t_start;

    while (running && now_sec() < t_end) {
        usleep(200000); /* 200ms */
        double now = now_sec();
        if (now - t_last >= 2.0) {
            t_last = now;
            print_progress(now - t_start, duration_sec,
                           res_raw.integ_rad, res_flt.integ_rad);
        }
    }

    pthread_join(tid_raw, NULL);
    pthread_join(tid_flt, NULL);
    printf("\n\n");

    /* ---------- in kết quả ---------- */

    struct result *rs[2] = { &res_raw, &res_flt };
    const char    *R2D   = NULL; (void)R2D;
    double         r2d   = 180.0 / M_PI;

    printf("==================== KẾT QUẢ SO SÁNH DRIFT (trục Z / heading) ====================\n");
    printf("%-26s | %22s | %22s\n", "Thông số", res_raw.label, res_flt.label);
    printf("---------------------------|------------------------|------------------------\n");

    for (int pass = 0; pass < 8; pass++) {
        double v0, v1;
        const char *label;
        switch (pass) {
        case 0:
            label = "Samples";
            v0 = (double)rs[0]->count; v1 = (double)rs[1]->count;
            printf("%-26s | %22.0f | %22.0f\n", label, v0, v1);
            continue;
        case 1:
            label = "Drops";
            v0 = rs[0]->drops; v1 = rs[1]->drops;
            printf("%-26s | %22.0f | %22.0f\n", label, v0, v1);
            continue;
        case 2:
            label = "Rate (Hz)";
            printf("%-26s | %22.1f | %22.1f\n", label,
                   rs[0]->sample_rate, rs[1]->sample_rate);
            continue;
        case 3: label = "Bias/Mean (rad/s)";
                v0 = rs[0]->mean;  v1 = rs[1]->mean;   break;
        case 4: label = "Std Dev (rad/s)";
                v0 = rs[0]->std;   v1 = rs[1]->std;    break;
        case 5: label = "Bias/Mean (°/s)";
                v0 = rs[0]->mean  * r2d; v1 = rs[1]->mean  * r2d; break;
        case 6: label = "Góc trôi tích phân (°)";
                v0 = rs[0]->integ_rad * r2d;
                v1 = rs[1]->integ_rad * r2d; break;
        case 7: label = "Tốc độ trôi (°/h)";
                v0 = rs[0]->mean * 3600.0 * r2d;
                v1 = rs[1]->mean * 3600.0 * r2d; break;
        default: continue;
        }
        printf("%-26s | %+22.6f | %+22.6f\n", label, v0, v1);
    }

    /* Hiệu giữa hai nguồn */
    double diff_bias  = (res_flt.mean - res_raw.mean) * r2d;
    double diff_integ = (res_flt.integ_rad - res_raw.integ_rad) * r2d;
    printf("---------------------------|------------------------|------------------------\n");
    printf("%-26s | %+22.6f°/s  (FILTER - RAW)\n", "Chênh lệch bias", diff_bias);
    printf("%-26s | %+22.4f°    (FILTER - RAW)\n", "Chênh lệch tích phân", diff_integ);
    printf("===================================================================================\n\n");

    printf("[KẾT LUẬN]\n");
    double drift_heading = res_flt.integ_rad * r2d;
    double drift_raw     = res_raw.integ_rad * r2d;
    double dph_heading   = res_flt.mean * 3600.0 * r2d;
    double dph_raw       = res_raw.mean * 3600.0 * r2d;
    printf("  Trong %.0fs đo: RAW gyro_z trôi %+.4f°,  FILTER heading trôi %+.4f°\n",
           res_flt.actual_time, drift_raw, drift_heading);
    printf("  Ước lượng 1 giờ:  RAW %+.1f°/h,  FILTER %+.1f°/h\n",
           dph_raw, dph_heading);
    if (fabs(dph_heading) < 10.0)
        printf("  -> Drift FILTER rất tốt (< 10°/h), phù hợp điều khiển hướng.\n");
    else if (fabs(dph_heading) < 30.0)
        printf("  -> Drift FILTER chấp nhận được (< 30°/h) cho AGV/xe tốc độ thấp.\n");
    else
        printf("  -> Drift FILTER cao (> 30°/h), nên chạy tare/calib lại.\n");
    printf("  Chênh lệch RAW vs FILTER = %.4f°/h → bộ lọc AHRS %s bù bias.\n",
           fabs(dph_raw - dph_heading),
           fabs(dph_heading) < fabs(dph_raw) ? "có" : "không hiệu quả trong việc");

    return 0;
}
