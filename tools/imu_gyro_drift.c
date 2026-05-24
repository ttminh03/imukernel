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

struct wt_raw_frame {
    uint64_t ktime_ns;
    uint32_t seq;
    uint32_t hz;
    float    gyro_x, gyro_y, gyro_z;
    float    accel_x, accel_y, accel_z;
    float    mag_x, mag_y, mag_z;
    int64_t  hw_timestamp;
} __attribute__((packed));

static volatile int running = 1;
static void on_sigint(int sig) { (void)sig; running = 0; }

static long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

int main(int argc, char *argv[])
{
    const char *path = "/dev/wheeltec_imu0_raw";
    int duration_sec = 10; // Đánh giá trong 10 giây mặc định

    for (int i = 1; i < argc; i++) {
        if (!strncmp(argv[i], "-t", 2) && argv[i][2]) {
            duration_sec = atoi(argv[i] + 2);
        } else if (!strcmp(argv[i], "-t") && i + 1 < argc) {
            duration_sec = atoi(argv[++i]);
        } else {
            path = argv[i];
        }
    }

    if (duration_sec <= 0) duration_sec = 1;

    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "Lỗi khi mở luồng %s: %s\n", path, strerror(errno));
        return 1;
    }

    signal(SIGINT, on_sigint);

    printf("Đang đánh giá drift (độ lệch/trôi) con quay hồi chuyển (Gyroscope)...\n");
    printf("\033[1;31mYÊU CẦU: HÃY GIỮ IMU ĐỨNG YÊN HOÀN TOÀN TRONG SUỐT QUÁ TRÌNH NÀY!\033[0m\n");
    printf("Device   : %s\n", path);
    printf("Thời gian: %d giây (Bấm Ctrl+C để kết thúc sớm)\n\n", duration_sec);

    struct wt_raw_frame f;
    
    long start_time = now_ms();
    long end_time = start_time + duration_sec * 1000L;
    
    uint64_t count = 0;
    
    double sum_x = 0, sum_y = 0, sum_z = 0;
    double sum_sq_x = 0, sum_sq_y = 0, sum_sq_z = 0;
    float max_x = -1e9, max_y = -1e9, max_z = -1e9;
    float min_x = 1e9, min_y = 1e9, min_z = 1e9;
    
    uint32_t prev_seq = UINT32_MAX;
    uint32_t drop_count = 0;

    while (running) {
        long current_time = now_ms();
        if (current_time >= end_time) {
            break;
        }

        ssize_t n = read(fd, &f, sizeof(f));
        if (n < 0) {
            if (errno == EINTR) break;
            fprintf(stderr, "read error: %s\n", strerror(errno));
            break;
        }
        if (n != (ssize_t)sizeof(f)) {
            fprintf(stderr, "short read: %zd bytes\n", n);
            break;
        }

        if (prev_seq != UINT32_MAX) {
            if (f.seq == prev_seq) {
                // Đọc trùng frame cũ do driver không block. 
                // Bỏ qua, không đưa vào tính toán để không làm sai lệch tần số Hz và giá trị.
                continue;
            } else if (f.seq > prev_seq + 1) {
                drop_count += (f.seq - prev_seq - 1);
            } else if (f.seq < prev_seq) {
                // Trường hợp seq bị reset (ví dụ IMU khởi động lại) hoặc out of order
                // Không tính là rớt để tránh tính sai
            }
        }
        prev_seq = f.seq;

        // Tích luỹ giá trị
        sum_x += f.gyro_x;
        sum_y += f.gyro_y;
        sum_z += f.gyro_z;
        
        // Bình phương giá trị để tính phương sai
        sum_sq_x += (double)f.gyro_x * f.gyro_x;
        sum_sq_y += (double)f.gyro_y * f.gyro_y;
        sum_sq_z += (double)f.gyro_z * f.gyro_z;
        
        // Tìm giá trị max, min cho từng thiết bị
        if (f.gyro_x > max_x) max_x = f.gyro_x;
        if (f.gyro_y > max_y) max_y = f.gyro_y;
        if (f.gyro_z > max_z) max_z = f.gyro_z;
        
        if (f.gyro_x < min_x) min_x = f.gyro_x;
        if (f.gyro_y < min_y) min_y = f.gyro_y;
        if (f.gyro_z < min_z) min_z = f.gyro_z;

        count++;
        
        // In tiến độ
        if (count % 2000000 == 0) {
            int elapsed = (current_time - start_time) / 1000;
            printf("\rTiến độ: %d / %d giây (Đã lấy %lu mẫu)", elapsed, duration_sec, count);
            fflush(stdout);
        }
    }

    printf("\n\n");
    close(fd);

    if (count == 0) {
        printf("Lỗi: Không thu thập được dữ liệu nào!\n");
        return 1;
    }

    // Thời gian lấy mẫu thức tế
    float actual_time = (float)(now_ms() - start_time) / 1000.0f;
    float sample_rate = count / actual_time;

    // Tính giá trị trung bình (Drift / Bias offset của Zero-Rate Level)
    double mean_x = sum_x / count;
    double mean_y = sum_y / count;
    double mean_z = sum_z / count;

    // Tính phương sai
    double var_x = (sum_sq_x / count) - (mean_x * mean_x);
    double var_y = (sum_sq_y / count) - (mean_y * mean_y);
    double var_z = (sum_sq_z / count) - (mean_z * mean_z);

    // Độ lệch chuẩn (Noise content)
    double std_x = sqrt(var_x > 0 ? var_x : 0);
    double std_y = sqrt(var_y > 0 ? var_y : 0);
    double std_z = sqrt(var_z > 0 ? var_z : 0);
    
    // Tính toán góc bị trôi (Drift Angle)
    // Tích phân: góc trôi (radian) = bias (rad/s) * actual_time (s)
    // Đổi sang độ: rad * (180 / PI)
    double drift_deg_x = mean_x * actual_time * (180.0 / M_PI);
    double drift_deg_y = mean_y * actual_time * (180.0 / M_PI);
    double drift_deg_z = mean_z * actual_time * (180.0 / M_PI);
    
    // Tốc độ trôi trên giờ (Degrees Per Hour - dph)
    double drift_dph_x = mean_x * 3600.0 * (180.0 / M_PI);
    double drift_dph_y = mean_y * 3600.0 * (180.0 / M_PI);
    double drift_dph_z = mean_z * 3600.0 * (180.0 / M_PI);

    printf("========== KẾT QUẢ ĐÁNH GIÁ GYRO DRIFT ==========\n");
    printf("Tổng số mẫu đọc (Samples) : %lu\n", count);
    printf("Số gói bị rớt (Drops)     : %u\n", drop_count);
    printf("Thời gian lấy mẫu thực tế : %.2f giây\n", actual_time);
    printf("Tốc độ lấy mẫu trung bình : %.1f Hz\n", sample_rate);
    printf("----------------------------------------------------------------------\n");
    printf("%-20s | %-15s | %-15s | %-15s |\n", "Thông số sinh ra", "Trục X", "Trục Y", "Trục Z");
    printf("---------------------|-----------------|-----------------|-----------------|\n");
    printf("%-20s | %+15.7f | %+15.7f | %+15.7f |\n", "Mean (rad/s)", mean_x, mean_y, mean_z);
    printf("%-20s | %+15.7f | %+15.7f | %+15.7f |\n", "Std Dev (rad/s)", std_x, std_y, std_z);
    printf("%-20s | %+15.7f | %+15.7f | %+15.7f |\n", "Max (rad/s)", max_x, max_y, max_z);
    printf("%-20s | %+15.7f | %+15.7f | %+15.7f |\n", "Min (rad/s)", min_x, min_y, min_z);
    printf("---------------------|-----------------|-----------------|-----------------|\n");
    printf("%-20s | %+15.7f | %+15.7f | %+15.7f |\n", "Góc trôi (Độ)", drift_deg_x, drift_deg_y, drift_deg_z);
    printf("%-20s | %+15.7f | %+15.7f | %+15.7f |\n", "Tốc độ trôi (Deg/h)", drift_dph_x, drift_dph_y, drift_dph_z);
    printf("======================================================================\n");
    printf("\n[GIẢI THÍCH]:\n");
    printf(" - Mean (Bias/Drift): Chính là độ lệch trôi tĩnh. Ở trạng thái đứng yên,\n");
    printf("   lý thuyết con quay hồi chuyển sẽ trả về 0. Giá trị Mean này là sai số thực tế.\n");
    printf(" - Std Dev (Noise)  : Độ nhiễu của tín hiệu. Giá trị càng nhỏ càng tốt.\n");
    printf(" - Góc trôi (Độ)    : Tổng số đo góc đã bị trôi đi trong suốt quá trình đo.\n");
    printf(" - Tốc độ trôi      : Ước lượng số độ sẽ bị trôi trong 1 giờ (Degrees per hour).\n");

    return 0;
}
