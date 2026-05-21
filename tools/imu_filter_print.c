#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <time.h>

struct wt_filter_frame {
    uint64_t ktime_ns;
    uint32_t seq;
    uint32_t hz;
    float    roll_speed, pitch_speed, heading_speed;
    float    roll, pitch, heading;
    float    qw, qx, qy, qz;
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

static void print_header(void)
{
    printf("\033[1m");
    printf("┌──────────┬───────┬");
    printf("────────────────────────────────────┬");
    printf("──────────────────────────────────────┐\n");

    printf("│ %-8s │ %-5s │", "seq", "hz");
    printf(" %-11s %-11s %-11s │", "roll", "pitch", "heading");
    printf(" %-9s %-9s %-9s %-9s │\n", "qw", "qx", "qy", "qz");

    printf("│ %-8s │ %-5s │", "", "");
    printf(" %-35s │", "(rad)");
    printf(" %-38s │\n", "(quaternion)");

    printf("├──────────┼───────┼");
    printf("────────────────────────────────────┼");
    printf("──────────────────────────────────────┤\n");
    printf("\033[0m");
}

int main(int argc, char *argv[])
{
    const char *path        = "/dev/wheeltec_imu0_filter";
    int         interval_ms = 200;

    for (int i = 1; i < argc; i++) {
        if (!strncmp(argv[i], "-i", 2) && argv[i][2]) {
            interval_ms = atoi(argv[i] + 2);
        } else if (!strcmp(argv[i], "-i") && i + 1 < argc) {
            interval_ms = atoi(argv[++i]);
        } else {
            path = argv[i];
        }
    }
    if (interval_ms < 1) interval_ms = 1;

    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "open %s: %s\n", path, strerror(errno));
        return 1;
    }

    signal(SIGINT, on_sigint);

    printf("Device : %s   (in mỗi %d ms = %.1f dòng/giây)\n\n",
           path, interval_ms, 1000.0 / interval_ms);
    print_header();

    struct wt_filter_frame f, last;
    uint32_t prev_seq   = UINT32_MAX;
    int      row_count  = 0;
    int      has_last   = 0;
    long     next_print = now_ms();

    while (running) {
        ssize_t n = read(fd, &f, sizeof(f));
        if (n < 0) {
            if (errno == EINTR) break;
            fprintf(stderr, "read: %s\n", strerror(errno));
            break;
        }
        if (n != (ssize_t)sizeof(f)) {
            fprintf(stderr, "short read: %zd bytes\n", n);
            break;
        }

        if (prev_seq != UINT32_MAX && f.seq != prev_seq + 1)
        prev_seq = f.seq;

        last     = f;
        has_last = 1;

        long now = now_ms();
        if (now < next_print)
            continue;
        next_print = now + interval_ms;

        if (!has_last)
            continue;

        if (row_count > 0 && row_count % 30 == 0)
            print_header();
        row_count++;

        printf("│ %8u │ %5u │"
               " %+10.4f  %+10.4f  %+10.4f │"
               " %+8.5f %+8.5f %+8.5f %+8.5f │\n",
               last.seq, last.hz,
               last.roll,  last.pitch,  last.heading,
               last.qw, last.qx, last.qy, last.qz);
    }

    printf("└──────────┴───────┴");
    printf("────────────────────────────────────┴");
    printf("──────────────────────────────────────┘\n");
    printf("Dừng. seq cuối = %u\n", prev_seq);

    close(fd);
    return 0;
}
