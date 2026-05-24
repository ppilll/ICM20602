#define _GNU_SOURCE
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif

#define IIO_SYSFS_ROOT "/sys/bus/iio/devices"
#define ICM20602_SCAN_BYTES 24u
#define DEFAULT_DEV_NAME "icm20602"
#define DEFAULT_SAMPLING_HZ 1000
#define DEFAULT_FIFO_WATERMARK 16
#define DEFAULT_BUFFER_LENGTH 256
#define DEFAULT_READ_BUFFER_BYTES 65536u
#define MAX_TRIGGER_NAME 256

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

typedef enum {
    WAIT_POLL = 0,
    WAIT_EPOLL = 1,
} wait_mode_t;

typedef struct {
    int device_index;                 /* -1 means auto by device name */
    char device_name[128];
    char trigger[MAX_TRIGGER_NAME];   /* exact name or auto:fifo / auto:drdy / auto:any */
    int sampling_frequency;
    int fifo_watermark;
    int buffer_length;
    int buffer_watermark;             /* -1 means use fifo_watermark */
    wait_mode_t wait_mode;
    char *output_path;
    bool csv_header;
    double duration_sec;              /* 0 means unlimited */
    uint64_t max_frames;              /* 0 means unlimited */
    size_t read_buffer_bytes;
    double gap_threshold_mul;
    bool ignore_missing_buffer_watermark;
} app_config_t;

typedef struct {
    uint64_t frames;
    uint64_t read_calls;
    uint64_t successful_read_calls;
    uint64_t wakeups;
    uint64_t eagain_count;
    uint64_t bytes_read;
    uint64_t timestamp_gap_count;
    uint64_t estimated_drops;
    uint64_t partial_bytes_seen;

    bool have_prev_ts;
    bool have_first_ts;
    int64_t first_ts;
    int64_t last_ts;
    int64_t prev_ts;
    int64_t min_dt;
    int64_t max_dt;
    long double sum_dt;
    uint64_t dt_count;

    struct timespec wall_start;
    struct timespec wall_end;
    struct rusage ru_start;
    struct rusage ru_end;
} capture_stats_t;

typedef struct {
    uint8_t carry[ICM20602_SCAN_BYTES];
    size_t carry_len;
} frame_parser_t;

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "IIO userspace capture tool for ICM20602 fixed 24-byte scan layout.\n"
        "CSV is written to stdout by default; statistics are written to stderr.\n"
        "\n"
        "Options:\n"
        "  -d, --device-index N          Use iio:deviceN. Default: auto by --device-name.\n"
        "  -n, --device-name NAME        IIO device name to auto-find. Default: icm20602.\n"
        "  -t, --trigger NAME            Exact trigger name, or auto:fifo, auto:drdy, auto:any.\n"
        "                               Default: auto:fifo.\n"
        "  -f, --sampling-frequency HZ   Write sampling_frequency. Default: 1000.\n"
        "      --fifo-watermark N        Write custom device fifo_watermark. Default: 16.\n"
        "      --buffer-length N         Write buffer/length. Default: 256 scans.\n"
        "      --buffer-watermark N      Write buffer/watermark. Default: same as fifo_watermark.\n"
        "  -m, --mode poll|epoll         Wait mode. epoll uses EPOLLIN|EPOLLET. Default: epoll.\n"
        "  -o, --output FILE             CSV output file. Default: stdout.\n"
        "      --duration SEC            Stop after SEC seconds. Default: unlimited.\n"
        "      --frames N                Stop after N frames. Default: unlimited.\n"
        "      --read-buffer BYTES       read(2) buffer size. Default: 65536.\n"
        "      --gap-mul X               Timestamp gap threshold multiplier. Default: 1.5.\n"
        "      --no-header               Do not print CSV header.\n"
        "      --ignore-missing-buffer-watermark\n"
        "                               Continue if buffer/watermark is absent.\n"
        "  -h, --help                    Show this help.\n"
        "\n"
        "Example:\n"
        "  %s -d 0 -t auto:fifo -f 1000 --fifo-watermark 16 --buffer-length 512 \\\n"
        "     --buffer-watermark 16 -m epoll -o imu.csv --duration 10\n",
        prog, prog);
}

static int timespec_now(struct timespec *ts)
{
    return clock_gettime(CLOCK_MONOTONIC, ts);
}

static double timespec_diff_sec(const struct timespec *a, const struct timespec *b)
{
    return (double)(b->tv_sec - a->tv_sec) + (double)(b->tv_nsec - a->tv_nsec) / 1e9;
}

static double timeval_diff_sec(const struct timeval *a, const struct timeval *b)
{
    return (double)(b->tv_sec - a->tv_sec) + (double)(b->tv_usec - a->tv_usec) / 1e6;
}

static bool path_exists(const char *path)
{
    return access(path, F_OK) == 0;
}

static int read_first_line(const char *path, char *buf, size_t buflen)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        return -errno;
    }

    if (!fgets(buf, (int)buflen, f)) {
        int e = ferror(f) ? errno : ENODATA;
        fclose(f);
        return -e;
    }
    fclose(f);

    size_t len = strlen(buf);
    while (len && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) {
        buf[--len] = '\0';
    }
    return 0;
}

static int write_text_file(const char *path, const char *text)
{
    int fd = open(path, O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        return -errno;
    }

    size_t len = strlen(text);
    ssize_t wr = write(fd, text, len);
    int saved = errno;
    close(fd);

    if (wr < 0) {
        return -saved;
    }
    if ((size_t)wr != len) {
        return -EIO;
    }
    return 0;
}

static int write_int_file(const char *path, int value)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "%d\n", value);
    return write_text_file(path, buf);
}

static int build_path(char *out, size_t outlen, const char *a, const char *b)
{
    int n = snprintf(out, outlen, "%s/%s", a, b);
    if (n < 0 || (size_t)n >= outlen) {
        return -ENAMETOOLONG;
    }
    return 0;
}

static int build_iio_device_base(int index, char *out, size_t outlen)
{
    int n = snprintf(out, outlen, IIO_SYSFS_ROOT "/iio:device%d", index);
    if (n < 0 || (size_t)n >= outlen) {
        return -ENAMETOOLONG;
    }
    return 0;
}

static int find_iio_device_by_name(const char *name)
{
    DIR *dir = opendir(IIO_SYSFS_ROOT);
    if (!dir) {
        return -errno;
    }

    struct dirent *de;
    while ((de = readdir(dir)) != NULL) {
        int idx;
        if (sscanf(de->d_name, "iio:device%d", &idx) != 1) {
            continue;
        }

        char path[PATH_MAX];
        char got[256];
        if (snprintf(path, sizeof(path), IIO_SYSFS_ROOT "/%s/name", de->d_name) >= (int)sizeof(path)) {
            continue;
        }
        if (read_first_line(path, got, sizeof(got)) == 0 && strcmp(got, name) == 0) {
            closedir(dir);
            return idx;
        }
    }

    closedir(dir);
    return -ENODEV;
}

static int find_buffer_dir(const char *base, char *out, size_t outlen)
{
    char p[PATH_MAX];

    if (snprintf(p, sizeof(p), "%s/buffer", base) >= (int)sizeof(p)) {
        return -ENAMETOOLONG;
    }
    if (path_exists(p)) {
        snprintf(out, outlen, "%s", p);
        return 0;
    }

    if (snprintf(p, sizeof(p), "%s/buffer0", base) >= (int)sizeof(p)) {
        return -ENAMETOOLONG;
    }
    if (path_exists(p)) {
        snprintf(out, outlen, "%s", p);
        return 0;
    }

    return -ENOENT;
}

static bool str_has_suffix(const char *s, const char *suffix)
{
    size_t sl = strlen(s);
    size_t su = strlen(suffix);
    return sl >= su && strcmp(s + sl - su, suffix) == 0;
}

static bool is_expected_scan_enable_file(const char *name)
{
    static const char *expected[] = {
        "in_accel_x_en",
        "in_accel_y_en",
        "in_accel_z_en",
        "in_anglvel_x_en",
        "in_anglvel_y_en",
        "in_anglvel_z_en",
        "in_timestamp_en",
    };

    for (size_t i = 0; i < sizeof(expected) / sizeof(expected[0]); i++) {
        if (strcmp(name, expected[i]) == 0) {
            return true;
        }
    }
    return false;
}

static int configure_scan_elements(const char *base)
{
    char scan_dir[PATH_MAX];
    int ret = build_path(scan_dir, sizeof(scan_dir), base, "scan_elements");
    if (ret) {
        return ret;
    }

    DIR *dir = opendir(scan_dir);
    if (!dir) {
        return -errno;
    }

    bool seen_accel_x = false, seen_accel_y = false, seen_accel_z = false;
    bool seen_gyro_x = false, seen_gyro_y = false, seen_gyro_z = false;
    bool seen_ts = false;
    struct dirent *de;

    /* First disable every *_en entry so the fixed 24-byte layout is not polluted by old state. */
    while ((de = readdir(dir)) != NULL) {
        if (!str_has_suffix(de->d_name, "_en")) {
            continue;
        }
        char p[PATH_MAX];
        if (snprintf(p, sizeof(p), "%s/%s", scan_dir, de->d_name) >= (int)sizeof(p)) {
            closedir(dir);
            return -ENAMETOOLONG;
        }
        ret = write_int_file(p, 0);
        if (ret) {
            fprintf(stderr, "WARN: failed to disable %s: %s\n", p, strerror(-ret));
        }
    }

    rewinddir(dir);

    while ((de = readdir(dir)) != NULL) {
        if (!str_has_suffix(de->d_name, "_en")) {
            continue;
        }
        if (!is_expected_scan_enable_file(de->d_name)) {
            continue;
        }

        char p[PATH_MAX];
        if (snprintf(p, sizeof(p), "%s/%s", scan_dir, de->d_name) >= (int)sizeof(p)) {
            closedir(dir);
            return -ENAMETOOLONG;
        }
        ret = write_int_file(p, 1);
        if (ret) {
            closedir(dir);
            fprintf(stderr, "ERROR: failed to enable %s: %s\n", p, strerror(-ret));
            return ret;
        }

        if (strcmp(de->d_name, "in_accel_x_en") == 0) seen_accel_x = true;
        if (strcmp(de->d_name, "in_accel_y_en") == 0) seen_accel_y = true;
        if (strcmp(de->d_name, "in_accel_z_en") == 0) seen_accel_z = true;
        if (strcmp(de->d_name, "in_anglvel_x_en") == 0) seen_gyro_x = true;
        if (strcmp(de->d_name, "in_anglvel_y_en") == 0) seen_gyro_y = true;
        if (strcmp(de->d_name, "in_anglvel_z_en") == 0) seen_gyro_z = true;
        if (strcmp(de->d_name, "in_timestamp_en") == 0) seen_ts = true;
    }

    closedir(dir);

    if (!seen_accel_x || !seen_accel_y || !seen_accel_z ||
        !seen_gyro_x || !seen_gyro_y || !seen_gyro_z || !seen_ts) {
        fprintf(stderr,
                "ERROR: missing required scan element(s). Expected accel xyz, anglvel xyz, timestamp.\n");
        return -ENOENT;
    }

    return 0;
}

static int find_trigger_name(const char *mode, char *out, size_t outlen)
{
    DIR *dir = opendir(IIO_SYSFS_ROOT);
    if (!dir) {
        return -errno;
    }

    bool want_fifo = strcmp(mode, "auto:fifo") == 0 || strcmp(mode, "auto:fifo-wm") == 0;
    bool want_drdy = strcmp(mode, "auto:drdy") == 0;
    bool want_any = strcmp(mode, "auto:any") == 0;

    struct dirent *de;
    while ((de = readdir(dir)) != NULL) {
        int idx;
        if (sscanf(de->d_name, "trigger%d", &idx) != 1) {
            continue;
        }

        char path[PATH_MAX];
        char name[256];
        if (snprintf(path, sizeof(path), IIO_SYSFS_ROOT "/%s/name", de->d_name) >= (int)sizeof(path)) {
            continue;
        }
        if (read_first_line(path, name, sizeof(name)) != 0) {
            continue;
        }

        if ((want_fifo && strstr(name, "fifo") && strstr(name, "wm")) ||
            (want_drdy && strstr(name, "drdy")) ||
            (want_any && (strstr(name, "fifo") || strstr(name, "drdy") || strstr(name, "icm20602")))) {
            snprintf(out, outlen, "%s", name);
            closedir(dir);
            return 0;
        }
    }

    closedir(dir);
    return -ENOENT;
}

static int configure_trigger(const char *base, const char *trigger_opt)
{
    char trig_path[PATH_MAX];
    int ret = snprintf(trig_path, sizeof(trig_path), "%s/trigger/current_trigger", base);
    if (ret < 0 || ret >= (int)sizeof(trig_path)) {
        return -ENAMETOOLONG;
    }

    char trigger_name[MAX_TRIGGER_NAME];
    if (strncmp(trigger_opt, "auto:", 5) == 0) {
        ret = find_trigger_name(trigger_opt, trigger_name, sizeof(trigger_name));
        if (ret) {
            fprintf(stderr, "ERROR: failed to find trigger matching %s: %s\n",
                    trigger_opt, strerror(-ret));
            return ret;
        }
    } else {
        snprintf(trigger_name, sizeof(trigger_name), "%s", trigger_opt);
    }

    char value[MAX_TRIGGER_NAME + 2];
    snprintf(value, sizeof(value), "%s\n", trigger_name);
    ret = write_text_file(trig_path, value);
    if (ret) {
        fprintf(stderr, "ERROR: failed to set current_trigger=%s at %s: %s\n",
                trigger_name, trig_path, strerror(-ret));
        return ret;
    }

    fprintf(stderr, "IIO trigger: %s\n", trigger_name);
    return 0;
}

static int configure_sampling_frequency(const char *base, int hz)
{
    const char *files[] = {
        "in_accel_sampling_frequency",
        "in_anglvel_sampling_frequency",
        "sampling_frequency",
    };

    int success = 0;
    int last_ret = -ENOENT;
    for (size_t i = 0; i < sizeof(files) / sizeof(files[0]); i++) {
        char p[PATH_MAX];
        int n = snprintf(p, sizeof(p), "%s/%s", base, files[i]);
        if (n < 0 || n >= (int)sizeof(p)) {
            return -ENAMETOOLONG;
        }
        if (!path_exists(p)) {
            continue;
        }
        int ret = write_int_file(p, hz);
        if (ret) {
            last_ret = ret;
            fprintf(stderr, "WARN: failed to write %s=%d: %s\n", p, hz, strerror(-ret));
        } else {
            success++;
        }
    }

    if (!success) {
        fprintf(stderr, "ERROR: no sampling_frequency sysfs attribute was writable.\n");
        return last_ret;
    }
    return 0;
}

static int configure_iio_sysfs(const char *base, const char *buffer_dir, const app_config_t *cfg)
{
    char p[PATH_MAX];
    int ret;

    /* Must be disabled before changing scan_elements/current_trigger/buffer settings. */
    if (snprintf(p, sizeof(p), "%s/enable", buffer_dir) >= (int)sizeof(p)) {
        return -ENAMETOOLONG;
    }
    ret = write_int_file(p, 0);
    if (ret) {
        fprintf(stderr, "WARN: failed to disable buffer before configuration: %s\n", strerror(-ret));
    }

    ret = configure_scan_elements(base);
    if (ret) {
        return ret;
    }

    ret = configure_trigger(base, cfg->trigger);
    if (ret) {
        return ret;
    }

    ret = configure_sampling_frequency(base, cfg->sampling_frequency);
    if (ret) {
        return ret;
    }

    if (snprintf(p, sizeof(p), "%s/fifo_watermark", base) >= (int)sizeof(p)) {
        return -ENAMETOOLONG;
    }
    ret = write_int_file(p, cfg->fifo_watermark);
    if (ret) {
        fprintf(stderr, "ERROR: failed to write fifo_watermark: %s\n", strerror(-ret));
        return ret;
    }

    if (snprintf(p, sizeof(p), "%s/length", buffer_dir) >= (int)sizeof(p)) {
        return -ENAMETOOLONG;
    }
    ret = write_int_file(p, cfg->buffer_length);
    if (ret) {
        fprintf(stderr, "ERROR: failed to write buffer/length: %s\n", strerror(-ret));
        return ret;
    }

    if (snprintf(p, sizeof(p), "%s/watermark", buffer_dir) >= (int)sizeof(p)) {
        return -ENAMETOOLONG;
    }
    ret = write_int_file(p, cfg->buffer_watermark);
    if (ret) {
        if (ret == -ENOENT && cfg->ignore_missing_buffer_watermark) {
            fprintf(stderr, "WARN: buffer/watermark not present; continuing.\n");
        } else {
            fprintf(stderr, "ERROR: failed to write buffer/watermark: %s\n", strerror(-ret));
            return ret;
        }
    }

    if (snprintf(p, sizeof(p), "%s/enable", buffer_dir) >= (int)sizeof(p)) {
        return -ENAMETOOLONG;
    }
    ret = write_int_file(p, 1);
    if (ret) {
        fprintf(stderr, "ERROR: failed to write buffer/enable=1: %s\n", strerror(-ret));
        return ret;
    }

    return 0;
}

static int disable_iio_buffer(const char *buffer_dir)
{
    char p[PATH_MAX];
    if (snprintf(p, sizeof(p), "%s/enable", buffer_dir) >= (int)sizeof(p)) {
        return -ENAMETOOLONG;
    }
    return write_int_file(p, 0);
}

static int16_t load_be_s16(const uint8_t *p)
{
    uint16_t u = ((uint16_t)p[0] << 8) | (uint16_t)p[1];
    return (int16_t)u;
}

static int64_t load_le_s64(const uint8_t *p)
{
    uint64_t u = 0;
    for (int i = 0; i < 8; i++) {
        u |= ((uint64_t)p[i]) << (8 * i);
    }
    return (int64_t)u;
}

static uint64_t estimate_gap(int64_t dt_ns, int64_t expected_period_ns, double mul)
{
    if (expected_period_ns <= 0 || dt_ns <= 0) {
        return 0;
    }
    long double threshold = (long double)expected_period_ns * (long double)mul;
    if ((long double)dt_ns <= threshold) {
        return 0;
    }

    long double ratio = (long double)dt_ns / (long double)expected_period_ns;
    uint64_t periods = (uint64_t)(ratio + 0.5L); /* nearest integer */
    if (periods <= 1) {
        return 1;
    }
    return periods - 1;
}

static void process_frame(const uint8_t frame[ICM20602_SCAN_BYTES], FILE *csv,
                          capture_stats_t *st, const app_config_t *cfg)
{
    int16_t ax = load_be_s16(frame + 0);
    int16_t ay = load_be_s16(frame + 2);
    int16_t az = load_be_s16(frame + 4);
    int16_t gx = load_be_s16(frame + 6);
    int16_t gy = load_be_s16(frame + 8);
    int16_t gz = load_be_s16(frame + 10);
    int64_t ts = load_le_s64(frame + 16);

    int64_t dt_ns = 0;
    uint64_t gap = 0;
    int64_t expected_period_ns = 0;
    if (cfg->sampling_frequency > 0) {
        expected_period_ns = (int64_t)(1000000000LL / cfg->sampling_frequency);
    }

    if (st->have_prev_ts) {
        dt_ns = ts - st->prev_ts;
        gap = estimate_gap(dt_ns, expected_period_ns, cfg->gap_threshold_mul);

        if (st->dt_count == 0 || dt_ns < st->min_dt) st->min_dt = dt_ns;
        if (st->dt_count == 0 || dt_ns > st->max_dt) st->max_dt = dt_ns;
        st->sum_dt += (long double)dt_ns;
        st->dt_count++;

        if (gap > 0) {
            st->timestamp_gap_count++;
            st->estimated_drops += gap;
        }
    } else {
        st->have_first_ts = true;
        st->first_ts = ts;
    }

    st->last_ts = ts;
    st->prev_ts = ts;
    st->have_prev_ts = true;

    fprintf(csv,
            "%" PRIu64 ",%d,%d,%d,%d,%d,%d,%" PRId64 ",%" PRId64 ",%" PRIu64 "\n",
            st->frames, ax, ay, az, gx, gy, gz, ts, dt_ns, gap);

    st->frames++;

    if (cfg->max_frames && st->frames >= cfg->max_frames) {
        g_stop = 1;
    }
}

static void process_bytes(const uint8_t *buf, size_t len, FILE *csv,
                          frame_parser_t *parser, capture_stats_t *st,
                          const app_config_t *cfg)
{
    size_t off = 0;

    if (parser->carry_len > 0) {
        size_t need = ICM20602_SCAN_BYTES - parser->carry_len;
        size_t take = len < need ? len : need;
        memcpy(parser->carry + parser->carry_len, buf, take);
        parser->carry_len += take;
        off += take;

        if (parser->carry_len == ICM20602_SCAN_BYTES) {
            process_frame(parser->carry, csv, st, cfg);
            parser->carry_len = 0;
        }
    }

    while (!g_stop && off + ICM20602_SCAN_BYTES <= len) {
        process_frame(buf + off, csv, st, cfg);
        off += ICM20602_SCAN_BYTES;
    }

    if (!g_stop && off < len) {
        parser->carry_len = len - off;
        memcpy(parser->carry, buf + off, parser->carry_len);
        st->partial_bytes_seen += parser->carry_len;
    }
}

static bool duration_expired(const capture_stats_t *st, const app_config_t *cfg)
{
    if (cfg->duration_sec <= 0.0) {
        return false;
    }
    struct timespec now;
    if (timespec_now(&now) != 0) {
        return false;
    }
    return timespec_diff_sec(&st->wall_start, &now) >= cfg->duration_sec;
}

static int timeout_ms_for_duration(const capture_stats_t *st, const app_config_t *cfg)
{
    if (cfg->duration_sec <= 0.0) {
        return 1000;
    }

    struct timespec now;
    if (timespec_now(&now) != 0) {
        return 1000;
    }
    double elapsed = timespec_diff_sec(&st->wall_start, &now);
    double remain = cfg->duration_sec - elapsed;
    if (remain <= 0.0) {
        return 0;
    }
    if (remain > 1.0) {
        return 1000;
    }
    int ms = (int)(remain * 1000.0);
    return ms > 0 ? ms : 1;
}

static int drain_reads_until_eagain(int fd, uint8_t *read_buf, size_t read_buf_len,
                                    FILE *csv, frame_parser_t *parser,
                                    capture_stats_t *st, const app_config_t *cfg)
{
    for (;;) {
        ssize_t n = read(fd, read_buf, read_buf_len);
        st->read_calls++;

        if (n > 0) {
            st->successful_read_calls++;
            st->bytes_read += (uint64_t)n;
            process_bytes(read_buf, (size_t)n, csv, parser, st, cfg);
            if (g_stop || duration_expired(st, cfg)) {
                g_stop = 1;
                return 0;
            }
            continue;
        }

        if (n == 0) {
            return 0;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            st->eagain_count++;
            return 0;
        }

        if (errno == EINTR) {
            continue;
        }

        fprintf(stderr, "ERROR: read failed: %s\n", strerror(errno));
        return -errno;
    }
}

static int run_poll_loop(int fd, uint8_t *read_buf, size_t read_buf_len,
                         FILE *csv, frame_parser_t *parser,
                         capture_stats_t *st, const app_config_t *cfg)
{
    struct pollfd pfd = {
        .fd = fd,
        .events = POLLIN,
    };

    int ret = drain_reads_until_eagain(fd, read_buf, read_buf_len, csv, parser, st, cfg);
    if (ret) {
        return ret;
    }

    while (!g_stop && !duration_expired(st, cfg)) {
        pfd.revents = 0;
        ret = poll(&pfd, 1, timeout_ms_for_duration(st, cfg));
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            fprintf(stderr, "ERROR: poll failed: %s\n", strerror(errno));
            return -errno;
        }
        if (ret == 0) {
            continue;
        }

        st->wakeups++;
        if (pfd.revents & (POLLIN | POLLERR | POLLHUP)) {
            ret = drain_reads_until_eagain(fd, read_buf, read_buf_len, csv, parser, st, cfg);
            if (ret) {
                return ret;
            }
        }
    }

    return 0;
}

static int run_epoll_loop(int fd, uint8_t *read_buf, size_t read_buf_len,
                          FILE *csv, frame_parser_t *parser,
                          capture_stats_t *st, const app_config_t *cfg)
{
    int epfd = epoll_create1(EPOLL_CLOEXEC);
    if (epfd < 0) {
        fprintf(stderr, "ERROR: epoll_create1 failed: %s\n", strerror(errno));
        return -errno;
    }

    struct epoll_event ev;
    memset(&ev, 0, sizeof(ev));
    ev.events = EPOLLIN | EPOLLET;
    ev.data.fd = fd;

    if (epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev) < 0) {
        int e = errno;
        close(epfd);
        fprintf(stderr, "ERROR: epoll_ctl ADD failed: %s\n", strerror(e));
        return -e;
    }

    int ret = drain_reads_until_eagain(fd, read_buf, read_buf_len, csv, parser, st, cfg);
    if (ret) {
        close(epfd);
        return ret;
    }

    while (!g_stop && !duration_expired(st, cfg)) {
        struct epoll_event events[4];
        ret = epoll_wait(epfd, events, 4, timeout_ms_for_duration(st, cfg));
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            int e = errno;
            close(epfd);
            fprintf(stderr, "ERROR: epoll_wait failed: %s\n", strerror(e));
            return -e;
        }
        if (ret == 0) {
            continue;
        }

        for (int i = 0; i < ret; i++) {
            if (events[i].data.fd != fd) {
                continue;
            }
            st->wakeups++;
            if (events[i].events & (EPOLLIN | EPOLLERR | EPOLLHUP)) {
                int r = drain_reads_until_eagain(fd, read_buf, read_buf_len,
                                                 csv, parser, st, cfg);
                if (r) {
                    close(epfd);
                    return r;
                }
            }
            if (g_stop) {
                break;
            }
        }
    }

    close(epfd);
    return 0;
}

static void print_stats(const capture_stats_t *st, const app_config_t *cfg)
{
    double wall_sec = timespec_diff_sec(&st->wall_start, &st->wall_end);
    double user_sec = timeval_diff_sec(&st->ru_start.ru_utime, &st->ru_end.ru_utime);
    double sys_sec = timeval_diff_sec(&st->ru_start.ru_stime, &st->ru_end.ru_stime);
    double cpu_sec = user_sec + sys_sec;
    double cpu_pct = wall_sec > 0.0 ? (cpu_sec / wall_sec) * 100.0 : 0.0;

    double actual_rate_ts = 0.0;
    if (st->frames > 1 && st->last_ts > st->first_ts) {
        actual_rate_ts = ((double)(st->frames - 1) * 1e9) /
                         (double)(st->last_ts - st->first_ts);
    }

    double actual_rate_wall = wall_sec > 0.0 ? (double)st->frames / wall_sec : 0.0;
    double avg_read_size = st->successful_read_calls ?
                           (double)st->bytes_read / (double)st->successful_read_calls : 0.0;
    long double avg_dt = st->dt_count ? st->sum_dt / (long double)st->dt_count : 0.0L;

    fprintf(stderr, "\n=== ICM20602 IIO capture statistics ===\n");
    fprintf(stderr, "total frames:             %" PRIu64 "\n", st->frames);
    fprintf(stderr, "actual sample rate:       %.3f Hz by timestamp", actual_rate_ts);
    fprintf(stderr, "; %.3f Hz by wall clock\n", actual_rate_wall);
    fprintf(stderr, "read calls:               %" PRIu64 "\n", st->read_calls);
    fprintf(stderr, "successful read calls:    %" PRIu64 "\n", st->successful_read_calls);
    fprintf(stderr, "%s wakeups:              %" PRIu64 "\n",
            cfg->wait_mode == WAIT_EPOLL ? "epoll" : "poll", st->wakeups);
    fprintf(stderr, "EAGAIN count:             %" PRIu64 "\n", st->eagain_count);
    fprintf(stderr, "bytes read:               %" PRIu64 "\n", st->bytes_read);
    fprintf(stderr, "avg read size:            %.1f bytes/successful read\n", avg_read_size);
    if (st->dt_count) {
        fprintf(stderr, "timestamp delta min:      %" PRId64 " ns\n", st->min_dt);
        fprintf(stderr, "timestamp delta max:      %" PRId64 " ns\n", st->max_dt);
        fprintf(stderr, "timestamp delta avg:      %.1Lf ns\n", avg_dt);
    } else {
        fprintf(stderr, "timestamp delta min/max/avg: n/a\n");
    }
    fprintf(stderr, "timestamp gap count:      %" PRIu64 "\n", st->timestamp_gap_count);
    fprintf(stderr, "estimated drops:          %" PRIu64 "\n", st->estimated_drops);
    fprintf(stderr, "CPU usage:                %.2f%% process CPU over wall time\n", cpu_pct);
    fprintf(stderr, "wall time:                %.6f s\n", wall_sec);
    fprintf(stderr, "process CPU time:         %.6f s\n", cpu_sec);
    if (st->partial_bytes_seen) {
        fprintf(stderr, "partial bytes carried:    %" PRIu64 "\n", st->partial_bytes_seen);
    }
}

static int parse_int_nonnegative(const char *s, int *out)
{
    char *end = NULL;
    errno = 0;
    long v = strtol(s, &end, 10);
    if (errno || !end || *end != '\0' || v < 0 || v > INT32_MAX) {
        return -EINVAL;
    }
    *out = (int)v;
    return 0;
}

static int parse_int_positive(const char *s, int *out)
{
    char *end = NULL;
    errno = 0;
    long v = strtol(s, &end, 10);
    if (errno || !end || *end != '\0' || v <= 0 || v > INT32_MAX) {
        return -EINVAL;
    }
    *out = (int)v;
    return 0;
}

static int parse_u64(const char *s, uint64_t *out)
{
    char *end = NULL;
    errno = 0;
    unsigned long long v = strtoull(s, &end, 10);
    if (errno || !end || *end != '\0') {
        return -EINVAL;
    }
    *out = (uint64_t)v;
    return 0;
}

static int parse_double_nonnegative(const char *s, double *out)
{
    char *end = NULL;
    errno = 0;
    double v = strtod(s, &end);
    if (errno || !end || *end != '\0' || v < 0.0) {
        return -EINVAL;
    }
    *out = v;
    return 0;
}

static int parse_size(const char *s, size_t *out)
{
    uint64_t v;
    int ret = parse_u64(s, &v);
    if (ret) {
        return ret;
    }
    if (v < ICM20602_SCAN_BYTES || v > (1024ull * 1024ull * 64ull)) {
        return -EINVAL;
    }
    *out = (size_t)v;
    return 0;
}

static int parse_args(int argc, char **argv, app_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->device_index = -1;
    snprintf(cfg->device_name, sizeof(cfg->device_name), "%s", DEFAULT_DEV_NAME);
    snprintf(cfg->trigger, sizeof(cfg->trigger), "%s", "auto:fifo");
    cfg->sampling_frequency = DEFAULT_SAMPLING_HZ;
    cfg->fifo_watermark = DEFAULT_FIFO_WATERMARK;
    cfg->buffer_length = DEFAULT_BUFFER_LENGTH;
    cfg->buffer_watermark = -1;
    cfg->wait_mode = WAIT_EPOLL;
    cfg->csv_header = true;
    cfg->duration_sec = 0.0;
    cfg->max_frames = 0;
    cfg->read_buffer_bytes = DEFAULT_READ_BUFFER_BYTES;
    cfg->gap_threshold_mul = 1.5;
    cfg->ignore_missing_buffer_watermark = false;

    enum {
        OPT_FIFO_WATERMARK = 1000,
        OPT_BUFFER_LENGTH,
        OPT_BUFFER_WATERMARK,
        OPT_DURATION,
        OPT_FRAMES,
        OPT_READ_BUFFER,
        OPT_GAP_MUL,
        OPT_NO_HEADER,
        OPT_IGNORE_MISSING_BUFFER_WATERMARK,
    };

    static const struct option long_opts[] = {
        {"device-index", required_argument, NULL, 'd'},
        {"device-name", required_argument, NULL, 'n'},
        {"trigger", required_argument, NULL, 't'},
        {"sampling-frequency", required_argument, NULL, 'f'},
        {"fifo-watermark", required_argument, NULL, OPT_FIFO_WATERMARK},
        {"buffer-length", required_argument, NULL, OPT_BUFFER_LENGTH},
        {"buffer-watermark", required_argument, NULL, OPT_BUFFER_WATERMARK},
        {"mode", required_argument, NULL, 'm'},
        {"output", required_argument, NULL, 'o'},
        {"duration", required_argument, NULL, OPT_DURATION},
        {"frames", required_argument, NULL, OPT_FRAMES},
        {"read-buffer", required_argument, NULL, OPT_READ_BUFFER},
        {"gap-mul", required_argument, NULL, OPT_GAP_MUL},
        {"no-header", no_argument, NULL, OPT_NO_HEADER},
        {"ignore-missing-buffer-watermark", no_argument, NULL, OPT_IGNORE_MISSING_BUFFER_WATERMARK},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0},
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "d:n:t:f:m:o:h", long_opts, NULL)) != -1) {
        int ret;
        switch (opt) {
        case 'd':
            ret = parse_int_nonnegative(optarg, &cfg->device_index);
            if (ret) return ret;
            break;
        case 'n':
            snprintf(cfg->device_name, sizeof(cfg->device_name), "%s", optarg);
            break;
        case 't':
            snprintf(cfg->trigger, sizeof(cfg->trigger), "%s", optarg);
            break;
        case 'f':
            ret = parse_int_positive(optarg, &cfg->sampling_frequency);
            if (ret) return ret;
            break;
        case 'm':
            if (strcmp(optarg, "poll") == 0) {
                cfg->wait_mode = WAIT_POLL;
            } else if (strcmp(optarg, "epoll") == 0) {
                cfg->wait_mode = WAIT_EPOLL;
            } else {
                return -EINVAL;
            }
            break;
        case 'o':
            cfg->output_path = optarg;
            break;
        case OPT_FIFO_WATERMARK:
            ret = parse_int_positive(optarg, &cfg->fifo_watermark);
            if (ret) return ret;
            break;
        case OPT_BUFFER_LENGTH:
            ret = parse_int_positive(optarg, &cfg->buffer_length);
            if (ret) return ret;
            break;
        case OPT_BUFFER_WATERMARK:
            ret = parse_int_positive(optarg, &cfg->buffer_watermark);
            if (ret) return ret;
            break;
        case OPT_DURATION:
            ret = parse_double_nonnegative(optarg, &cfg->duration_sec);
            if (ret) return ret;
            break;
        case OPT_FRAMES:
            ret = parse_u64(optarg, &cfg->max_frames);
            if (ret) return ret;
            break;
        case OPT_READ_BUFFER:
            ret = parse_size(optarg, &cfg->read_buffer_bytes);
            if (ret) return ret;
            break;
        case OPT_GAP_MUL:
            ret = parse_double_nonnegative(optarg, &cfg->gap_threshold_mul);
            if (ret) return ret;
            if (cfg->gap_threshold_mul < 1.0) return -EINVAL;
            break;
        case OPT_NO_HEADER:
            cfg->csv_header = false;
            break;
        case OPT_IGNORE_MISSING_BUFFER_WATERMARK:
            cfg->ignore_missing_buffer_watermark = true;
            break;
        case 'h':
            usage(argv[0]);
            exit(EXIT_SUCCESS);
        default:
            usage(argv[0]);
            return -EINVAL;
        }
    }

    if (cfg->buffer_watermark < 0) {
        cfg->buffer_watermark = cfg->fifo_watermark;
    }

    cfg->read_buffer_bytes -= cfg->read_buffer_bytes % ICM20602_SCAN_BYTES;
    if (cfg->read_buffer_bytes < ICM20602_SCAN_BYTES) {
        cfg->read_buffer_bytes = ICM20602_SCAN_BYTES;
    }

    return 0;
}

int main(int argc, char **argv)
{
    app_config_t cfg;
    int ret = parse_args(argc, argv, &cfg);
    if (ret) {
        usage(argv[0]);
        fprintf(stderr, "ERROR: invalid arguments: %s\n", strerror(-ret));
        return EXIT_FAILURE;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    int dev_index = cfg.device_index;
    if (dev_index < 0) {
        dev_index = find_iio_device_by_name(cfg.device_name);
        if (dev_index < 0) {
            fprintf(stderr, "ERROR: failed to find IIO device named '%s': %s\n",
                    cfg.device_name, strerror(-dev_index));
            return EXIT_FAILURE;
        }
    }

    char base[PATH_MAX];
    ret = build_iio_device_base(dev_index, base, sizeof(base));
    if (ret) {
        fprintf(stderr, "ERROR: failed to build sysfs path: %s\n", strerror(-ret));
        return EXIT_FAILURE;
    }

    char buffer_dir[PATH_MAX];
    ret = find_buffer_dir(base, buffer_dir, sizeof(buffer_dir));
    if (ret) {
        fprintf(stderr, "ERROR: failed to find buffer directory under %s: %s\n",
                base, strerror(-ret));
        return EXIT_FAILURE;
    }

    fprintf(stderr, "IIO device: iio:device%d (%s)\n", dev_index, base);
    fprintf(stderr, "IIO buffer dir: %s\n", buffer_dir);

    ret = configure_iio_sysfs(base, buffer_dir, &cfg);
    if (ret) {
        disable_iio_buffer(buffer_dir);
        return EXIT_FAILURE;
    }

    char devnode[PATH_MAX];
    if (snprintf(devnode, sizeof(devnode), "/dev/iio:device%d", dev_index) >= (int)sizeof(devnode)) {
        disable_iio_buffer(buffer_dir);
        fprintf(stderr, "ERROR: devnode path too long\n");
        return EXIT_FAILURE;
    }

    int fd = open(devnode, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        int e = errno;
        disable_iio_buffer(buffer_dir);
        fprintf(stderr, "ERROR: open %s failed: %s\n", devnode, strerror(e));
        return EXIT_FAILURE;
    }

    FILE *csv = stdout;
    if (cfg.output_path) {
        csv = fopen(cfg.output_path, "w");
        if (!csv) {
            int e = errno;
            close(fd);
            disable_iio_buffer(buffer_dir);
            fprintf(stderr, "ERROR: failed to open output file %s: %s\n", cfg.output_path, strerror(e));
            return EXIT_FAILURE;
        }
    }

    uint8_t *read_buf = malloc(cfg.read_buffer_bytes);
    if (!read_buf) {
        if (csv != stdout) fclose(csv);
        close(fd);
        disable_iio_buffer(buffer_dir);
        fprintf(stderr, "ERROR: malloc read buffer failed\n");
        return EXIT_FAILURE;
    }

    if (cfg.csv_header) {
        fprintf(csv, "seq,ax,ay,az,gx,gy,gz,timestamp_ns,dt_ns,gap\n");
    }

    capture_stats_t stats;
    memset(&stats, 0, sizeof(stats));
    frame_parser_t parser;
    memset(&parser, 0, sizeof(parser));

    timespec_now(&stats.wall_start);
    getrusage(RUSAGE_SELF, &stats.ru_start);

    if (cfg.wait_mode == WAIT_EPOLL) {
        ret = run_epoll_loop(fd, read_buf, cfg.read_buffer_bytes, csv, &parser, &stats, &cfg);
    } else {
        ret = run_poll_loop(fd, read_buf, cfg.read_buffer_bytes, csv, &parser, &stats, &cfg);
    }

    fflush(csv);
    getrusage(RUSAGE_SELF, &stats.ru_end);
    timespec_now(&stats.wall_end);

    int dis_ret = disable_iio_buffer(buffer_dir);
    if (dis_ret) {
        fprintf(stderr, "WARN: failed to disable buffer on exit: %s\n", strerror(-dis_ret));
    }

    print_stats(&stats, &cfg);

    free(read_buf);
    if (csv != stdout) {
        fclose(csv);
    }
    close(fd);

    return ret ? EXIT_FAILURE : EXIT_SUCCESS;
}
