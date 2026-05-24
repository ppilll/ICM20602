// iio_icm20602_reader.cpp
// C++17 Linux IIO userspace IMU capture tool for ICM20602-style scan layout.
// Build: g++ -std=c++17 -O2 -Wall -Wextra -pedantic -o iio_icm20602_reader iio_icm20602_reader.cpp

#include <sys/epoll.h>
#include <sys/poll.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

volatile sig_atomic_t g_stop = 0;

void on_signal(int) { g_stop = 1; }

std::string errno_string(int e = errno) {
    return std::string(std::strerror(e));
}

[[noreturn]] void throw_errno(const std::string& what) {
    throw std::runtime_error(what + ": " + errno_string());
}

bool path_exists(const std::string& path) {
    struct stat st {};
    return ::stat(path.c_str(), &st) == 0;
}

std::string basename_of(const std::string& path) {
    auto p = path.find_last_of('/');
    if (p == std::string::npos) return path;
    return path.substr(p + 1);
}

std::string join_path(const std::string& a, const std::string& b) {
    if (a.empty()) return b;
    if (a.back() == '/') return a + b;
    return a + "/" + b;
}

int64_t monotonic_ns() {
    struct timespec ts {};
    if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        throw_errno("clock_gettime(CLOCK_MONOTONIC)");
    }
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

double timeval_to_sec(const timeval& tv) {
    return static_cast<double>(tv.tv_sec) + static_cast<double>(tv.tv_usec) / 1e6;
}

// -----------------------------------------------------------------------------
// 1. Fd RAII wrapper
// -----------------------------------------------------------------------------
class Fd {
public:
    Fd() = default;
    explicit Fd(int fd) : fd_(fd) {}
    ~Fd() { reset(); }

    Fd(const Fd&) = delete;
    Fd& operator=(const Fd&) = delete;

    Fd(Fd&& other) noexcept : fd_(other.fd_) { other.fd_ = -1; }
    Fd& operator=(Fd&& other) noexcept {
        if (this != &other) {
            reset();
            fd_ = other.fd_;
            other.fd_ = -1;
        }
        return *this;
    }

    int get() const { return fd_; }
    bool valid() const { return fd_ >= 0; }

    int release() {
        int tmp = fd_;
        fd_ = -1;
        return tmp;
    }

    void reset(int new_fd = -1) {
        if (fd_ >= 0) {
            while (::close(fd_) != 0 && errno == EINTR) {}
        }
        fd_ = new_fd;
    }

private:
    int fd_ = -1;
};

// -----------------------------------------------------------------------------
// 2. Sysfs helper
// -----------------------------------------------------------------------------
class Sysfs {
public:
    explicit Sysfs(std::string root) : root_(std::move(root)) {}

    const std::string& root() const { return root_; }

    std::string path(const std::string& rel) const {
        return join_path(root_, rel);
    }

    bool exists(const std::string& rel) const {
        return path_exists(path(rel));
    }

    std::string read_string(const std::string& rel) const {
        std::string p = path(rel);
        Fd fd(::open(p.c_str(), O_RDONLY | O_CLOEXEC));
        if (!fd.valid()) throw_errno("open " + p);

        std::string out;
        char buf[256];
        for (;;) {
            ssize_t n = ::read(fd.get(), buf, sizeof(buf));
            if (n > 0) {
                out.append(buf, buf + n);
                continue;
            }
            if (n == 0) break;
            if (errno == EINTR) continue;
            throw_errno("read " + p);
        }
        while (!out.empty() && (out.back() == '\n' || out.back() == '\r' || out.back() == ' ' || out.back() == '\t')) {
            out.pop_back();
        }
        return out;
    }

    void write_string(const std::string& rel, const std::string& value) const {
        std::string p = path(rel);
        Fd fd(::open(p.c_str(), O_WRONLY | O_CLOEXEC));
        if (!fd.valid()) throw_errno("open " + p);

        const char* ptr = value.data();
        size_t left = value.size();
        while (left > 0) {
            ssize_t n = ::write(fd.get(), ptr, left);
            if (n > 0) {
                ptr += n;
                left -= static_cast<size_t>(n);
                continue;
            }
            if (n < 0 && errno == EINTR) continue;
            throw_errno("write " + p + " = " + value);
        }
    }

    bool try_write_string(const std::string& rel, const std::string& value, std::string* err = nullptr) const {
        try {
            write_string(rel, value);
            return true;
        } catch (const std::exception& e) {
            if (err) *err = e.what();
            return false;
        }
    }

    void write_int(const std::string& rel, int64_t v) const {
        write_string(rel, std::to_string(v));
    }

    bool try_write_int(const std::string& rel, int64_t v, std::string* err = nullptr) const {
        return try_write_string(rel, std::to_string(v), err);
    }

    // Write the first existing/working candidate. Useful because some IIO attrs
    // are named per-type, e.g. in_accel_sampling_frequency.
    std::string write_one_of(const std::vector<std::string>& candidates,
                             const std::string& value,
                             bool required,
                             const std::string& label) const {
        std::string last_err;
        for (const auto& rel : candidates) {
            if (!exists(rel)) continue;
            if (try_write_string(rel, value, &last_err)) return rel;
        }
        // Some sysfs nodes are virtual; try even if stat did not see them.
        for (const auto& rel : candidates) {
            if (try_write_string(rel, value, &last_err)) return rel;
        }
        if (required) {
            throw std::runtime_error("failed to write " + label + "; last error: " + last_err);
        }
        if (!last_err.empty()) {
            std::cerr << "warning: failed to write optional " << label << ": " << last_err << "\n";
        } else {
            std::cerr << "warning: optional " << label << " not found\n";
        }
        return {};
    }

private:
    std::string root_;
};

struct ProgramOptions {
    std::string sysfs_root;
    std::string chardev;
    std::string trigger;
    std::string csv_path;  // empty means stdout
    std::string mode = "poll";

    int sampling_frequency = 1000;
    int fifo_watermark = 16;
    int buffer_length = 1024;
    int buffer_watermark = 16;
    int timeout_ms = 1000;
    int read_frames = 128;

    uint64_t max_frames = 0;       // 0 means unlimited
    double duration_sec = 0.0;     // 0 means unlimited
    double gap_factor = 1.5;

    bool configure = true;
    bool strict_optional_sysfs = false;
    bool print_summary = true;
};

// -----------------------------------------------------------------------------
// 3. IioConfigurator
// -----------------------------------------------------------------------------
class IioConfigurator {
public:
    explicit IioConfigurator(Sysfs sysfs, ProgramOptions opts)
        : sysfs_(std::move(sysfs)), opts_(std::move(opts)) {}

    const Sysfs& sysfs() const { return sysfs_; }

    void configure() {
        if (!opts_.configure) return;

        // Buffer must be disabled before changing scan elements, trigger, or
        // this driver's fifo_watermark because the driver returns -EBUSY while
        // the buffer is active.
        sysfs_.try_write_string("buffer/enable", "0");

        configure_scan_elements();
        configure_sampling_frequency();
        configure_watermarks();
        configure_buffer_length();
        configure_trigger();

        sysfs_.write_string("buffer/enable", "1");
        enabled_ = true;
    }

    void disable_buffer_noexcept() noexcept {
        if (!enabled_ && opts_.configure) {
            // Still try once; it is safe and helps after exceptions.
        }
        try {
            sysfs_.try_write_string("buffer/enable", "0");
            enabled_ = false;
        } catch (...) {}
    }

    ~IioConfigurator() { disable_buffer_noexcept(); }

private:
    static std::vector<std::vector<std::string>> scan_disable_candidates() {
        return {
            {"scan_elements/in_accel_x_en"},
            {"scan_elements/in_accel_y_en"},
            {"scan_elements/in_accel_z_en"},
            {"scan_elements/in_anglvel_x_en", "scan_elements/in_gyro_x_en"},
            {"scan_elements/in_anglvel_y_en", "scan_elements/in_gyro_y_en"},
            {"scan_elements/in_anglvel_z_en", "scan_elements/in_gyro_z_en"},
            {"scan_elements/in_timestamp_en"},
        };
    }

    void configure_scan_elements() {
        // First disable the elements that may exist, then enable exactly the
        // layout this tool parses.
        for (const auto& group : scan_disable_candidates()) {
            for (const auto& rel : group) {
                if (sysfs_.exists(rel)) sysfs_.try_write_string(rel, "0");
            }
        }

        enable_scan_one({"scan_elements/in_accel_x_en"}, "accel_x");
        enable_scan_one({"scan_elements/in_accel_y_en"}, "accel_y");
        enable_scan_one({"scan_elements/in_accel_z_en"}, "accel_z");
        enable_scan_one({"scan_elements/in_anglvel_x_en", "scan_elements/in_gyro_x_en"}, "gyro_x/anglvel_x");
        enable_scan_one({"scan_elements/in_anglvel_y_en", "scan_elements/in_gyro_y_en"}, "gyro_y/anglvel_y");
        enable_scan_one({"scan_elements/in_anglvel_z_en", "scan_elements/in_gyro_z_en"}, "gyro_z/anglvel_z");
        enable_scan_one({"scan_elements/in_timestamp_en"}, "timestamp");
    }

    void enable_scan_one(const std::vector<std::string>& candidates, const std::string& label) {
        sysfs_.write_one_of(candidates, "1", true, "scan element " + label);
    }

    void configure_sampling_frequency() {
        // Driver code usually exposes per-type shared attrs for IIO_ACCEL and
        // IIO_ANGL_VEL, but keep root sampling_frequency first because it was
        // requested explicitly.
        const std::string hz = std::to_string(opts_.sampling_frequency);
        sysfs_.write_one_of({
            "sampling_frequency",
            "in_accel_sampling_frequency",
            "in_anglvel_sampling_frequency",
            "in_gyro_sampling_frequency",
        }, hz, !opts_.strict_optional_sysfs, "sampling_frequency");

        // If per-type attrs exist, set both accel and gyro/anglvel to the same Hz.
        sysfs_.try_write_string("in_accel_sampling_frequency", hz);
        sysfs_.try_write_string("in_anglvel_sampling_frequency", hz);
        sysfs_.try_write_string("in_gyro_sampling_frequency", hz);
    }

    void configure_watermarks() {
        sysfs_.write_one_of({"fifo_watermark"}, std::to_string(opts_.fifo_watermark),
                            !opts_.strict_optional_sysfs, "fifo_watermark");
        sysfs_.write_one_of({"buffer/watermark"}, std::to_string(opts_.buffer_watermark),
                            !opts_.strict_optional_sysfs, "buffer/watermark");
    }

    void configure_buffer_length() {
        sysfs_.write_string("buffer/length", std::to_string(opts_.buffer_length));
    }

    void configure_trigger() {
        if (opts_.trigger.empty()) {
            if (sysfs_.exists("trigger/current_trigger")) {
                std::string cur = sysfs_.read_string("trigger/current_trigger");
                std::cerr << "warning: --trigger not set; current_trigger='" << cur << "'\n";
            }
            return;
        }
        sysfs_.write_string("trigger/current_trigger", opts_.trigger);
    }

    Sysfs sysfs_;
    ProgramOptions opts_;
    bool enabled_ = false;
};

// -----------------------------------------------------------------------------
// 5. SampleParser
// -----------------------------------------------------------------------------
struct Sample {
    uint64_t seq = 0;
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
    int64_t timestamp_ns = 0;
    int64_t dt_ns = 0;
    bool gap = false;
    uint64_t estimated_drops_here = 0;
};

class SampleParser {
public:
    static constexpr size_t kScanSize = 24;
    static constexpr size_t kTimestampOffset = 16;

    static Sample parse_frame(const uint8_t* p) {
        Sample s;
        s.ax = read_be_s16(p + 0);
        s.ay = read_be_s16(p + 2);
        s.az = read_be_s16(p + 4);
        s.gx = read_be_s16(p + 6);
        s.gy = read_be_s16(p + 8);
        s.gz = read_be_s16(p + 10);
        s.timestamp_ns = read_le_s64(p + kTimestampOffset);
        return s;
    }

private:
    static int16_t read_be_s16(const uint8_t* p) {
        uint16_t u = (static_cast<uint16_t>(p[0]) << 8) |
                     (static_cast<uint16_t>(p[1]));
        return static_cast<int16_t>(u);
    }

    static int64_t read_le_s64(const uint8_t* p) {
        uint64_t u = 0;
        for (int i = 7; i >= 0; --i) {
            u = (u << 8) | p[i];
        }
        return static_cast<int64_t>(u);
    }
};

// -----------------------------------------------------------------------------
// 7. CsvWriter
// -----------------------------------------------------------------------------
class CsvWriter {
public:
    explicit CsvWriter(const std::string& path) {
        if (path.empty() || path == "-") {
            fd_ = STDOUT_FILENO;
            owns_fd_ = false;
        } else {
            int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
            if (fd < 0) throw_errno("open csv " + path);
            fd_ = fd;
            owns_fd_ = true;
        }
        write_raw("seq,ax,ay,az,gx,gy,gz,timestamp_ns,dt_ns,gap\n");
    }

    ~CsvWriter() {
        if (owns_fd_ && fd_ >= 0) {
            while (::close(fd_) != 0 && errno == EINTR) {}
        }
    }

    CsvWriter(const CsvWriter&) = delete;
    CsvWriter& operator=(const CsvWriter&) = delete;

    void write_sample(const Sample& s) {
        std::ostringstream oss;
        oss << s.seq << ','
            << s.ax << ',' << s.ay << ',' << s.az << ','
            << s.gx << ',' << s.gy << ',' << s.gz << ','
            << s.timestamp_ns << ',' << s.dt_ns << ',' << (s.gap ? 1 : 0) << '\n';
        write_raw(oss.str());
    }

private:
    void write_raw(const std::string& line) {
        const char* ptr = line.data();
        size_t left = line.size();
        while (left > 0) {
            ssize_t n = ::write(fd_, ptr, left);
            if (n > 0) {
                ptr += n;
                left -= static_cast<size_t>(n);
                continue;
            }
            if (n < 0 && errno == EINTR) continue;
            throw_errno("write csv");
        }
    }

    int fd_ = -1;
    bool owns_fd_ = false;
};

// -----------------------------------------------------------------------------
// 6. StatsCollector
// -----------------------------------------------------------------------------
class StatsCollector {
public:
    explicit StatsCollector(double expected_hz, double gap_factor)
        : expected_period_ns_(expected_hz > 0.0 ? static_cast<int64_t>(std::llround(1e9 / expected_hz)) : 0),
          gap_factor_(gap_factor) {
        start_wall_ns_ = monotonic_ns();
        if (::getrusage(RUSAGE_SELF, &start_ru_) != 0) {
            throw_errno("getrusage start");
        }
    }

    void on_wakeup() { ++wakeups_; }
    void on_read_call() { ++read_calls_; }
    void on_eagain() { ++eagain_count_; }
    void on_bytes(size_t n) { bytes_read_ += n; }

    Sample complete_sample(Sample s) {
        s.seq = total_frames_;
        if (have_prev_ts_) {
            s.dt_ns = s.timestamp_ns - prev_timestamp_ns_;
            if (s.dt_ns > 0) {
                min_dt_ns_ = std::min(min_dt_ns_, s.dt_ns);
                max_dt_ns_ = std::max(max_dt_ns_, s.dt_ns);
                sum_dt_ns_ += static_cast<long double>(s.dt_ns);
                ++dt_count_;

                if (expected_period_ns_ > 0) {
                    const long double threshold = static_cast<long double>(expected_period_ns_) * gap_factor_;
                    if (static_cast<long double>(s.dt_ns) > threshold) {
                        s.gap = true;
                        ++timestamp_gap_count_;
                        int64_t est = static_cast<int64_t>(std::llround(
                            static_cast<long double>(s.dt_ns) / static_cast<long double>(expected_period_ns_))) - 1;
                        if (est > 0) {
                            s.estimated_drops_here = static_cast<uint64_t>(est);
                            estimated_drops_ += s.estimated_drops_here;
                        }
                    }
                }
            }
        } else {
            first_timestamp_ns_ = s.timestamp_ns;
            have_first_ts_ = true;
        }

        prev_timestamp_ns_ = s.timestamp_ns;
        have_prev_ts_ = true;
        ++total_frames_;
        return s;
    }

    uint64_t total_frames() const { return total_frames_; }

    void print_summary(std::ostream& os) const {
        int64_t end_wall_ns = monotonic_ns();
        struct rusage end_ru {};
        if (::getrusage(RUSAGE_SELF, &end_ru) != 0) {
            throw_errno("getrusage end");
        }

        const double wall_sec = std::max(1e-9, static_cast<double>(end_wall_ns - start_wall_ns_) / 1e9);
        const double actual_hz = static_cast<double>(total_frames_) / wall_sec;
        const double avg_read_size = read_calls_ ? static_cast<double>(bytes_read_) / static_cast<double>(read_calls_) : 0.0;

        const double user_cpu = timeval_to_sec(end_ru.ru_utime) - timeval_to_sec(start_ru_.ru_utime);
        const double sys_cpu = timeval_to_sec(end_ru.ru_stime) - timeval_to_sec(start_ru_.ru_stime);
        const double cpu_sec = std::max(0.0, user_cpu + sys_cpu);
        const double cpu_pct = 100.0 * cpu_sec / wall_sec;

        const double avg_dt = dt_count_ ? static_cast<double>(sum_dt_ns_ / static_cast<long double>(dt_count_)) : 0.0;
        const int64_t min_dt = dt_count_ ? min_dt_ns_ : 0;
        const int64_t max_dt = dt_count_ ? max_dt_ns_ : 0;

        os << std::fixed << std::setprecision(3);
        os << "\n=== IIO capture summary ===\n";
        os << "total frames: " << total_frames_ << "\n";
        os << "actual Hz: " << actual_hz << "\n";
        os << "read calls: " << read_calls_ << "\n";
        os << "poll/epoll wakeups: " << wakeups_ << "\n";
        os << "EAGAIN count: " << eagain_count_ << "\n";
        os << "bytes read: " << bytes_read_ << "\n";
        os << "avg read size: " << avg_read_size << " bytes\n";
        os << "timestamp delta ns min/max/avg: " << min_dt << " / " << max_dt << " / " << avg_dt << "\n";
        os << "timestamp gap count: " << timestamp_gap_count_ << "\n";
        os << "estimated drops: " << estimated_drops_ << "\n";
        os << "CPU usage: " << cpu_pct << "% (user=" << user_cpu << "s sys=" << sys_cpu << "s wall=" << wall_sec << "s)\n";
        if (have_first_ts_ && have_prev_ts_ && prev_timestamp_ns_ >= first_timestamp_ns_) {
            const double sensor_span_sec = static_cast<double>(prev_timestamp_ns_ - first_timestamp_ns_) / 1e9;
            const double sensor_hz = sensor_span_sec > 0.0 && total_frames_ > 1
                ? static_cast<double>(total_frames_ - 1) / sensor_span_sec
                : 0.0;
            os << "sensor timestamp Hz: " << sensor_hz << "\n";
        }
    }

private:
    int64_t start_wall_ns_ = 0;
    struct rusage start_ru_ {};

    uint64_t total_frames_ = 0;
    uint64_t read_calls_ = 0;
    uint64_t wakeups_ = 0;
    uint64_t eagain_count_ = 0;
    uint64_t bytes_read_ = 0;

    bool have_first_ts_ = false;
    bool have_prev_ts_ = false;
    int64_t first_timestamp_ns_ = 0;
    int64_t prev_timestamp_ns_ = 0;

    int64_t min_dt_ns_ = std::numeric_limits<int64_t>::max();
    int64_t max_dt_ns_ = std::numeric_limits<int64_t>::min();
    long double sum_dt_ns_ = 0.0;
    uint64_t dt_count_ = 0;

    int64_t expected_period_ns_ = 0;
    double gap_factor_ = 1.5;
    uint64_t timestamp_gap_count_ = 0;
    uint64_t estimated_drops_ = 0;
};

// -----------------------------------------------------------------------------
// 4. IioReader
// -----------------------------------------------------------------------------
class IioReader {
public:
    IioReader(std::string chardev, const ProgramOptions& opts)
        : chardev_(std::move(chardev)), opts_(opts) {
        // Requirement: open /dev/iio:deviceX with O_RDONLY | O_NONBLOCK.
        int fd = ::open(chardev_.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) throw_errno("open " + chardev_);
        fd_.reset(fd);

        // Also enforce O_NONBLOCK via fcntl, useful if the fd came from a wrapper
        // in future changes.
        int flags = ::fcntl(fd_.get(), F_GETFL, 0);
        if (flags < 0) throw_errno("fcntl(F_GETFL)");
        if ((flags & O_NONBLOCK) == 0) {
            if (::fcntl(fd_.get(), F_SETFL, flags | O_NONBLOCK) != 0) {
                throw_errno("fcntl(F_SETFL O_NONBLOCK)");
            }
        }

        if (opts_.read_frames <= 0) {
            throw std::runtime_error("--read-frames must be > 0");
        }
        read_buf_.resize(static_cast<size_t>(opts_.read_frames) * SampleParser::kScanSize);
        staging_.reserve(read_buf_.size() + SampleParser::kScanSize);
    }

    void run(StatsCollector& stats, CsvWriter& csv) {
        const int64_t run_start_ns = monotonic_ns();
        if (opts_.mode == "poll") {
            run_poll(stats, csv, run_start_ns);
        } else if (opts_.mode == "epoll") {
            run_epoll(stats, csv, run_start_ns);
        } else {
            throw std::runtime_error("unknown --mode '" + opts_.mode + "', expected poll or epoll");
        }
    }

private:
    bool should_stop(const StatsCollector& stats, int64_t run_start_ns) const {
        if (g_stop) return true;
        if (opts_.max_frames && stats.total_frames() >= opts_.max_frames) return true;
        if (opts_.duration_sec > 0.0) {
            const double elapsed = static_cast<double>(monotonic_ns() - run_start_ns) / 1e9;
            if (elapsed >= opts_.duration_sec) return true;
        }
        return false;
    }

    void run_poll(StatsCollector& stats, CsvWriter& csv, int64_t run_start_ns) {
        struct pollfd pfd {};
        pfd.fd = fd_.get();
        pfd.events = POLLIN | POLLERR | POLLHUP;

        while (!should_stop(stats, run_start_ns)) {
            int r = ::poll(&pfd, 1, opts_.timeout_ms);
            if (r < 0) {
                if (errno == EINTR) continue;
                throw_errno("poll");
            }
            if (r == 0) {
                // Nonblocking IIO read can still retrieve samples below watermark
                // after a timeout, providing bounded latency.
                drain_until_eagain(stats, csv, run_start_ns);
                continue;
            }

            stats.on_wakeup();
            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                std::cerr << "warning: poll revents=0x" << std::hex << pfd.revents << std::dec << "\n";
            }
            if (pfd.revents & (POLLIN | POLLERR | POLLHUP)) {
                drain_until_eagain(stats, csv, run_start_ns);
            }
        }
    }

    void run_epoll(StatsCollector& stats, CsvWriter& csv, int64_t run_start_ns) {
        Fd ep(::epoll_create1(EPOLL_CLOEXEC));
        if (!ep.valid()) throw_errno("epoll_create1");

        struct epoll_event ev {};
        ev.events = EPOLLIN | EPOLLERR | EPOLLHUP | EPOLLET;
        ev.data.fd = fd_.get();
        if (::epoll_ctl(ep.get(), EPOLL_CTL_ADD, fd_.get(), &ev) != 0) {
            throw_errno("epoll_ctl ADD");
        }

        std::vector<struct epoll_event> events(4);
        while (!should_stop(stats, run_start_ns)) {
            int r = ::epoll_wait(ep.get(), events.data(), static_cast<int>(events.size()), opts_.timeout_ms);
            if (r < 0) {
                if (errno == EINTR) continue;
                throw_errno("epoll_wait");
            }
            if (r == 0) {
                // Same bounded-latency timeout drain as poll mode.
                drain_until_eagain(stats, csv, run_start_ns);
                continue;
            }
            for (int i = 0; i < r; ++i) {
                stats.on_wakeup();
                if (events[static_cast<size_t>(i)].events & (EPOLLERR | EPOLLHUP)) {
                    std::cerr << "warning: epoll events=0x" << std::hex
                              << events[static_cast<size_t>(i)].events << std::dec << "\n";
                }
                // EPOLLET rule: after receiving an event, keep reading until
                // nonblocking read returns EAGAIN/EWOULDBLOCK.
                drain_until_eagain(stats, csv, run_start_ns);
                if (should_stop(stats, run_start_ns)) break;
            }
        }
    }

    void drain_until_eagain(StatsCollector& stats, CsvWriter& csv, int64_t run_start_ns) {
        while (!should_stop(stats, run_start_ns)) {
            stats.on_read_call();
            ssize_t n = ::read(fd_.get(), read_buf_.data(), read_buf_.size());
            if (n > 0) {
                stats.on_bytes(static_cast<size_t>(n));
                append_and_process(read_buf_.data(), static_cast<size_t>(n), stats, csv, run_start_ns);
                continue;
            }
            if (n == 0) {
                // No data at this instant. For an IIO chardev this is unusual;
                // avoid a busy loop and go back to poll/epoll.
                break;
            }
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                stats.on_eagain();
                break;
            }
            throw_errno("read " + chardev_);
        }
    }

    void append_and_process(const uint8_t* data, size_t n,
                            StatsCollector& stats, CsvWriter& csv,
                            int64_t run_start_ns) {
        // Keep a small staging buffer so a defensive partial read does not break
        // the fixed 24-byte scan framing. In normal IIO buffer reads, n should
        // already be a multiple of the scan size.
        const size_t old_size = staging_.size();
        staging_.resize(old_size + n);
        std::memcpy(staging_.data() + old_size, data, n);

        size_t consumed = 0;
        const size_t full = staging_.size() / SampleParser::kScanSize * SampleParser::kScanSize;
        while (consumed < full) {
            Sample s = SampleParser::parse_frame(staging_.data() + consumed);
            s = stats.complete_sample(s);
            csv.write_sample(s);
            consumed += SampleParser::kScanSize;
            if (should_stop(stats, run_start_ns)) break;
        }

        if (consumed > 0) {
            staging_.erase(staging_.begin(), staging_.begin() + static_cast<std::ptrdiff_t>(consumed));
        }
    }

    std::string chardev_;
    ProgramOptions opts_;
    Fd fd_;
    std::vector<uint8_t> read_buf_;
    std::vector<uint8_t> staging_;
};

// -----------------------------------------------------------------------------
// 8. main argument parsing
// -----------------------------------------------------------------------------
void usage(const char* argv0) {
    std::cerr
        << "Usage:\n"
        << "  " << argv0 << " --iio /sys/bus/iio/devices/iio:deviceX [options]\n\n"
        << "Options:\n"
        << "  --iio PATH                 sysfs IIO device path, e.g. /sys/bus/iio/devices/iio:device0\n"
        << "  --dev PATH                 char device path; default derived as /dev/<basename(--iio)>\n"
        << "  --trigger NAME             write trigger/current_trigger to NAME\n"
        << "  --mode poll|epoll          default: poll; epoll uses EPOLLET and drains to EAGAIN\n"
        << "  --sampling-frequency HZ    default: 1000\n"
        << "  --fifo-watermark N         driver fifo_watermark; default: 16\n"
        << "  --buffer-length N          default: 1024 scans\n"
        << "  --buffer-watermark N       default: 16 scans\n"
        << "  --read-frames N            read buffer size in scans; default: 128\n"
        << "  --timeout-ms N             poll/epoll timeout; default: 1000\n"
        << "  --count N                  stop after N frames; default: unlimited\n"
        << "  --duration-sec S           stop after S seconds; default: unlimited\n"
        << "  --gap-factor F             gap when dt > F * expected_period; default: 1.5\n"
        << "  --csv PATH|-               CSV output file; default: stdout\n"
        << "  --no-config                skip sysfs configuration and only read chardev\n"
        << "  --strict-optional-sysfs    make sampling_frequency/fifo_watermark/buffer/watermark failures fatal\n"
        << "  --no-summary               do not print stderr summary\n"
        << "  -h, --help                 show this help\n";
}

std::string need_value(int& i, int argc, char** argv) {
    if (i + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + argv[i]);
    }
    return argv[++i];
}

int parse_int_arg(const std::string& s, const std::string& name) {
    char* end = nullptr;
    errno = 0;
    long v = std::strtol(s.c_str(), &end, 10);
    if (errno || !end || *end != '\0' || v < std::numeric_limits<int>::min() || v > std::numeric_limits<int>::max()) {
        throw std::runtime_error("invalid integer for " + name + ": " + s);
    }
    return static_cast<int>(v);
}

uint64_t parse_u64_arg(const std::string& s, const std::string& name) {
    char* end = nullptr;
    errno = 0;
    unsigned long long v = std::strtoull(s.c_str(), &end, 10);
    if (errno || !end || *end != '\0') {
        throw std::runtime_error("invalid integer for " + name + ": " + s);
    }
    return static_cast<uint64_t>(v);
}

double parse_double_arg(const std::string& s, const std::string& name) {
    char* end = nullptr;
    errno = 0;
    double v = std::strtod(s.c_str(), &end);
    if (errno || !end || *end != '\0' || !std::isfinite(v)) {
        throw std::runtime_error("invalid number for " + name + ": " + s);
    }
    return v;
}

ProgramOptions parse_args(int argc, char** argv) {
    ProgramOptions opts;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "-h" || a == "--help") {
            usage(argv[0]);
            std::exit(0);
        } else if (a == "--iio") {
            opts.sysfs_root = need_value(i, argc, argv);
        } else if (a == "--dev") {
            opts.chardev = need_value(i, argc, argv);
        } else if (a == "--trigger") {
            opts.trigger = need_value(i, argc, argv);
        } else if (a == "--mode") {
            opts.mode = need_value(i, argc, argv);
        } else if (a == "--sampling-frequency") {
            opts.sampling_frequency = parse_int_arg(need_value(i, argc, argv), a);
        } else if (a == "--fifo-watermark") {
            opts.fifo_watermark = parse_int_arg(need_value(i, argc, argv), a);
        } else if (a == "--buffer-length") {
            opts.buffer_length = parse_int_arg(need_value(i, argc, argv), a);
        } else if (a == "--buffer-watermark") {
            opts.buffer_watermark = parse_int_arg(need_value(i, argc, argv), a);
        } else if (a == "--read-frames") {
            opts.read_frames = parse_int_arg(need_value(i, argc, argv), a);
        } else if (a == "--timeout-ms") {
            opts.timeout_ms = parse_int_arg(need_value(i, argc, argv), a);
        } else if (a == "--count") {
            opts.max_frames = parse_u64_arg(need_value(i, argc, argv), a);
        } else if (a == "--duration-sec") {
            opts.duration_sec = parse_double_arg(need_value(i, argc, argv), a);
        } else if (a == "--gap-factor") {
            opts.gap_factor = parse_double_arg(need_value(i, argc, argv), a);
        } else if (a == "--csv") {
            opts.csv_path = need_value(i, argc, argv);
        } else if (a == "--no-config") {
            opts.configure = false;
        } else if (a == "--strict-optional-sysfs") {
            opts.strict_optional_sysfs = true;
        } else if (a == "--no-summary") {
            opts.print_summary = false;
        } else {
            throw std::runtime_error("unknown argument: " + a);
        }
    }

    if (opts.sysfs_root.empty()) {
        throw std::runtime_error("--iio is required");
    }
    if (opts.chardev.empty()) {
        opts.chardev = "/dev/" + basename_of(opts.sysfs_root);
    }
    if (opts.mode != "poll" && opts.mode != "epoll") {
        throw std::runtime_error("--mode must be poll or epoll");
    }
    if (opts.sampling_frequency <= 0) throw std::runtime_error("--sampling-frequency must be > 0");
    if (opts.fifo_watermark <= 0) throw std::runtime_error("--fifo-watermark must be > 0");
    if (opts.buffer_length <= 0) throw std::runtime_error("--buffer-length must be > 0");
    if (opts.buffer_watermark <= 0) throw std::runtime_error("--buffer-watermark must be > 0");
    if (opts.read_frames <= 0) throw std::runtime_error("--read-frames must be > 0");
    if (opts.timeout_ms < 0) throw std::runtime_error("--timeout-ms must be >= 0");
    if (opts.gap_factor <= 0.0) throw std::runtime_error("--gap-factor must be > 0");

    return opts;
}

} // namespace

int main(int argc, char** argv) {
    try {
        ::signal(SIGINT, on_signal);
        ::signal(SIGTERM, on_signal);

        ProgramOptions opts = parse_args(argc, argv);

        IioConfigurator configurator(Sysfs(opts.sysfs_root), opts);
        configurator.configure();

        CsvWriter csv(opts.csv_path);
        StatsCollector stats(static_cast<double>(opts.sampling_frequency), opts.gap_factor);
        IioReader reader(opts.chardev, opts);
        reader.run(stats, csv);

        configurator.disable_buffer_noexcept();

        if (opts.print_summary) {
            stats.print_summary(std::cerr);
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
