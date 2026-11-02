// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Bounded real-input experiment, not an OpenXR pose provider. See
// imu-input-contract.json and start-contract.json for the recovered wire ABI.
#include "qvr_bridge.hpp"
#include "a081_pose_output.hpp"
#include <qiyu/device/a081_camera_capture.hpp>
#include <qiyu/device/a081_camera_groups.hpp>
#include "a081_tracking_server.hpp"
#include <qiyu/device/a081_controller_frame_export.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <linux/dma-buf.h>
#include <misc/fastrpc.h>
#include <poll.h>
#include <spawn.h>
#include <sstream>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include <vector>

extern char **environ;
extern "C" void a081_apps_mem_counters(std::uint64_t out[4]) __attribute__((weak));
extern "C" void a081_apps_mem_lifetimes(std::uint64_t out[6]) __attribute__((weak));

namespace a081::stock::imu_live {
inline void report(const char *stage, Result r, const std::string &detail = {})
{
    std::lock_guard<std::mutex> log(camera::output_mutex);
    std::cout << "stage=" << stage << " rc=" << r.rc << " errno=" << r.system_errno
              << " message=" << std::quoted(describe(r));
    if (!detail.empty()) std::cout << " detail=" << std::quoted(detail);
    std::cout << std::endl;
}
inline Result failure(int error) { return {-1, error}; }
inline Result sysresult(int rc) { return {rc, rc == -1 ? errno : 0}; }

inline bool text_file(const std::string &path, std::string &out)
{
    std::ifstream file(path, std::ios::binary);
    if (!file) return false;
    std::ostringstream contents;
    contents << file.rdbuf();
    if (file.bad()) return false;
    out = contents.str();
    return true;
}
inline bool attribute(const std::string &path, std::string &out)
{
    if (!text_file(path, out)) return false;
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) out.pop_back();
    return true;
}
inline Result write_attribute(const std::string &path, const std::string &value)
{
    const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0) return failure(errno);
    const auto n = ::write(fd, value.data(), value.size());
    Result result = n == static_cast<ssize_t>(value.size()) ? Result{} : failure(n < 0 ? errno : EIO);
    if (::close(fd) != 0 && result) result = failure(errno);
    return result;
}
inline std::uint64_t boottime_ns()
{
    timespec now{};
    if (clock_gettime(CLOCK_BOOTTIME, &now) != 0) return 0;
    return static_cast<std::uint64_t>(now.tv_sec) * 1000000000ULL + now.tv_nsec;
}

// Use the installed coreutils program through argv, never through a shell.
// Verify the raw files, then apply only the proven config6 LF normalization.
inline bool verified_xml(const char *variable, const char *expected, std::string &xml)
{
    const char *path = std::getenv(variable);
    if (!path || path[0] != '/') {
        report("xml_path", failure(EINVAL), std::string(variable) + " must name an absolute stock XML path");
        return false;
    }
    int descriptors[2];
    if (pipe2(descriptors, O_CLOEXEC) != 0) {
        report("xml_hash_pipe", failure(errno));
        return false;
    }
    posix_spawn_file_actions_t actions;
    int rc = posix_spawn_file_actions_init(&actions);
    const bool actions_initialized = rc == 0;
    if (rc == 0) {
        rc = posix_spawn_file_actions_adddup2(&actions, descriptors[1], STDOUT_FILENO);
        if (rc == 0) rc = posix_spawn_file_actions_addclose(&actions, descriptors[0]);
        if (rc == 0) rc = posix_spawn_file_actions_addclose(&actions, descriptors[1]);
    }
    char command[] = "sha256sum", separator[] = "--";
    char *arguments[] = {command, separator, const_cast<char *>(path), nullptr};
    pid_t child = -1;
    if (rc == 0) rc = posix_spawnp(&child, command, &actions, nullptr, arguments, environ);
    if (actions_initialized) posix_spawn_file_actions_destroy(&actions);
    ::close(descriptors[1]);
    std::string output;
    char buffer[512];
    ssize_t n;
    while ((n = ::read(descriptors[0], buffer, sizeof(buffer))) > 0)
        output.append(buffer, static_cast<std::size_t>(n));
    ::close(descriptors[0]);
    int status = 0;
    pid_t waited = -1;
    if (child > 0) {
        do { waited = waitpid(child, &status, 0); } while (waited < 0 && errno == EINTR);
    }
    const bool matched = rc == 0 && waited == child && WIFEXITED(status) && WEXITSTATUS(status) == 0 &&
                         output.size() >= 64 && output.compare(0, 64, expected) == 0;
    report("xml_hash", matched ? Result{} : failure(rc ? rc : EINVAL),
           std::string(variable) + " path=" + path + " expected=" + expected +
           " actual=" + output.substr(0, std::min<std::size_t>(64, output.size())));
    if (!matched || !text_file(path, xml) || xml.empty() || xml.find('\0') != std::string::npos)
        return false;
    return true;
}

inline bool axis_mapping(std::array<int, 3> &axes)
{
    const char *value = std::getenv("A081_QVR_IIO_TO_ANDROID");
    if (!value) {
        report("iio_axis_gate", failure(EINVAL),
               "A081_QVR_IIO_TO_ANDROID is required: verified signed source-axis permutation, e.g. 1,2,3 only after confirming Android-portrait equivalence");
        return false;
    }
    std::istringstream input(value);
    char comma1 = 0, comma2 = 0, trailing = 0;
    if (!(input >> axes[0] >> comma1 >> axes[1] >> comma2 >> axes[2]) ||
        comma1 != ',' || comma2 != ',' || (input >> trailing)) return false;
    for (unsigned i = 0; i < 3; ++i) {
        if (axes[i] < -3 || axes[i] > 3 || axes[i] == 0) return false;
        for (unsigned j = 0; j < i; ++j)
            if (std::abs(axes[i]) == std::abs(axes[j])) return false;
    }
    report("iio_axis_gate", {}, std::string("explicit IIO-to-Android-portrait permutation=") + value);
    return true;
}

inline volatile std::sig_atomic_t interrupted = 0;
inline void request_stop(int) { interrupted = 1; }
class Signals {
public:
    bool install()
    {
        interrupted = 0;
        struct sigaction action{};
        action.sa_handler = request_stop;
        // Preserve the daemon listener's blocking syscalls when a termination
        // request goes to that thread; the client loop observes the flag.
        action.sa_flags = SA_RESTART;
        sigemptyset(&action.sa_mask);
        if (sigaction(SIGINT, &action, &old_int_) != 0) return false;
        int_set_ = true;
        if (sigaction(SIGTERM, &action, &old_term_) != 0) return false;
        term_set_ = true;
        return true;
    }
    ~Signals()
    {
        if (term_set_) sigaction(SIGTERM, &old_term_, nullptr);
        if (int_set_) sigaction(SIGINT, &old_int_, nullptr);
    }
private:
    struct sigaction old_int_{}, old_term_{};
    bool int_set_{}, term_set_{};
};

class IioLocks {
public:
    bool acquire()
    {
        std::vector<std::string> paths{"/run/a081-iio.lock"};
        if (const char *runtime = std::getenv("XDG_RUNTIME_DIR")) {
            const std::string path = std::string(runtime) + "/a081-iio.lock";
            if (path != paths.front()) paths.push_back(path);
        }
        for (const auto &path : paths) {
            const int fd = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
            if (fd < 0) { report("iio_lock", failure(errno), path); return false; }
            fds_.push_back(fd);
            if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
                report("iio_lock", failure(errno), path);
                return false;
            }
        }
        return true;
    }
    ~IioLocks() { for (int fd : fds_) ::close(fd); }
private:
    std::vector<int> fds_;
};

struct Sample {
    std::uint64_t time{};
    std::array<float, 3> value{};
    bool gyro{};
};
struct Scan {
    std::int32_t xyz[3];
    std::int32_t padding;
    std::int64_t time;
};
static_assert(sizeof(Scan) == 24 && offsetof(Scan, time) == 16);

class IioStream {
public:
    IioStream(const char *name, const char *channel, bool gyro)
        : name_(name), channel_(channel), gyro_(gyro) {}
    ~IioStream() { restore(); }
    bool discover()
    {
        DIR *dir = opendir("/sys/bus/iio/devices");
        if (!dir) { report("iio_discover", failure(errno), name_); return false; }
        std::string node;
        unsigned matches = 0;
        while (auto *entry = readdir(dir)) {
            if (std::strncmp(entry->d_name, "iio:device", 10)) continue;
            std::string actual;
            const std::string candidate = std::string("/sys/bus/iio/devices/") + entry->d_name;
            if (attribute(candidate + "/name", actual) && actual == name_) {
                node = entry->d_name;
                ++matches;
            }
        }
        closedir(dir);
        if (matches != 1) { report("iio_discover", failure(ENODEV), name_); return false; }
        path_ = "/sys/bus/iio/devices/" + node;
        node_ = "/dev/" + node;
        std::string value;
        if (!expect("buffer/enable", "0")) return false;
        for (unsigned i = 0; i < 4; ++i) {
            stems_[i] = i == 3 ? "in_timestamp" : "in_" + channel_ + "_" + "xyz"[i];
            if (!expect("scan_elements/" + stems_[i] + "_index", std::to_string(i)) ||
                !expect("scan_elements/" + stems_[i] + "_type", i == 3 ? "le:s64/64>>0" : "le:s32/32>>0"))
                return false;
        }
        // Extra enabled channels would invalidate the confirmed 24-byte scan.
        dir = opendir((path_ + "/scan_elements").c_str());
        if (!dir) return false;
        bool shape = true;
        while (auto *entry = readdir(dir)) {
            const std::string attr = entry->d_name;
            if (attr.size() < 3 || attr.substr(attr.size() - 3) != "_en") continue;
            bool known = false;
            for (const auto &stem : stems_) known = known || attr == stem + "_en";
            if (!known && (!attribute(path_ + "/scan_elements/" + attr, value) || value != "0")) shape = false;
        }
        closedir(dir);
        if (!shape || !attribute(path_ + "/in_" + channel_ + "_scale", value)) return false;
        char *end = nullptr;
        errno = 0;
        scale_ = std::strtod(value.c_str(), &end);
        if (errno || end != value.c_str() + value.size() || !std::isfinite(scale_) ||
            std::abs(scale_ - 0.000001) > 1e-15) {
            report("iio_scale", failure(EINVAL), path_ + " scale=" + value);
            return false;
        }
        if (!attribute(path_ + "/in_" + channel_ + "_sampling_frequency", rate_)) return false;
        report("iio_discover", {}, name_ + " node=" + node_ + " scale=" + value +
               " previous_sampling_frequency=" + rate_);
        return true;
    }
    bool configure(unsigned requested_rate)
    {
        if (!expect("buffer/enable", "0")) return false;
        std::string available;
        if (!attribute(path_ + "/in_" + channel_ + "_sampling_frequency_available", available)) return false;
        std::istringstream values(available);
        double value = 0;
        bool supported = false;
        while (values >> value) supported = supported || value == requested_rate;
        if (!supported) {
            report("iio_rate", failure(ENOTSUP), name_ + " requested=" + std::to_string(requested_rate));
            return false;
        }
        owned_ = true;
        if (!change("in_" + channel_ + "_sampling_frequency", std::to_string(requested_rate) + ".000000")) return false;
        for (const auto &stem : stems_)
            if (!change("scan_elements/" + stem + "_en", "1")) return false;
        if (!change("current_timestamp_clock", "boottime") ||
            !change("buffer/length", "256") || !change("buffer/watermark", "1")) return false;
        if (access((path_ + "/buffer/hwfifo_timeout").c_str(), F_OK) == 0 &&
            !change("buffer/hwfifo_timeout", "0.000000")) return false;
        fd_ = ::open(node_.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) { report("iio_open", failure(errno), node_); return false; }
        return true;
    }
    bool enable() { return change("buffer/enable", "1"); }
    bool disable()
    {
        if (!owned_) return true;
        const auto result = write_attribute(path_ + "/buffer/enable", "0");
        report("iio_disable", result, name_);
        return static_cast<bool>(result);
    }
    bool restore()
    {
        if (!owned_) return true;
        bool ok = disable();
        if (fd_ >= 0) { ok = (::close(fd_) == 0) && ok; fd_ = -1; }
        for (auto item = saved_.rbegin(); item != saved_.rend(); ++item) {
            const auto result = write_attribute(item->first, item->second);
            std::string actual;
            const bool restored = result && attribute(item->first, actual) && actual == item->second;
            report("iio_restore", restored ? Result{} : (result ? failure(EIO) : result),
                   "path=" + item->first + " value=" + item->second);
            ok = restored && ok;
        }
        saved_.clear();
        owned_ = false;
        return ok;
    }
    bool read_batch(std::vector<Sample> &batch, const std::array<int, 3> &axes)
    {
        for (unsigned i = 0; i < 64; ++i) {
            Scan raw{};
            const auto n = ::read(fd_, &raw, sizeof(raw));
            if (n < 0 && (errno == EAGAIN || errno == EINTR)) return true;
            if (n != sizeof(raw)) {
                report("iio_read", failure(n < 0 ? errno : EIO), name_);
                return false;
            }
            ++samples;
            if (raw.time <= 0 || static_cast<std::uint64_t>(raw.time) <= last_time_) {
                ++discarded;
                continue;
            }
            const auto now = boottime_ns();
            const auto time = static_cast<std::uint64_t>(raw.time);
            // Preserve the actual scan timestamp. Reject a clock mismatch;
            // old queued samples are discarded, never retimestamped.
            if (!now || time > now + 1000000ULL) {
                report("iio_timestamp", failure(EPROTO), name_ + " sample=" + std::to_string(time) + " boottime=" + std::to_string(now));
                return false;
            }
            last_time_ = time;
            if (now > time && now - time > 1000000000ULL) { ++discarded; continue; }
            Sample sample{time, {}, gyro_};
            for (unsigned axis = 0; axis < 3; ++axis) {
                const int source = axes[axis];
                const double sign = source < 0 ? -1.0 : 1.0;
                sample.value[axis] = static_cast<float>(raw.xyz[std::abs(source) - 1] * scale_ * sign);
            }
            batch.push_back(sample);
        }
        return true;
    }
    int fd() const { return fd_; }
    void report_rate() const
    {
        std::string requested, reported;
        const std::string stem = path_ + "/in_" + channel_ + "_sampling_frequency";
        if (attribute(stem, requested) && attribute(stem + "_reported", reported))
            report("iio_rate", {}, name_ + " requested=" + requested + " firmware_reported=" + reported);
        else report("iio_rate", failure(EIO), name_);
    }
    std::uint64_t samples{}, discarded{};
private:
    bool expect(const std::string &attr, const std::string &expected)
    {
        std::string value;
        if (attribute(path_ + "/" + attr, value) && value == expected) return true;
        report("iio_attribute_gate", failure(EINVAL), path_ + "/" + attr + " expected=" + expected + " actual=" + value);
        return false;
    }
    bool change(const std::string &attr, const std::string &value)
    {
        const std::string path = path_ + "/" + attr;
        std::string old;
        if (!attribute(path, old)) { report("iio_save", failure(EIO), path); return false; }
        // Log the journal before mutation, including unchanged values. This
        // also provides recovery values if an external SIGKILL prevents RAII.
        report("iio_save", {}, "path=" + path + " value=" + old);
        saved_.emplace_back(path, old);
        const auto result = write_attribute(path, value);
        report("iio_configure", result, "path=" + path + " value=" + value);
        return result && expect(attr, value);
    }
    std::string name_, channel_, path_, node_, rate_;
    std::array<std::string, 4> stems_;
    bool gyro_, owned_{};
    int fd_ = -1;
    double scale_{};
    std::uint64_t last_time_{};
    std::vector<std::pair<std::string, std::string>> saved_;
};

struct RingSpec {
    head::RingId id;
    std::uint32_t bytes, index, offset, element, count, type;
};
// AP descriptor rows in service-layout-constants.json; allocation sizes from
// stock ELF block table base0x136004 + block_id*28. Frame block21 is16384,
// debug block3 is10485760, prediction block12 is4096, DL block39 is12288.
inline constexpr std::array<RingSpec, 7> ring_specs{{
    {head::RingId::health, 8192, 6816, 0, 6816, 1, 7},
    {head::RingId::raw_gyro, 4096, 512, 0, 64, 8, 2},
    {head::RingId::pose, 16384, 0, 64, 192, 80, 0},
    {head::RingId::frame_pose, 16384, 0, 64, 64, 80, 10},
    {head::RingId::debug_6dof, 10485760, 4096, 0, 4096, 1, 4},
    {head::RingId::predicted_pose, 4096, 0, 64, 192, 8, 8},
    {head::RingId::dl_prediction, 12288, 0, 64, 128, 80, 15},
}};

class Ring {
public:
    Ring() = default;
    Ring(const Ring &) = delete;
    Ring &operator=(const Ring &) = delete;
    ~Ring() { if (!retained_) release(); }
    bool allocate(int session, RingSpec spec)
    {
        spec_ = spec;
        fastrpc_alloc_dma_buf allocation{};
        allocation.fd = -1;
        allocation.size = spec.bytes;
        errno = 0;
        auto result = sysresult(ioctl(session, FASTRPC_IOCTL_ALLOC_DMA_BUFF, &allocation));
        report("ring_alloc", result, detail() + " allocated_fd=" + std::to_string(allocation.fd));
        if (!result) return false;
        fd_ = allocation.fd;
        host_ = mmap(nullptr, spec.bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        result = host_ == MAP_FAILED ? failure(errno) : Result{};
        report("ring_mmap", result, detail());
        if (!result || !sync(DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE)) return false;
        std::memset(host_, 0, spec.bytes);
        std::atomic_thread_fence(std::memory_order_release);
        return sync(DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);
    }
    head::RingDescriptor descriptor() const
    {
        return {fd_, spec_.bytes, spec_.index, spec_.offset, spec_.element, spec_.count, spec_.type, 0};
    }
    std::string detail() const
    {
        return "ring=" + std::to_string(static_cast<unsigned>(spec_.id)) + " fd=" + std::to_string(fd_) +
               " bytes=" + std::to_string(spec_.bytes) + " index=" + std::to_string(spec_.index) +
               " offset=" + std::to_string(spec_.offset) + " element=" + std::to_string(spec_.element) +
               " count=" + std::to_string(spec_.count) + " type=" + std::to_string(spec_.type);
    }
    bool publish(unsigned slot, const Sample &gyro, const Sample &accel)
    {
        if (spec_.id != head::RingId::raw_gyro || slot >= 8 || !sync(DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE)) return false;
        head::ExternalSensorRecord64 record{};
        record.gyro_time_ns = gyro.time;
        record.accel_time_ns = accel.time;
        std::copy(gyro.value.begin(), gyro.value.end(), record.gyro);
        std::copy(accel.value.begin(), accel.value.end(), record.accel);
        auto *base = static_cast<std::uint8_t *>(host_);
        std::memcpy(base + slot * 64, &record, sizeof(record));
        // These allocations are FastRPC dma_alloc_coherent/dma_mmap_coherent,
        // not arbitrary imported cached buffers. The DMA-buf CPU-access pair
        // plus release ordering precedes the DSP's explicit cache invalidates.
        std::atomic_thread_fence(std::memory_order_release);
        const std::uint32_t index = slot;
        std::memcpy(base + spec_.index, &index, sizeof(index));
        std::atomic_thread_fence(std::memory_order_release);
        return sync(DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);
    }
    bool summary(const char *when)
    {
        if (fd_ < 0 || host_ == MAP_FAILED) return true;
        if (!sync(DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ)) return false;
        std::atomic_thread_fence(std::memory_order_acquire);
        const auto *base = static_cast<const std::uint8_t *>(host_);
        const std::size_t length = static_cast<std::size_t>(spec_.element) * spec_.count;
        std::vector<std::uint8_t> snapshot(length);
        std::memcpy(snapshot.data(), base + spec_.offset, length);
        std::uint32_t header = 0;
        if (spec_.index + sizeof(header) <= spec_.bytes) std::memcpy(&header, base + spec_.index, sizeof(header));
        if (!sync(DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ)) return false;
        std::uint64_t hash = 14695981039346656037ULL, maximum32 = 0;
        std::size_t nonzero = 0;
        for (auto byte : snapshot) { hash = (hash ^ byte) * 1099511628211ULL; nonzero += byte != 0; }
        // SDK offset32 is a timestamp candidate, not a verified live pose ABI.
        // Report the raw field only; no position/orientation validity flags.
        if (spec_.element == 192)
            for (unsigned slot = 0; slot < spec_.count; ++slot) {
                std::uint64_t value = 0;
                std::memcpy(&value, snapshot.data() + slot * spec_.element + 32, sizeof(value));
                maximum32 = std::max(maximum32, value);
            }
        if (!have_baseline_) { baseline_ = hash; have_baseline_ = true; }
        std::cout << "stage=ring_raw_summary when=" << when << " " << detail()
                  << " raw_header_u32=" << header << " fnv1a64=" << hash
                  << " nonzero_bytes=" << nonzero << " changed_from_initial=" << (hash != baseline_)
                  << " raw_u64_at32_max=" << maximum32
                  << " validity=uninterpreted" << std::endl;
        return true;
    }
    bool save_stopped_snapshot()
    {
        if (spec_.id != head::RingId::pose && spec_.id != head::RingId::dl_prediction) return true;
        const char *directory = std::getenv("A081_QVR_OUTPUT_DIR");
        if (!directory || directory[0] != '/') {
            report("ring_snapshot", failure(EINVAL), "wrapper output directory required");
            return false;
        }
        if (!sync(DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ)) return false;
        std::atomic_thread_fence(std::memory_order_acquire);
        std::vector<std::uint8_t> snapshot(spec_.bytes), confirm(spec_.bytes);
        std::memcpy(snapshot.data(), host_, snapshot.size());
        std::atomic_thread_fence(std::memory_order_acquire);
        std::memcpy(confirm.data(), host_, confirm.size());
        if (!sync(DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ)) return false;
        // The original ring has only a wrapping slot index, not a seqlock.
        // This is after successful stop and with no new IMU notifications;
        // byte equality alone is not a general concurrent-reader protocol.
        if (snapshot != confirm) {
            report("ring_snapshot", failure(EAGAIN), "buffer changed after stop");
            return false;
        }
        const std::string path = std::string(directory) + "/ring-" +
            std::to_string(static_cast<unsigned>(spec_.id)) + "-after-stop.bin";
        const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
        if (fd < 0) { report("ring_snapshot", failure(errno), path); return false; }
        std::size_t written = 0;
        bool ok = true;
        while (written < snapshot.size()) {
            const auto n = ::write(fd, snapshot.data() + written, snapshot.size() - written);
            if (n < 0 && errno == EINTR) continue;
            if (n <= 0) { ok = false; break; }
            written += static_cast<std::size_t>(n);
        }
        ok = (::close(fd) == 0) && ok;
        report("ring_snapshot", ok ? Result{} : failure(EIO), path + " bytes=" + std::to_string(written));
        if (spec_.id == head::RingId::pose) {
            pose_output::PoseSnapshot pose;
            const auto decoded = pose_output::decode_latest_pose_snapshot(snapshot.data(), snapshot.size(), pose);
            double norm_squared = 0;
            for (const auto value : pose.rotation_xyzw) norm_squared += static_cast<double>(value) * value;
            std::cout << "stage=pose_snapshot_decode rc=" << static_cast<unsigned>(decoded)
                      << " slot=" << pose.published_index << " timestamp_ns=" << pose.timestamp_ns
                      << " rotation_xyzw=" << pose.rotation_xyzw[0] << ',' << pose.rotation_xyzw[1] << ','
                      << pose.rotation_xyzw[2] << ',' << pose.rotation_xyzw[3]
                      << " quaternion_norm=" << std::sqrt(norm_squared)
                      << " translation=" << pose.translation[0] << ',' << pose.translation[1] << ',' << pose.translation[2]
                      << " stock_tracking_state=" << pose.tracking_state << " warning_flags=" << pose.warning_flags
                      << " pose_quality=" << pose.pose_quality << " sensor_quality=" << pose.sensor_quality
                      << " camera_quality=" << pose.camera_quality
                      << " openxr_valid=0 quiescent_snapshot=1" << std::endl;
            ok = decoded == pose_output::DecodeStatus::ok && ok;
        } else {
            pose_output::DlPredictionSnapshot dl;
            const auto decoded = pose_output::decode_latest_dl_prediction_snapshot(snapshot.data(), snapshot.size(), dl);
            std::cout << "stage=dl_snapshot_decode rc=" << static_cast<unsigned>(decoded)
                      << " slot=" << dl.published_index << " timestamp_ns=" << dl.timestamp_ns
                      << " coefficient_counts=" << unsigned(dl.coefficient_counts[0]) << ','
                      << unsigned(dl.coefficient_counts[1]) << ',' << unsigned(dl.coefficient_counts[2]) << ','
                      << unsigned(dl.coefficient_counts[3]) << " producer_marker=" << unsigned(dl.producer_marker)
                      << " openxr_valid=0 quiescent_snapshot=1" << std::endl;
            ok = decoded == pose_output::DecodeStatus::ok && ok;
        }
        return ok;
    }
    // Called only by the sole external-IMU/QVR RPC caller, after method10
    // returns and before any next IMU/camera call. Method10 runs the writer
    // synchronously; this path never registers native Android direct channels.
    // Copy equality/index checks detect unexpected changes, not a seqlock.
    // A missing update is explicit and does not suppress the real IMU packet.
    bool copy_after_notification(std::uint64_t gyro_time, a081_tracking_packet &packet)
    {
        if (spec_.id != head::RingId::pose) return false;
        if (!sync(DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ)) return false;
        std::atomic_thread_fence(std::memory_order_acquire);
        const auto *base = static_cast<const std::uint8_t *>(host_);
        std::uint32_t first = 0, last = 0;
        std::memcpy(&first, base + spec_.index, sizeof(first));
        std::array<std::uint8_t, pose_output::pose_slot_bytes> copy{}, confirm{};
        if (first < spec_.count) {
            const auto *slot = base + spec_.offset + first * spec_.element;
            std::memcpy(copy.data(), slot, copy.size());
            std::atomic_thread_fence(std::memory_order_acquire);
            std::memcpy(confirm.data(), slot, confirm.size());
        }
        std::atomic_thread_fence(std::memory_order_acquire);
        std::memcpy(&last, base + spec_.index, sizeof(last));
        if (!sync(DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ)) return false;
        packet.pose_index = first;
        if (first >= spec_.count) { packet.pose_read_status = A081_POSE_BAD_INDEX; return true; }
        if (first != last || copy != confirm) { packet.pose_read_status = A081_POSE_CHANGED_DURING_COPY; return true; }
        pose_output::PoseSnapshot pose;
        if (pose_output::decode_pose_record(copy.data(), copy.size(), first, pose) != pose_output::DecodeStatus::ok)
            return false;
        if (pose.timestamp_ns != gyro_time) { packet.pose_read_status = A081_POSE_TIMESTAMP_MISMATCH; return true; }
        packet.pose_read_status = A081_POSE_COPIED;
        packet.flags |= A081_TRACKING_RAW_POSE;
        packet.pose_boottime_ns = pose.timestamp_ns;
        std::copy(pose.rotation_xyzw.begin(), pose.rotation_xyzw.end(), packet.rotation_xyzw);
        std::copy(pose.translation.begin(), pose.translation.end(), packet.translation);
        packet.pose_quality = pose.pose_quality;
        packet.tracking_state = pose.tracking_state;
        packet.warning_flags = pose.warning_flags;
        return true;
    }
    void retain_until_process_exit() { retained_ = true; }
    bool release()
    {
        if (retained_) return false;
        bool ok = true;
        if (host_ != MAP_FAILED) {
            errno = 0;
            const auto result = sysresult(munmap(host_, spec_.bytes));
            report("ring_munmap", result, detail());
            ok = static_cast<bool>(result);
            host_ = MAP_FAILED;
        }
        if (fd_ >= 0) {
            errno = 0;
            const auto result = sysresult(::close(fd_));
            report("ring_close_fd", result, detail());
            ok = static_cast<bool>(result) && ok;
            fd_ = -1;
        }
        return ok;
    }
private:
    bool sync(std::uint64_t flags)
    {
        dma_buf_sync request{flags};
        int rc;
        do { errno = 0; rc = ioctl(fd_, DMA_BUF_IOCTL_SYNC, &request); } while (rc < 0 && errno == EINTR);
        if (rc != 0) report("ring_cpu_sync", sysresult(rc), detail() + " flags=" + std::to_string(flags));
        return rc == 0;
    }
    RingSpec spec_{};
    int fd_ = -1;
    void *host_ = MAP_FAILED;
    std::uint64_t baseline_{};
    bool have_baseline_{}, retained_{};
};

class VisualInput {
public:
    explicit VisualInput(std::uint64_t exposure_ns)
        : groups(exposure_ns, [this](const camera::FrameGroup &group) { controller_export.frame(group); }),
          exposure_ns(exposure_ns) {}
    ~VisualInput() { finish(); }
    void start()
    {
        callbacks.frame = [this](unsigned slot, camera::TimedFrame &&frame) { groups.push(slot, std::move(frame)); };
        callbacks.gate = [this](std::uint64_t time) { groups.gate_high(time); };
        callbacks.abort = &abort;
        worker = std::thread([this] {
            result.store(camera::capture_live(&callbacks));
            ended_at.store(boottime_ns());
            done.store(true);
        });
    }
    void finish()
    {
        abort.store(true);
        if (worker.joinable()) worker.join();
    }
    // finish() joins capture before groups and the exporter are destroyed.
    controller::FrameExporter controller_export;
    camera::FrameGroups groups;
    const std::uint64_t exposure_ns;
    std::atomic<bool> done{false}, abort{false};
    std::atomic<int> result{-1};
    std::atomic<std::uint64_t> ended_at{0};
private:
    camera::CaptureCallbacks callbacks;
    std::thread worker;
};

inline bool feed(QvrModule &module, IioStream &accel, IioStream &gyro, Ring &ring, Ring &pose_ring,
                 TrackingServer &stream, const std::array<int, 3> &axes, std::uint64_t start,
                 VisualInput *visual = nullptr)
{
    const char *continuous_value = std::getenv("A081_QVR_CONTINUOUS");
    const bool continuous = visual && continuous_value && std::string(continuous_value) == "1";
    const auto deadline = continuous ? UINT64_MAX : start + (visual ? 370000000000ULL : 5000000000ULL);
    const std::uint64_t max_notifications = continuous ? UINT64_MAX : (visual ? 200000 : 5000);
    std::deque<Sample> accel_history;
    std::uint64_t notified = 0, skipped = 0, first = 0, last = 0;
    std::uint64_t camera_calls = 0, last_camera_time = 0;
    std::uint64_t last_qvr_exposure_start = 0, camera_short_skipped = 0, camera_rate_skipped = 0;
    std::array<std::uint64_t, 5> pose_reads{};
    bool ok = true;
    while (!interrupted && boottime_ns() < deadline && notified < max_notifications) {
        if (visual && visual->done.load()) {
            if (visual->result.load() != 0) { ok = false; break; }
            // Once all real camera groups are submitted, stop the producer.
            // An extra IMU-only tail overwrites the small output ring and
            // obscures the quality of the last camera-fed records.
            if (visual->groups.empty()) break;
        }
        pollfd descriptors[2]{{accel.fd(), POLLIN, 0}, {gyro.fd(), POLLIN, 0}};
        const int ready = poll(descriptors, 2, 20);
        if (ready < 0) {
            if (errno == EINTR) continue;
            report("iio_poll", failure(errno)); ok = false; break;
        }
        if (ready == 0) continue;
        std::vector<Sample> batch;
        batch.reserve(128);
        for (unsigned i = 0; i < 2; ++i) {
            if (descriptors[i].revents & (POLLERR | POLLHUP | POLLNVAL)) { ok = false; break; }
            if ((descriptors[i].revents & POLLIN) && !(i ? gyro : accel).read_batch(batch, axes)) { ok = false; break; }
        }
        if (!ok) break;
        std::stable_sort(batch.begin(), batch.end(), [](const Sample &a, const Sample &b) {
            return a.time == b.time ? a.gyro < b.gyro : a.time < b.time;
        });
        for (const auto &sample : batch) {
            if (interrupted || boottime_ns() >= deadline || notified >= max_notifications) break;
            if (!sample.gyro) {
                accel_history.push_back(sample);
                if (accel_history.size() > 128) accel_history.pop_front();
                continue;
            }
            // Independent IIO delivery may expose a newer accel before an
            // older gyro is read. Retain real history instead of losing the
            // previous suitable sample, or relabelling the newer timestamp.
            const auto accel_at = std::find_if(accel_history.rbegin(), accel_history.rend(),
                [&](const Sample &a) { return a.time <= sample.time; });
            if (accel_at == accel_history.rend() ||
                sample.time - accel_at->time > 100000000ULL) { ++skipped; continue; }
            const auto &latest_accel = *accel_at;
            if (!ring.publish(static_cast<unsigned>(notified % 8), sample, latest_accel)) { ok = false; break; }
            const auto result = module.external_sensor_ready();
            if (!result) { report("external_sensor_ready", result); ok = false; break; }
            ++notified;
            if (!first) first = sample.time;
            last = sample.time;
            if (stream.enabled()) {
                a081_tracking_packet packet{};
                packet.flags = A081_TRACKING_IMU | (camera_calls ? A081_TRACKING_CAMERA_SUBMITTED : 0);
                packet.gyro_boottime_ns = sample.time; packet.accel_boottime_ns = latest_accel.time;
                packet.last_camera_boottime_ns = last_camera_time;
                std::copy(sample.value.begin(), sample.value.end(), packet.gyro);
                std::copy(latest_accel.value.begin(), latest_accel.value.end(), packet.accel);
                if (!pose_ring.copy_after_notification(sample.time, packet)) { ok = false; break; }
                ++pose_reads.at(packet.pose_read_status);
                stream.publish(packet);
                if (visual) visual->controller_export.pose(packet);
            }
            if (notified == 1 || notified % 128 == 0) {
                std::lock_guard<std::mutex> log(camera::output_mutex);
                std::cout << "stage=imu_publish rc=0 count=" << notified << " gyro_ns=" << sample.time
                          << " accel_ns=" << latest_accel.time << " slot=" << (notified - 1) % 8
                          << " gyro=" << sample.value[0] << ',' << sample.value[1] << ',' << sample.value[2]
                          << " accel=" << latest_accel.value[0] << ',' << latest_accel.value[1] << ',' << latest_accel.value[2]
                          << " position_valid=not_evaluated" << std::endl;
            }
        }
        if (!ok) break;
        if (visual) {
            camera::FrameGroup group;
            if (visual->groups.take(last, group)) {
                head::Bytes tracking, controllers;
                if (group.submit_boottime_ns <= last_camera_time ||
                    group.frames[0].image.sequence > INT32_MAX || group.frames[2].image.sequence > INT32_MAX ||
                    !camera::side_by_side(group.frames[0].image.y8, group.frames[1].image.y8, tracking) ||
                    !camera::side_by_side(group.frames[2].image.y8, group.frames[3].image.y8, controllers)) {
                    report("camera_group", failure(EINVAL), "Invalid real stereo payload/order"); ok = false; break;
                }
                // Stock VRTrackerF skips frames whose AE-active flag is false
                // and enforces a 30,303,030ns SOE interval before DSP submission.
                // In our kernel three-phase schedule only phase2 is the held-AE
                // tracking exposure. Controller export still receives both short
                // phases, with their real sensor frame numbers and timestamps.
                const auto &tracking_meta = group.frames[0].timing;
                camera::StockY8Time qvr_time{};
                if (!camera::stock_a081_y8_time(tracking_meta.sof_boottime_ns,
                        camera::frame_exposure_ns(tracking_meta, visual->exposure_ns), qvr_time)) {
                    report("camera_group", failure(EINVAL), "Invalid real tracking exposure time"); ok = false; break;
                }
                const bool short_phase = camera::valid_pattern_exposure(tracking_meta) &&
                    tracking_meta.exposure_phase != 2;
                const bool rate_skip = last_qvr_exposure_start &&
                    (qvr_time.exposure_start_boottime_ns <= last_qvr_exposure_start ||
                     qvr_time.exposure_start_boottime_ns - last_qvr_exposure_start < 30303030ULL);
                if (short_phase || rate_skip) {
                    if (short_phase) ++camera_short_skipped; else ++camera_rate_skipped;
                    continue;
                }
                // Only this thread invokes QVR, after IMU has reached the
                // camera time. Capture workers only publish bounded real data.
                const auto r = module.camera_frame_from_sof(group.frames[0].timing.sof_boottime_ns,
                    camera::frame_exposure_ns(group.frames[0].timing, visual->exposure_ns), tracking, controllers,
                    static_cast<std::int32_t>(group.frames[0].image.sequence),
                    static_cast<std::int32_t>(group.frames[2].image.sequence));
                report("camera_frame", r, "submit_boottime_ns=" + std::to_string(group.submit_boottime_ns) +
                       " tracking_frame=" + std::to_string(group.frames[0].image.sequence) +
                       " controller_frame=" + std::to_string(group.frames[2].image.sequence));
                if (!r) { ok = false; break; }
                if (camera_calls == 0) {
                    const std::string output = std::getenv("A081_QVR_OUTPUT_DIR");
                    for (unsigned slot = 0; slot < 4; ++slot)
                        camera::save_bytes(output + "/camera-slot" + std::to_string(slot) +
                                           "-first-submitted.y8", group.frames[slot].image.y8);
                }
                {
                    std::lock_guard<std::mutex> log(camera::output_mutex);
                    std::cout << "stage=camera_group_submitted submit_boottime_ns=" << group.submit_boottime_ns;
                    for (unsigned slot = 0; slot < 4; ++slot) {
                        const auto &f = group.frames[slot];
                        std::cout << " slot" << slot << "=" << f.timing.stream_id << ',' << f.image.sequence << ','
                                  << f.image.buffer_index << ',' << f.timing.enqueue_cookie << ',' << f.timing.sof_boottime_ns;
                    }
                    std::cout << " fields=stream_id,sequence,index,cookie,sof_boottime_ns" << std::endl;
                }
                ++camera_calls; last_camera_time = group.submit_boottime_ns;
                last_qvr_exposure_start = qvr_time.exposure_start_boottime_ns;
            }
        }
    }
    {
    std::lock_guard<std::mutex> log(camera::output_mutex);
    std::cout << "stage=imu_input_summary elapsed_ns=" << boottime_ns() - start
              << " accel_samples=" << accel.samples << " gyro_samples=" << gyro.samples
              << " discarded_accel=" << accel.discarded << " discarded_gyro=" << gyro.discarded
              << " skipped_no_recent_accel=" << skipped << " notifications=" << notified
              << " first_gyro_ns=" << first << " last_gyro_ns=" << last
              << " interrupted=" << interrupted << " camera_rpc_success=" << camera_calls
              << " camera_short_skipped=" << camera_short_skipped << " camera_rate_skipped=" << camera_rate_skipped
              << " last_camera_submit_ns=" << last_camera_time << " visual_6dof=not_validated" << std::endl;
    }
    accel.report_rate();
    gyro.report_rate();
    if (stream.enabled()) {
        std::cout << "stage=pose_stream_reads copied=" << pose_reads[A081_POSE_COPIED]
                  << " bad_index=" << pose_reads[A081_POSE_BAD_INDEX]
                  << " changed=" << pose_reads[A081_POSE_CHANGED_DURING_COPY]
                  << " timestamp_mismatch=" << pose_reads[A081_POSE_TIMESTAMP_MISMATCH]
                  << " openxr_position_valid=0" << std::endl;
    }
    return ok && !interrupted && notified > 0 && (!visual || camera_calls > 0);
}

inline int run(int session, const std::string &uri)
{
    if (!a081_apps_mem_counters || !a081_apps_mem_lifetimes) {
        report("imu_live_gate", failure(ENOTSUP), "requires private in-process -P plugin");
        return 2;
    }
    std::array<int, 3> axes{};
    unsigned requested_rate = 100;
    std::unique_ptr<VisualInput> visual;
    if (const char *option = std::getenv("A081_QVR_VISUAL")) {
        const char *value = std::getenv("A081_QVR_CAMERA_EXPOSURE_NS");
        char *end = nullptr;
        errno = 0;
        const auto exposure = value ? std::strtoull(value, &end, 10) : 0;
        if (std::string(option) != "qcft" || errno || !exposure || exposure > 1000000000ULL ||
            !end || *end || !std::getenv("A081_QVR_CAMERA_TIMING_NODES") || !std::getenv("A081_QVR_CAMERA_STOP_FD")) {
            report("visual_gate", failure(EINVAL), "Requires QCFT and a real gate owner with fixed pre-STREAMON exposure"); return 2;
        }
        visual = std::make_unique<VisualInput>(exposure);
    }
    if (const char *rate = std::getenv("A081_QVR_IMU_HZ")) {
        const std::string selected(rate);
        if (selected != "25" && selected != "50" && selected != "100" && selected != "200" && selected != "500") {
            report("imu_live_gate", failure(EINVAL), "A081_QVR_IMU_HZ must be 25,50,100,200 or 500");
            return 2;
        }
        requested_rate = static_cast<unsigned>(std::stoul(selected));
    }
    std::string model, config, calibration;
    const char *cvp_option = std::getenv("A081_QVR_CVP_MODE");
    const std::string cvp_mode = cvp_option ? cvp_option : "stock";
    if (cvp_mode != "stock" && cvp_mode != "disabled") {
        report("imu_live_gate", failure(EINVAL), "A081_QVR_CVP_MODE must be stock or disabled");
        return 2;
    }
    // The stock parser and Tracker3_5Core branches support this exact XML
    // option. It disables CVP work; it does not emulate a CVP service or alter
    // the signed algorithm. See cvp-start-dependency.json.
    const char *config_hash = cvp_mode == "disabled"
        ? "f67e7189447212a13b82bb18cc14161221e84d1f36eece205eed6a61ed9be276"
        : "aeef461d9ac5b191f0d2c3e83de3a5d45f305b0f7286c8fc4c5bcecb3b612db9";
    if (!text_file("/proc/device-tree/model", model) || model.find("A081") == std::string::npos ||
        !axis_mapping(axes) ||
        !verified_xml("A081_QVR_CONFIG6", config_hash, config) ||
        !verified_xml("A081_QVR_CALIBRATION", "eeee483e5ac268a85cd2b6ba69241bb951612045fe05d61de2f35301d816ac00", calibration)) {
        report("imu_live_gate", failure(EINVAL), "A081 identity, explicit axes, pinned config variant and original calibration are required before mutation");
        return 2;
    }
    report("config_variant", {}, "cvp_mode=" + cvp_mode +
           " config_sha256=" + config_hash + " signed_algorithm=unchanged calibration=original");
    config.erase(std::remove(config.begin(), config.end(), '\n'), config.end());
    std::cout << "stage=imu_live_scope max_input_seconds=" << (visual ? 370 : 5) << " max_notifications=" << (visual ? 200000 : 5000) <<
                 " source=real_qcom_see_iio camera_input=" << (visual ? "real_qcft_four_camera" : "none")
              << " visual_6dof=not_validated DSP_start_sensor_frequency=" << requested_rate
              << " continuous_requested=" << (std::getenv("A081_QVR_CONTINUOUS") ? std::getenv("A081_QVR_CONTINUOUS") : "0")
              << " actual_frequency=reported_per_stream RPC_calls=synchronous_external_process_timeout_still_required" << std::endl;
    Signals signals;
    IioLocks locks;
    IioStream accel("qcom-see-accel", "accel", false), gyro("qcom-see-gyro", "anglvel", true);
    std::array<Ring, 7> rings;
    TrackingServer stream;
    QvrModule module(session);
    bool opened = false, initialized = false, ok = true, safe_to_release = true;
    std::uint64_t before[4]{};
    a081_apps_mem_counters(before);
    std::uint64_t owned_before[6]{};
    a081_apps_mem_lifetimes(owned_before);
    std::string message;
    try {
        ok = stream.open_from_environment();
        if (!ok) report("tracking_stream_open", failure(errno));
        ok = ok && signals.install() && locks.acquire() && accel.discover() && gyro.discover() &&
             accel.configure(requested_rate) && gyro.configure(requested_rate);
        if (ok) { const auto r = module.open(uri, message); report("open", r, message); opened = ok = static_cast<bool>(r); }
        if (ok) { const auto r = module.driver_version(message); report("driver_version", r, message); ok = static_cast<bool>(r); }
        for (std::size_t i = 0; ok && i < rings.size(); ++i) ok = rings[i].allocate(session, ring_specs[i]);
        if (ok) { const auto r = module.init(); report("init", r); initialized = ok = static_cast<bool>(r); }
        if (ok) { const auto r = module.tracker_version(message); report("tracker_version", r, message); ok = static_cast<bool>(r); }
        for (std::size_t i = 0; ok && i < rings.size(); ++i) {
            const auto r = module.set_ring_layout(ring_specs[i].id, rings[i].descriptor());
            report("set_ring_layout", r, rings[i].detail());
            ok = static_cast<bool>(r);
        }
        for (auto &ring : rings) if (ok) ok = ring.summary("before_start");
        if (ok) { const auto r = module.enable_position_mode(1); report("enable_position_mode", r, "DSP mode=1"); ok = static_cast<bool>(r); }
        if (ok) { const auto r = module.sensor_orientation_identity(); report("sensor_orientation", r, "stock indices=0,1,2 signs=1,1,1"); ok = static_cast<bool>(r); }
        if (ok) {
            std::uint64_t ticks = 0;
            const auto r = module.qtimer_ticks(ticks);
            report("qtimer_ticks", r, "raw_ticks=" + std::to_string(ticks) + " external IMU retains BOOTTIME");
            ok = static_cast<bool>(r);
        }
        // QVR DSP start ELF0x7a3c only calls set_cvp_fds if camera_fds_len>0.
        // Supply the real calibration names/XML but no fabricated camera fds
        // or frames. No observed hard camera-buffer gate precedes tracker init.
        if (ok) {
            const auto r = module.start(config, calibration, static_cast<float>(requested_rate));
            report("start", r, std::string("stock flags=0 CVP_buffers=0 camera_input=") + (visual ? "inline_Y8" : "none") + " requested_hz=" + std::to_string(requested_rate));
            ok = static_cast<bool>(r);
        }
        if (ok && !interrupted) {
            const auto start_time = boottime_ns();
            ok = start_time != 0 && accel.enable() && gyro.enable();
            if (ok && visual) visual->start();
            if (ok) ok = feed(module, accel, gyro, rings[1], rings[2], stream, axes, start_time, visual.get());
        } else if (interrupted) {
            ok = false;
        }
        if (module.started())
            for (auto &ring : rings) ok = ring.summary("after_input_raw_snapshot") && ok;
    } catch (const std::exception &e) {
        report("imu_live_exception", failure(EIO), e.what());
        ok = false;
    }
    // EOF invalidates consumers before DSP stop/deinit and IIO restoration.
    { std::lock_guard<std::mutex> log(camera::output_mutex); stream.close(); }
    if (visual) {
        visual->finish();
        ok = visual->result.load() == 0 && ok;
        visual->groups.summary();
    }
    // All cleanup is explicit so the log includes the real return values.
    ok = gyro.disable() && ok;
    ok = accel.disable() && ok;
    if (module.stop_required()) {
        report("stop_begin", {}, "boottime_ns=" + std::to_string(boottime_ns()));
        const auto r = module.stop(); report("stop", r);
        safe_to_release = static_cast<bool>(r);
        ok = safe_to_release && ok;
    }
    if (initialized && safe_to_release)
        for (auto &ring : rings) ok = ring.summary("after_stop") && ok;
    if (initialized && safe_to_release)
        for (auto &ring : rings) ok = ring.save_stopped_snapshot() && ok;
    if (initialized && safe_to_release) {
        const auto r = module.deinit(); report("deinit", r);
        ok = static_cast<bool>(r) && ok;
    }
    if (opened && safe_to_release) {
        const auto r = module.close(message); report("close", r, message);
        safe_to_release = static_cast<bool>(r);
        ok = safe_to_release && ok;
    }
    std::uint64_t after[4]{};
    a081_apps_mem_counters(after);
    std::uint64_t owned[6]{};
    a081_apps_mem_lifetimes(owned);
    const bool balanced = after[2] == before[2] && after[0] - before[0] == after[1] - before[1];
    std::cout << "stage=imu_mapping_check rc=" << (balanced ? 0 : 1)
              << " maps=" << after[0] - before[0] << " unmaps=" << after[1] - before[1]
              << " live=" << after[2] << " errors=" << after[3] - before[3]
              << " scope=explicit_unmap" << std::endl;
    const auto borrowed_maps = (after[0] - before[0]) - (owned[0] - owned_before[0]);
    const auto borrowed_unmaps = (after[1] - before[1]) - (owned[1] - owned_before[1]);
    const bool fresh = before[0] == 0 && before[1] == 0 && before[2] == 0 && before[3] == 0 &&
                       std::all_of(std::begin(owned_before), std::end(owned_before),
                                   [](std::uint64_t value) { return value == 0; });
    const bool borrowed_balanced = fresh && after[2] >= owned[2] &&
        after[2] - owned[2] == 0 && borrowed_maps == borrowed_unmaps;
    // Only the exact original MemLogger request may cross the module boundary.
    // Exit3 means REQUIRED post-process proof, never a successful session.
    const bool pd_release_pending = owned[2] == 1 && owned[4] == 1 &&
        owned[3] == 2101248 && owned[5] == 2101248 && owned[0] == owned[1] + 1;
    const bool owned_balanced = owned[2] == 0 && owned[3] == 0 && owned[4] == 0 &&
        owned[5] == 0 && owned[0] == owned[1];
    std::cout << "stage=imu_module_mapping_check rc=" << (borrowed_balanced ? 0 : 1)
              << " borrowed_maps=" << borrowed_maps << " borrowed_unmaps=" << borrowed_unmaps
              << " total_live=" << after[2] << " owned_live=" << owned[2]
              << " owned_bytes=" << owned[3] << std::endl;
    safe_to_release = safe_to_release && borrowed_balanced && (owned_balanced || pd_release_pending);
    ok = safe_to_release && after[3] == before[3] && ok;
    for (auto &ring : rings) {
        if (safe_to_release) ok = ring.release() && ok;
        else ring.retain_until_process_exit();
    }
    if (!safe_to_release)
        report("ring_release_deferred", failure(EBUSY), "retain DMA mappings/fds until private plugin process exits and closes its FastRPC session");
    ok = gyro.restore() && ok;
    ok = accel.restore() && ok;
    if (ok && pd_release_pending) {
        std::cout << "stage=imu_pd_release_required rc=3 owned_live=" << owned[2]
                  << " owned_bytes=" << owned[3]
                  << " final_reclamation=not_yet_observed" << std::endl;
        return 3;
    }
    return ok ? 0 : 1;
}
} // namespace a081::stock::imu_live
