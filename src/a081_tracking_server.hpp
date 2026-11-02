// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <qiyu/head/a081_tracking_wire.h>
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/random.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

namespace a081::stock {
class TrackingServer {
public:
    ~TrackingServer() { close(); }
    bool open_from_environment()
    {
        const char *path = std::getenv("A081_QVR_TRACKING_SOCKET");
        if (!path || !*path) return true; // Existing bounded path remains unchanged.
        sockaddr_un addr{};
        if (path[0] != '/' || std::strlen(path) >= sizeof(addr.sun_path)) { errno = EINVAL; return false; }
        if (getrandom(&session_, sizeof(session_), 0) != static_cast<ssize_t>(sizeof(session_)) || !session_) return false;
        fd_ = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd_ < 0) return false;
        addr.sun_family = AF_UNIX;
        std::strcpy(addr.sun_path, path);
        // Do not remove an existing endpoint: another producer may own it.
        if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) return false;
        path_ = path;
        struct stat st{};
        if (lstat(path, &st) != 0) return false;
        inode_ = st.st_ino; device_ = st.st_dev;
        if (chmod(path, 0600) != 0 || listen(fd_, 4) != 0) return false;
        return true;
    }
    bool enabled() const { return fd_ >= 0 && !path_.empty(); }
    static std::uint64_t now(clockid_t clock)
    {
        timespec t{};
        if (clock_gettime(clock, &t) != 0) return 0;
        return static_cast<std::uint64_t>(t.tv_sec) * 1000000000ULL + t.tv_nsec;
    }
    void publish(a081_tracking_packet &packet)
    {
        if (!enabled()) return;
        packet.magic = A081_TRACKING_MAGIC; packet.version = A081_TRACKING_VERSION;
        packet.bytes = sizeof(packet); packet.session_id = session_; packet.sequence = ++sequence_;
        packet.flags &= ~A081_TRACKING_CLOCK;
        packet.sample_monotonic_ns = 0;
        for (unsigned attempt = 0; attempt < 3; ++attempt) {
            if (attempt) ++clock_retries_;
            // Each attempt is one complete measured tuple. Never combine
            // readings from different attempts or change DSP/camera time.
            const auto before = now(CLOCK_MONOTONIC);
            const auto boot = now(CLOCK_BOOTTIME);
            const auto after = now(CLOCK_MONOTONIC);
            packet.send_monotonic_ns = after;
            packet.clock_bracket_ns = after >= before && after - before <= UINT32_MAX ?
                                      static_cast<std::uint32_t>(after - before) : UINT32_MAX;
            if (before && boot && after >= before && packet.clock_bracket_ns <= A081_TRACKING_MAX_BRACKET_NS &&
                boot >= packet.gyro_boottime_ns && boot - packet.gyro_boottime_ns <= A081_TRACKING_MAX_AGE_NS) {
                const auto mono = before + (after - before) / 2;
                const auto age = boot - packet.gyro_boottime_ns;
                if (mono > age) {
                    packet.sample_monotonic_ns = mono - age;
                    packet.flags |= A081_TRACKING_CLOCK;
                    break;
                }
            }
        }
        if (!(packet.flags & A081_TRACKING_CLOCK)) ++clock_invalid_;
        // Bounded accept/send work on the only QVR caller thread. A slow client
        // is disconnected; it must reconnect instead of reading stale backlog.
        for (unsigned i = 0; i < peers_.size(); ++i) {
            const int peer = accept4(fd_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
            if (peer < 0) break;
            ucred cred{}; socklen_t size = sizeof(cred);
            auto vacant = peers_.size();
            for (std::size_t j = 0; j < peers_.size(); ++j) if (peers_[j] < 0) { vacant = j; break; }
            if (vacant == peers_.size() || getsockopt(peer, SOL_SOCKET, SO_PEERCRED, &cred, &size) != 0 ||
                size != sizeof(cred) || cred.uid != geteuid()) { ::close(peer); continue; }
            peers_[vacant] = peer; ++connections_;
        }
        for (auto &peer : peers_) {
            if (peer < 0) continue;
            if (send(peer, &packet, sizeof(packet), MSG_DONTWAIT | MSG_NOSIGNAL) == static_cast<ssize_t>(sizeof(packet))) ++delivered_;
            else { ++disconnects_; ::close(peer); peer = -1; }
        }
    }
    void close()
    {
        for (auto &peer : peers_) if (peer >= 0) { ::close(peer); peer = -1; }
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        struct stat st{};
        if (!path_.empty() && lstat(path_.c_str(), &st) == 0 &&
            st.st_ino == inode_ && st.st_dev == device_) unlink(path_.c_str());
        if (!path_.empty()) std::printf("stage=tracking_stream_summary packets=%llu deliveries=%llu connections=%llu disconnects=%llu clock_retries=%llu clock_invalid=%llu\n",
            static_cast<unsigned long long>(sequence_), static_cast<unsigned long long>(delivered_),
            static_cast<unsigned long long>(connections_), static_cast<unsigned long long>(disconnects_),
            static_cast<unsigned long long>(clock_retries_), static_cast<unsigned long long>(clock_invalid_));
        path_.clear();
    }
private:
    int fd_{-1};
    std::array<int, 4> peers_{{-1, -1, -1, -1}};
    std::string path_;
    ino_t inode_{}; dev_t device_{};
    std::uint64_t session_{}, sequence_{}, delivered_{}, connections_{}, disconnects_{}, clock_retries_{}, clock_invalid_{};
};
} // namespace a081::stock
