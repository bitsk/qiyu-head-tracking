// SPDX-License-Identifier: GPL-3.0-or-later
#include "qvr_bridge.hpp"
#include "imu-live.hpp"
#include <qiyu/device/a081_camera_capture.hpp>

#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <misc/fastrpc.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

// Available only in the private daemon, with -Wl,--export-dynamic. A forked
// client has a different fd table and cannot use this health-ring experiment.
extern "C" void a081_apps_mem_counters(std::uint64_t out[4]) __attribute__((weak));

namespace {
using a081::stock::Result;
void report(const char *stage, const Result &result, const std::string &extra = {})
{
    std::cout << "stage=" << stage << " rc=" << result.rc << " errno=" << result.system_errno
              << " loader_error_word=" << result.loader_error_word
              << " message=" << std::quoted(a081::stock::describe(result));
    if (!extra.empty()) std::cout << " detail=" << std::quoted(extra);
    std::cout << std::endl;
}
Result syscall_result(int rc) { return {rc, rc == -1 ? errno : 0}; }

class HealthMemory {
public:
    // AP SetupRingLayout + raw static descriptor, see health-layout.json.
    static constexpr std::uint32_t block_bytes = 8192;
    ~HealthMemory() { release(); }
    Result allocate(int session_fd)
    {
        fastrpc_alloc_dma_buf request{};
        request.fd = -1;
        request.size = block_bytes;
        errno = 0;
        auto result = syscall_result(ioctl(session_fd, FASTRPC_IOCTL_ALLOC_DMA_BUFF, &request));
        report("health_alloc", result, "fd=" + std::to_string(request.fd) + " length=8192");
        if (!result) return result;
        fd_ = request.fd;
        errno = 0;
        host_ = mmap(nullptr, block_bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        result = host_ == MAP_FAILED ? Result{-1, errno} : Result{};
        report("health_mmap", result, detail());
        if (result) {
            // Current mainline FastRPC allocator uses dma_alloc_coherent and
            // dma_mmap_coherent. This is only the fresh health allocation.
            std::memset(host_, 0, block_bytes);
            std::atomic_thread_fence(std::memory_order_release);
        }
        return result;
    }
    a081::head::RingDescriptor layout() const
    {
        return {fd_, block_bytes, 6816, 0, 6816, 1, 7, 0};
    }
    bool release()
    {
        bool success = true;
        if (host_ != MAP_FAILED) {
            errno = 0;
            auto result = syscall_result(munmap(host_, block_bytes));
            report("health_munmap", result, detail());
            success = static_cast<bool>(result);
            host_ = MAP_FAILED;
        }
        if (fd_ >= 0) {
            errno = 0;
            auto result = syscall_result(::close(fd_));
            report("health_close_fd", result, detail());
            success = static_cast<bool>(result) && success;
            fd_ = -1;
        }
        return success;
    }
private:
    std::string detail() const { return "fd=" + std::to_string(fd_) + " length=8192"; }
    int fd_ = -1;
    void *host_ = MAP_FAILED;
};

void usage()
{
    std::cout << "Usage: a081-qvr-stock [--init | --health | --imu-live | --camera-capture] [--uri URI]\n"
                 "Requires HEXAGONRPC_FD from a CDSP hexagonrpcd session.\n"
                 "Default: open, driver version, close. --init adds init, tracker version, deinit.\n"
                 "Health requires private hexagonrpcd -P liba081-qvr-client.so, using\n"
                 "A081_QVR_STAGE=health: init, ring7 share_map, deinit, share_unmap.\n"
                 "A081_QVR_STAGE=version|init|health|imu-live and A081_QVR_URI are environment options.\n"
                 "imu-live requires the private plugin, exact stock XML paths in A081_QVR_CONFIG6\n"
                 "and A081_QVR_CALIBRATION, plus a verified signed axis permutation in\n"
                 "A081_QVR_IIO_TO_ANDROID. It starts the stock tracker for at most 5 seconds\n"
                 "of real BOOTTIME IIO samples; camera/visual 6DoF are not tested.\n"
                 "A081_QVR_CVP_MODE=disabled requires the pinned config variant with the\n"
                 "original algorithm's CvpConfig enable=false option; default is stock.\n"
                 "A081_QVR_IMU_HZ selects 25,50,100(default),200 or 500Hz; actual timestamps\n"
                 "are preserved and both IIO rates are restored after this bounded run.\n"
                 "--camera-capture needs no DSP session: capture configured real V4L2 nodes\n"
                 "in A081_QVR_CAMERA_NODES=/dev/videoN,... for 5 seconds. Save first raw/Y8\n"
                 "frames to a fresh A081_QVR_OUTPUT_DIR. It changes no graph/GPIO/controls\n"
                 "and submits no QVR frames until exposure/sync metadata is established.\n";
}

int run_client(int argc, char **argv)
{
    std::string stage = "version";
    if (const char *value = std::getenv("A081_QVR_STAGE")) stage = value;
    std::string uri = a081::stock::stock_uri;
    if (const char *value = std::getenv("A081_QVR_URI")) uri = value;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help") { usage(); return 0; }
        if (arg == "--init") stage = "init";
        else if (arg == "--health") stage = "health";
        else if (arg == "--imu-live") stage = "imu-live";
        else if (arg == "--camera-capture") stage = "camera-capture";
        else if (arg == "--uri" && i + 1 < argc) uri = argv[++i];
        else { usage(); return 2; }
    }
    if (stage == "camera-capture") return a081::stock::camera::capture_live();
    if (stage != "version" && stage != "init" && stage != "health" && stage != "imu-live") {
        std::cerr << "A081_QVR_STAGE must be version, init, health, imu-live or camera-capture\n";
        return 2;
    }
    const bool health = stage == "health";
    if (health && !a081_apps_mem_counters) {
        std::cerr << "Health mode requires the private daemon's in-process -P plugin.\n";
        return 2;
    }
    const int fd = a081::stock::inherited_session_fd();
    if (fd < 0 || fcntl(fd, F_GETFD) == -1) {
        std::cerr << "No valid inherited HEXAGONRPC_FD; run as hexagonrpcd client.\n";
        return 2;
    }
    std::cout << "session_fd=" << fd << " mode=" << stage
              << " uri=" << std::quoted(uri) << std::endl;
    if (stage == "imu-live") return a081::stock::imu_live::run(fd, uri);
    std::uint64_t before[4]{};
    if (health) a081_apps_mem_counters(before);
    // Keep the DMA fd alive until the module's teardown/close has completed,
    // including exception unwinding or early returns.
    HealthMemory memory;
    a081::stock::QvrModule module(fd);
    std::string message;
    auto result = module.open(uri, message);
    report("open", result, message);
    if (!result) return 1;
    std::cout << "remote_handle=" << module.handle() << std::endl;

    std::string version;
    result = module.driver_version(version);
    report("driver_version", result, version);
    bool success = static_cast<bool>(result);
    if (success && health) success = static_cast<bool>(memory.allocate(fd));
    if (success && stage != "version") {
        result = module.init();
        report("init", result);
        success = static_cast<bool>(result);
        if (success) {
            if (health) {
                result = module.set_ring_layout(a081::head::RingId::health, memory.layout());
                report("health_set_ring_layout", result,
                       "ring=7 fd=" + std::to_string(memory.layout().fd) +
                       " block=8192 index_offset=6816 ring_offset=0 element=6816 count=1 type=7");
                success = static_cast<bool>(result);
            } else {
                result = module.tracker_version(version);
                report("tracker_version", result, result ? version : std::string{});
                success = static_cast<bool>(result);
            }
            result = module.deinit();
            report("deinit", result);
            success = static_cast<bool>(result) && success;
        }
    }
    result = module.close(message);
    report("close", result, message);
    success = static_cast<bool>(result) && success;
    if (health) {
        std::uint64_t after[4]{};
        a081_apps_mem_counters(after);
        // Stock common_deinit ignores share_unmap's result, so its successful
        // return alone cannot establish that the remote mapping was removed.
        const bool matched = after[0] == before[0] + 1 && after[1] == before[1] + 1 &&
                             after[2] == before[2] && after[3] == before[3];
        std::cout << "stage=health_mapping_check rc=" << (matched ? 0 : 1)
                  << " maps=" << after[0] - before[0] << " unmaps=" << after[1] - before[1]
                  << " live=" << after[2] << " errors=" << after[3] - before[3] << std::endl;
        success = matched && success;
    }
    success = memory.release() && success;
    return success ? 0 : 1;
}
} // namespace

extern "C" int a081_qvr_client_main(void) { return run_client(0, nullptr); }
#ifndef A081_QVR_PLUGIN
int main(int argc, char **argv) { return run_client(argc, argv); }
#endif
