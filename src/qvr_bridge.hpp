// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <qiyu/device/abi_reference.hpp>

#include <cstdint>
#include <string>
#include <string_view>

namespace a081::stock {

// Uses a session already initialized by hexagonrpcd. It neither opens a device
// node nor creates a PD, and never assumes the Android remote_handle64 ABI.
struct Result {
    int rc{};
    int system_errno{};
    // remotectl's final output word: local header calls it err_valid_len.
    // Keep it separate from the ioctl result and do not decode it as an errno.
    std::uint32_t loader_error_word{};
    explicit operator bool() const { return rc == 0 && loader_error_word == 0; }
};

// Anonymous RPC buffers only. Shared DMA-buf registration is a separate step;
// embedding an fd number in a QVR ring descriptor does not register that fd.
Result invoke(int session_fd, std::uint32_t remote_handle, head::Invocation &call);

inline constexpr const char *stock_uri =
    "file:///libqvr_dsp_driver_skel.so?qvr_dsp_driver_skel_handle_invoke"
    "&_modver=1.0&_dom=cdsp"
    "&tracker=libtracker_6dof_skel_8250.so"
    "&tracker_3dof=libqvr_3dof_tracker_skel.so";

class QvrModule {
public:
    explicit QvrModule(int borrowed_session_fd) : fd_(borrowed_session_fd) {}
    ~QvrModule();
    QvrModule(const QvrModule &) = delete;
    QvrModule &operator=(const QvrModule &) = delete;

    Result open(const std::string &uri, std::string &loader_message);
    Result driver_version(std::string &version);
    Result tracker_version(std::string &version);
    Result init();
    Result set_ring_layout(head::RingId id, const head::RingDescriptor &layout);
    Result enable_position_mode(std::uint32_t mode = 1);
    // Stock [1,2,3] orientation -> indices [0,1,2], signs [1,1,1]. This
    // configures the stock frame; it does not determine the IIO mount matrix.
    Result sensor_orientation_identity();
    // A081 native arguments from start-contract.json. config6 must already
    // have LF removed (retain CR); calibration is the unchanged XML content.
    // Neither string includes its terminating NUL. Caller must configure the
    // rings, mode and orientation before start, and retain all mapped rings
    // until stop and deinit succeed. No camera/sensor input is synthesized.
    Result start(std::string_view config6, std::string_view calibration, float sensor_frequency_hz = 1000.0f);
    Result stop();
    // Publish the complete combined 64-byte ring0 record and its index before
    // this synchronous call. It consumes the current slot, not a queue. RPC
    // success does not establish that the tracker emitted a valid pose.
    Result external_sensor_ready();
    // Exact A081 inline-Y8 path from camera-input-contract.json. Each buffer
    // is 480 rows of 640 left + 640 right bytes (1280-byte stride). Primary is
    // trackingA/B, auxiliary is ctrl-trackingA/B. Caller must establish their
    // real physical identity, exposure pairing and a common timestamp domain
    // before calling. Our external-IMU path requires BOOTTIME for the camera
    // too; the stock native-sensor branch uses QTimer instead. Never mix them.
    // EOF/receipt time is insufficient; this method performs no conversion.
    // The QCFT caller requires the physical gate, fixed controls and verified
    // metadata association. RPC success alone does not establish valid 6DoF.
    Result camera_frame(std::uint64_t aligned_capture_time_ns,
                        const head::Bytes &tracking_pair, const head::Bytes &controller_pair,
                        std::int32_t frame_number, std::int32_t aux_frame_number);
    // Same import using the recovered A081 CamX/Y8 timing convention. Inputs
    // require a frame-associated SOF in BOOTTIME and effective exposure; EOF
    // or event notification time cannot be used. See a081_camera_timing.hpp.
    Result camera_frame_from_sof(std::uint64_t sof_boottime_ns, std::uint64_t effective_exposure_ns,
                                 const head::Bytes &tracking_pair, const head::Bytes &controller_pair,
                                 std::int32_t frame_number, std::int32_t aux_frame_number);
    // qurt_sysclock_get_hw_ticks(), not nanoseconds or CLOCK_MONOTONIC.
    // Available after open; out is changed only on RPC success.
    Result qtimer_ticks(std::uint64_t &out);
    Result deinit();
    Result close(std::string &loader_message);
    std::uint32_t handle() const { return handle_; }
    bool started() const { return started_; }
    // A submitted start can partially initialize the DSP even on failure.
    // Explicitly stop after any submitted start, then deinit/close. If stop
    // fails, leave the mappings intact and let the owner tear down the session.
    bool stop_required() const { return needs_stop_; }

private:
    Result version(std::uint32_t scalar, std::string &out);
    int fd_;
    std::uint32_t handle_{};
    bool open_{};
    bool initialized_{};
    bool started_{};
    bool needs_stop_{};
    bool stop_attempted_{};
};

int inherited_session_fd();
std::string describe(const Result &result);
} // namespace a081::stock
