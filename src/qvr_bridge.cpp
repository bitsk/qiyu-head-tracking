// SPDX-License-Identifier: GPL-3.0-or-later
#include "qvr_bridge.hpp"
#include <qiyu/device/a081_camera_timing.hpp>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <sys/ioctl.h>
#include <misc/fastrpc.h>

extern "C" {
#include <libhexagonrpc/error.h>
#include <libhexagonrpc/interface/remotectl.h>
#include <libhexagonrpc/session.h>
}

namespace a081::stock {
namespace {
Result result_from_rc(int rc) { return {rc, rc == -1 ? errno : 0}; }

void word(head::Bytes &b, std::uint32_t value, std::size_t offset = 0)
{
    for (unsigned i = 0; i < 4; ++i)
        b.at(offset + i) = static_cast<std::uint8_t>(value >> (8 * i));
}

Result local_failure(int number) { return {-1, number}; }
} // namespace

Result invoke(int fd, std::uint32_t handle, head::Invocation &call)
{
    const auto in_count = (call.scalar >> 16) & 255u;
    const auto out_count = (call.scalar >> 8) & 255u;
    if ((call.scalar & 0xe00000ffu) != 0 || in_count != call.inputs.size() ||
        out_count != call.outputs.size())
        return local_failure(EINVAL);

    std::vector<fastrpc_invoke_args> args;
    args.reserve(in_count + out_count);
    auto append = [&](std::vector<head::Bytes> &buffers) {
        for (auto &buffer : buffers) {
            fastrpc_invoke_args arg{};
            arg.ptr = buffer.empty() ? 0 : reinterpret_cast<std::uintptr_t>(buffer.data());
            arg.length = buffer.size();
            arg.fd = -1;
            args.push_back(arg);
        }
    };
    append(call.inputs);
    append(call.outputs);
    fastrpc_invoke request{};
    request.handle = handle;
    request.sc = call.scalar;
    request.args = args.empty() ? 0 : reinterpret_cast<std::uintptr_t>(args.data());
    errno = 0;
    return result_from_rc(ioctl(fd, FASTRPC_IOCTL_INVOKE, &request));
}

QvrModule::~QvrModule()
{
    // Stop before releasing DSP state. Do not hide an explicit failed stop by
    // retrying it during destruction, or deinit while the DSP may still run.
    // The borrowed session's owner performs the final session teardown.
    if (needs_stop_ && !stop_attempted_) stop();
    if (needs_stop_) return;
    if (initialized_) deinit();
    if (open_) {
        std::string ignored;
        close(ignored);
    }
}

Result QvrModule::open(const std::string &uri, std::string &message)
{
    if (open_) return local_failure(EALREADY);
    if (uri.empty() || uri.find('\0') != std::string::npos || uri.size() > 4095)
        return local_failure(EINVAL);

    char error[1024]{};
    std::uint32_t loader_error = 0;
    std::uint32_t candidate = 0;
    errno = 0;
    const int rc = remotectl_open(fd_, static_cast<std::uint32_t>(uri.size() + 1), uri.c_str(),
                                 &candidate, sizeof(error), error, &loader_error);
    const Result transport = result_from_rc(rc);
    message.assign(error, strnlen(error, sizeof(error)));
    if (!transport) return transport;
    if (loader_error != 0) {
        return {0, 0, loader_error};
    }
    handle_ = candidate;
    open_ = true;
    return {};
}

Result QvrModule::version(std::uint32_t scalar_value, std::string &out)
{
    if (!open_) return local_failure(EBADF);
    // REA: qvr_dsp_driver_get_version ELF0x34e8 / get_6dof_version ELF0x3424.
    // A four-byte capacity primitive plus one output string, no result word.
    head::Invocation call{scalar_value, {head::Bytes(4)}, {head::Bytes(64)}};
    word(call.inputs[0], 64);
    const Result result = invoke(fd_, handle_, call);
    if (result) {
        const auto &bytes = call.outputs[0];
        const auto end = std::find(bytes.begin(), bytes.end(), 0);
        out.assign(bytes.begin(), end);
    }
    return result;
}

Result QvrModule::driver_version(std::string &out) { return version(0x15010100, out); }
Result QvrModule::tracker_version(std::string &out) { return version(0x12010100, out); }

Result QvrModule::init()
{
    if (!open_) return local_failure(EBADF);
    if (initialized_) return local_failure(EALREADY);
    head::Invocation call{head::qvr_method::init, {}, {}};
    const Result result = invoke(fd_, handle_, call);
    initialized_ = static_cast<bool>(result);
    return result;
}

Result QvrModule::deinit()
{
    if (!initialized_) return local_failure(EINVAL);
    if (needs_stop_) return local_failure(EBUSY);
    head::Invocation call{0x05000000, {}, {}};
    const Result result = invoke(fd_, handle_, call);
    // Do not retry a failed DSP teardown implicitly in the destructor.
    initialized_ = false;
    return result;
}

Result QvrModule::set_ring_layout(head::RingId id, const head::RingDescriptor &layout)
{
    if (!initialized_) return local_failure(EINVAL);
    if (needs_stop_) return local_failure(EBUSY);
    auto call = head::qvr_set_ring_layout(id, layout);
    return invoke(fd_, handle_, call);
}

Result QvrModule::enable_position_mode(std::uint32_t mode)
{
    if (!initialized_) return local_failure(EINVAL);
    if (needs_stop_) return local_failure(EBUSY);
    if (mode != 1) return local_failure(EINVAL);
    // AP stub ELF0x3180: method8, one four-byte mode. Service mode2 -> DSP1.
    auto call = head::qvr_tracking_mode(mode);
    return invoke(fd_, handle_, call);
}

Result QvrModule::sensor_orientation_identity()
{
    if (!initialized_) return local_failure(EINVAL);
    if (needs_stop_) return local_failure(EBUSY);
    // version-init-decompile.json: AP stub ELF0x32fc, counts + two i32 arrays.
    // start-contract.json: QVRServiceConfiguration::GetSensorOrientation.
    head::Invocation call{0x0e030000,
                          {head::Bytes(8), head::Bytes(12), head::Bytes(12)}, {}};
    word(call.inputs[0], 3);
    word(call.inputs[0], 3, 4);
    for (unsigned i = 0; i < 3; ++i) {
        word(call.inputs[1], i, 4 * i);
        word(call.inputs[2], 1, 4 * i);
    }
    return invoke(fd_, handle_, call);
}

Result QvrModule::start(std::string_view config6, std::string_view calibration, float sensor_frequency_hz)
{
    if (!initialized_) return local_failure(EINVAL);
    if (needs_stop_) return local_failure(EBUSY);
    if (config6.empty() || calibration.empty() || config6.find('\n') != std::string_view::npos)
        return local_failure(EINVAL);
    if (!std::isfinite(sensor_frequency_hz) || sensor_frequency_hz <= 0)
        return local_failure(EINVAL);

    // Exact native defaults in start-contract.json, derived from Tracker::Start
    // ELF0xc6ce4 and QVRServiceConfiguration defaults. The flags word is zero;
    // it is separate from DSP tracking mode and SimpleViewer external format.
    const std::vector<head::CameraId64> ids{
        head::camera_id("trackingA"), head::camera_id("trackingB"),
        head::camera_id("ctrl-trackingA"), head::camera_id("ctrl-trackingB")};
    const std::vector<float> prediction{
        0.13f, 0.5f, 0.01f, 0.03f, 1.0f, 1.1f, 1.1f, 5.0f, 0.068f, 0.059f, 1.021f};
    head::Invocation call;
    try {
        // mode1 does not consume this tail frequency (sensor-rate-contract.json).
        // Still declare the actual requested rate instead of a fictitious 1kHz.
        call = head::qvr_start(config6, calibration, {}, 0, ids, prediction, sensor_frequency_hz, 100.0f);
    } catch (const std::invalid_argument &) {
        return local_failure(EINVAL);
    } catch (const std::length_error &) {
        return local_failure(EOVERFLOW);
    }

    // DSP start ELF0x7a3c can reach tracker initialization and clear its stop
    // flag before returning a failure. Treat every submitted start as needing
    // an explicit stop; only success enables sensor notifications.
    needs_stop_ = true;
    stop_attempted_ = false;
    const Result result = invoke(fd_, handle_, call);
    started_ = static_cast<bool>(result);
    return result;
}

Result QvrModule::stop()
{
    if (!initialized_ || !needs_stop_) return local_failure(EINVAL);
    head::Invocation call{head::qvr_method::stop, {}, {}};
    started_ = false;
    stop_attempted_ = true;
    const Result result = invoke(fd_, handle_, call);
    if (result) needs_stop_ = false;
    return result;
}

Result QvrModule::external_sensor_ready()
{
    if (!initialized_ || !started_) return local_failure(EINVAL);
    // DSP ELF0x87a4 -> FUN_000187b0(0), invalidates ring0 header/current64B
    // record and dispatches onSensorData_6Dof when mode==1. This wrapper in the
    // stock DSP returns zero even when the void helper exits without a pose.
    head::Invocation call{head::qvr_method::external_sensor_ready, {}, {}};
    return invoke(fd_, handle_, call);
}

Result QvrModule::camera_frame(std::uint64_t aligned_capture_time_ns,
                              const head::Bytes &tracking_pair, const head::Bytes &controller_pair,
                              std::int32_t frame_number, std::int32_t aux_frame_number)
{
    if (!initialized_ || !started_) return local_failure(EINVAL);
    constexpr std::size_t pair_bytes = 1280u * 480u;
    if (!aligned_capture_time_ns || tracking_pair.size() != pair_bytes || controller_pair.size() != pair_bytes)
        return local_failure(EINVAL);
    const std::vector<head::CameraId64> ids{
        head::camera_id("trackingA"), head::camera_id("trackingB"),
        head::camera_id("ctrl-trackingA"), head::camera_id("ctrl-trackingB")};
    auto call = head::qvr_camera_frame(1280, 480, aligned_capture_time_ns, tracking_pair,
                                      controller_pair, ids, frame_number, aux_frame_number);
    return invoke(fd_, handle_, call);
}

Result QvrModule::camera_frame_from_sof(std::uint64_t sof_boottime_ns, std::uint64_t effective_exposure_ns,
                                       const head::Bytes &tracking_pair, const head::Bytes &controller_pair,
                                       std::int32_t frame_number, std::int32_t aux_frame_number)
{
    camera::StockY8Time time;
    if (!camera::stock_a081_y8_time(sof_boottime_ns, effective_exposure_ns, time))
        return local_failure(EINVAL);
    return camera_frame(time.submit_boottime_ns, tracking_pair, controller_pair, frame_number, aux_frame_number);
}

Result QvrModule::qtimer_ticks(std::uint64_t &out)
{
    if (!open_) return local_failure(EBADF);
    // AP stub ELF0x31bc; DSP ELF0x8730 calls qurt_sysclock_get_hw_ticks.
    head::Invocation call{head::qvr_method::qtimer, {}, {head::Bytes(8)}};
    const Result result = invoke(fd_, handle_, call);
    if (result) {
        std::uint64_t ticks = 0;
        for (unsigned i = 0; i < 8; ++i)
            ticks |= static_cast<std::uint64_t>(call.outputs[0][i]) << (8 * i);
        out = ticks;
    }
    return result;
}

Result QvrModule::close(std::string &message)
{
    if (!open_) return local_failure(EBADF);
    if (initialized_) return local_failure(EBUSY);
    char error[1024]{};
    std::uint32_t loader_error = 0;
    errno = 0;
    const int rc = remotectl_close(fd_, handle_, sizeof(error), error, &loader_error);
    const Result transport = result_from_rc(rc);
    open_ = false;
    message.assign(error, strnlen(error, sizeof(error)));
    if (!transport) return transport;
    if (loader_error != 0) {
        return {0, 0, loader_error};
    }
    return {};
}

int inherited_session_fd() { return hexagonrpc_fd_from_env(); }

std::string describe(const Result &result)
{
    if (result.system_errno != 0) return std::strerror(result.system_errno);
    if (result.loader_error_word != 0) return "remote loader error (see detail)";
    return hexagonrpc_strerror(result.rc);
}
} // namespace a081::stock
