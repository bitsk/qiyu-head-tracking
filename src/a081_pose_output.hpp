// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace a081::stock::pose_output {

// Decoder only. The caller must supply an immutable, CPU-coherent copy and
// establish its capture protocol separately. Existing full-ring snapshots are
// captured after successful stop. A repeated equal header index is
// NOT a seqlock: the DSP publishes a wrapping slot index without a generation.
// This header does not synchronize DMA, read devices, predict, recenter, change
// coordinate systems, or establish OpenXR validity. See pose-output-contract.json.
inline constexpr std::size_t header_bytes = 64;
inline constexpr std::size_t slot_count = 80;
inline constexpr std::size_t pose_slot_bytes = 192;
inline constexpr std::size_t dl_slot_bytes = 128;
inline constexpr std::size_t pose_ring_bytes = header_bytes + slot_count * pose_slot_bytes;
inline constexpr std::size_t dl_ring_bytes = header_bytes + slot_count * dl_slot_bytes;

enum class DecodeStatus {
    ok = 0,
    null_input,
    truncated_snapshot,
    invalid_published_index,
};

enum class StockTrackingState : std::uint16_t {
    uninitialized = 0,
    relocating = 1,
    suspended = 2,
    tracking = 4,
    fatal_error = 8,
};

struct PoseSnapshot {
    // Parsing or a changing timestamp does not establish a valid pose.
    static constexpr bool valid = false;
    std::uint32_t published_index{};
    std::array<float, 4> rotation_xyzw{};
    std::array<float, 3> translation{};
    // Producer sensor timestamp. External-IMU mode preserves supplied gyro
    // BOOTTIME ns; other transports may use a different source time domain.
    std::uint64_t timestamp_ns{};
    std::array<float, 3> angular_s{};
    std::array<float, 3> angular_b{};
    std::array<float, 3> angular_bdt{};
    std::array<float, 3> angular_bdt2{};
    std::uint16_t tracking_state{}; // Raw stock bitmask, not XrSpaceLocationFlags.
    std::uint16_t warning_flags{};
    std::uint32_t magnetometer_flags_raw{};
    float pose_quality{}; // Stock binary quality: 0 bad, 1 good; preserve any value.
    float sensor_quality{};
    float camera_quality{};
    std::array<float, 3> translation_s{};
    std::array<float, 3> translation_b{};
};

struct DlPredictionSnapshot {
    static constexpr bool valid = false;
    std::uint32_t published_index{};
    // Four fixed six-float groups. Axis names, polynomial basis/time units,
    // and conversion to a predicted pose are not established by this decoder.
    std::array<std::array<float, 6>, 4> coefficient_groups{};
    std::array<std::uint8_t, 4> coefficient_counts{};
    std::uint64_t timestamp_ns{};
    // DSP writes 1 after a successful tracker update, including potentially
    // uninitialized/poor-quality poses. This is not a tracking-valid flag.
    std::uint8_t producer_marker{};
};

namespace detail {
inline std::uint16_t u16(const std::uint8_t *p) noexcept
{
    return static_cast<std::uint16_t>(p[0]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(p[1]) << 8);
}
inline std::uint32_t u32(const std::uint8_t *p) noexcept
{
    return static_cast<std::uint32_t>(p[0]) |
           (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) |
           (static_cast<std::uint32_t>(p[3]) << 24);
}
inline std::uint64_t u64(const std::uint8_t *p) noexcept
{
    return static_cast<std::uint64_t>(u32(p)) |
           (static_cast<std::uint64_t>(u32(p + 4)) << 32);
}
inline float f32(const std::uint8_t *p) noexcept
{
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
                  "A081 wire floats require IEEE-754 binary32");
    const std::uint32_t bits = u32(p);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}
template <std::size_t N>
inline void floats(const std::uint8_t *p, std::array<float, N> &out) noexcept
{
    for (std::size_t i = 0; i < N; ++i) out[i] = f32(p + i * 4);
}
inline DecodeStatus latest_slot(const void *snapshot, std::size_t bytes,
                                std::size_t element_bytes, std::uint32_t &index,
                                const std::uint8_t *&slot) noexcept
{
    if (snapshot == nullptr) return DecodeStatus::null_input;
    if (bytes < header_bytes + slot_count * element_bytes)
        return DecodeStatus::truncated_snapshot;
    const auto *base = static_cast<const std::uint8_t *>(snapshot);
    const std::uint32_t published = u32(base);
    if (published >= slot_count) return DecodeStatus::invalid_published_index;
    index = published;
    slot = base + header_bytes + published * element_bytes;
    return DecodeStatus::ok;
}
} // namespace detail

// On error, out is unchanged. A zero-initialized ring can decode index 0 and
// timestamp 0: successful decoding is a layout result, not proof of publication.
inline DecodeStatus decode_pose_record(const void *record, std::size_t bytes,
                                      std::uint32_t published_index, PoseSnapshot &out) noexcept
{
    if (!record) return DecodeStatus::null_input;
    if (bytes < pose_slot_bytes) return DecodeStatus::truncated_snapshot;
    if (published_index >= slot_count) return DecodeStatus::invalid_published_index;
    PoseSnapshot parsed{};
    parsed.published_index = published_index;
    const auto *p = static_cast<const std::uint8_t *>(record);
    detail::floats(p, parsed.rotation_xyzw);
    detail::floats(p + 16, parsed.translation);
    parsed.timestamp_ns = detail::u64(p + 32);
    detail::floats(p + 48, parsed.angular_s);
    detail::floats(p + 64, parsed.angular_b);
    detail::floats(p + 80, parsed.angular_bdt);
    detail::floats(p + 96, parsed.angular_bdt2);
    parsed.tracking_state = detail::u16(p + 108);
    parsed.warning_flags = detail::u16(p + 110);
    parsed.magnetometer_flags_raw = detail::u32(p + 112);
    parsed.pose_quality = detail::f32(p + 116);
    parsed.sensor_quality = detail::f32(p + 120);
    parsed.camera_quality = detail::f32(p + 124);
    detail::floats(p + 128, parsed.translation_s);
    detail::floats(p + 144, parsed.translation_b);
    out = parsed;
    return DecodeStatus::ok;
}

inline DecodeStatus decode_latest_pose_snapshot(const void *snapshot, std::size_t bytes,
                                                PoseSnapshot &out) noexcept
{
    std::uint32_t index = 0;
    const std::uint8_t *slot = nullptr;
    const auto status = detail::latest_slot(snapshot, bytes, pose_slot_bytes, index, slot);
    if (status != DecodeStatus::ok) return status;
    return decode_pose_record(slot, pose_slot_bytes, index, out);
}

inline DecodeStatus decode_latest_dl_prediction_snapshot(const void *snapshot,
                                                         std::size_t bytes,
                                                         DlPredictionSnapshot &out) noexcept
{
    DlPredictionSnapshot parsed{};
    const std::uint8_t *p = nullptr;
    const auto status = detail::latest_slot(snapshot, bytes, dl_slot_bytes,
                                            parsed.published_index, p);
    if (status != DecodeStatus::ok) return status;
    for (std::size_t group = 0; group < 4; ++group) {
        detail::floats(p + group * 24, parsed.coefficient_groups[group]);
        parsed.coefficient_counts[group] = p[96 + group];
    }
    parsed.timestamp_ns = detail::u64(p + 104);
    parsed.producer_marker = p[112];
    out = parsed;
    return DecodeStatus::ok;
}

} // namespace a081::stock::pose_output
