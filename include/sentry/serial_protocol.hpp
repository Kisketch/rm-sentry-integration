// Old sentry serial protocol, kept byte-compatible so the new vision core can
// drop into an existing MCU without a protocol rewrite.
//
// MCU -> vision (17 bytes):  AA BB [yaw i32][pitch i32][roll i32] color buff BB
// vision -> MCU (21 bytes):  AA BB [yaw i32][pitch i32][yaw_vel i32][ui_x i32]
//                            [ui_y i32] found fire BB
// All integers are big-endian; angles are 1e-4 degrees.
#pragma once

#include <array>
#include <cstdint>

#include "sentry/sentry.hpp"

namespace sentry {

struct AttitudeFrame {
    double stamp = 0.0;   // seconds, local monotonic clock
    double yaw = 0.0;     // rad
    double pitch = 0.0;   // rad
    double roll = 0.0;    // rad
    int color = 0;        // 0 blue, 1 red (MCU convention)
    int armor_or_buff = 1;
};

constexpr std::size_t kAttitudeSize = 17;
constexpr std::size_t kControlSize = 21;

// Returns false for a bad header / short buffer.
bool parseAttitude(const uint8_t* data, std::size_t size, AttitudeFrame& out);

// Pack a SerialCommand.  `ui_x` is the fourth int32 (the old yaw-rate/UI slot);
// the MCU protocol only has four int32 payloads.
std::array<uint8_t, kControlSize> packControl(const SerialCommand& cmd, int32_t ui_x = 0);

}  // namespace sentry
