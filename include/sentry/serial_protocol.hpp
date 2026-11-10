// Old sentry serial protocol, kept byte-compatible so the new vision core can
// drop into an existing MCU without a protocol rewrite.
//
// MCU -> vision (17 bytes):  AA BB [crc/pad] [yaw i32][pitch i32][roll i32] color buff
//                            (bytes 0-1 header, 2 spare/CRC, 3-14 angles,
//                             15 colour, 16 armor_or_buff; no tail byte)
// vision -> MCU (21 bytes):  AA BB [yaw i32][pitch i32][yaw_vel i32][ui_x i32]
//                            found fire BB   (bytes 2-17 payload, 18, 19, tail 20)
// All integers are big-endian; angles are 1e-4 degrees.
//
// This header is deliberately free of OpenCV / vision includes so the MCU-side
// tooling and the serial driver can use it on their own.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace sentry {

// Command payload of the 21-byte control frame.  yaw/pitch/yaw_vel are 1e-4
// degrees (rad/s for yaw_vel scaled the same way), exactly like the old
// AutoArmor::Transmit.  Build it from an AimCommand with sentry::toSerial().
struct SerialCommand {
    int32_t yaw = 0;
    int32_t pitch = 0;
    int32_t yaw_vel = 0;
    uint8_t found = 0;
    uint8_t fire = 0;
};

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

// --- Optional integrity extension (opt-in, byte-compatible) ---
// Byte 2 of the attitude frame is unused in the legacy protocol.  A sender
// supporting the extension stores a CRC-8/SMBUS (poly 0x07) of the payload
// bytes 3..16 there.  Legacy receivers ignore byte 2, and parseAttitude
// ignores it too, so the extension only takes effect where a receiver
// explicitly requires it (SerialPort's require_crc).  Enabling it needs the
// MCU firmware to fill byte 2; until then run with it off.
// The control frame has no spare byte, so it cannot carry a CRC without a
// length change; that stays a future protocol-v2 decision.
uint8_t crc8(const uint8_t* data, std::size_t size);
uint8_t attitudeCrc(const uint8_t* frame);  // frame must hold kAttitudeSize bytes
bool parseAttitudeCrc(const uint8_t* data, std::size_t size, AttitudeFrame& out);
// Reference packer (the vision side never sends attitude frames; this is
// the format definition for the MCU side and for tests).
std::array<uint8_t, kAttitudeSize> packAttitude(const AttitudeFrame& f, bool with_crc);

// Pack a SerialCommand.  `ui_x` is the fourth int32 (the old yaw-rate/UI slot);
// the MCU protocol only has four int32 payloads.
std::array<uint8_t, kControlSize> packControl(const SerialCommand& cmd, int32_t ui_x = 0);

}  // namespace sentry
