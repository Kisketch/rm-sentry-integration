#include "sentry/serial_protocol.hpp"

#include <cmath>

namespace sentry {
namespace {

int32_t readI32(const uint8_t* p) {
    return static_cast<int32_t>((static_cast<uint32_t>(p[0]) << 24) |
                                (static_cast<uint32_t>(p[1]) << 16) |
                                (static_cast<uint32_t>(p[2]) << 8) | p[3]);
}

void writeI32(uint8_t* p, int32_t v) {
    p[0] = static_cast<uint8_t>((v >> 24) & 0xff);
    p[1] = static_cast<uint8_t>((v >> 16) & 0xff);
    p[2] = static_cast<uint8_t>((v >> 8) & 0xff);
    p[3] = static_cast<uint8_t>(v & 0xff);
}

double wrapPi(double a) {
    constexpr double pi = 3.14159265358979323846;
    while (a > pi) a -= 2 * pi;
    while (a <= -pi) a += 2 * pi;
    return a;
}

}  // namespace

// Old MCU frame: AA BB [pad] yaw pitch roll color buff [BB]
// Bytes 3..6, 7..10, 11..14 are big-endian 1e-4 degrees; byte 15 color,
// byte 16 armor_or_buff, byte 2 unused.
bool parseAttitude(const uint8_t* data, std::size_t size, AttitudeFrame& out) {
    if (!data || size < kAttitudeSize || data[0] != 0xAA || data[1] != 0xBB) return false;
    constexpr double scale = 3.14159265358979323846 / 180.0 / 10000.0;
    out.yaw = wrapPi(readI32(data + 3) * scale);
    out.pitch = readI32(data + 7) * scale;
    // The old MCU sends roll - pi; keep the same convention.
    out.roll = wrapPi(readI32(data + 11) * scale - 3.14159265358979323846);
    out.color = data[15];
    out.armor_or_buff = data[16];
    return true;
}

uint8_t crc8(const uint8_t* data, std::size_t size) {
    uint8_t crc = 0;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
    }
    return crc;
}

uint8_t attitudeCrc(const uint8_t* frame) { return crc8(frame + 3, kAttitudeSize - 3); }

bool parseAttitudeCrc(const uint8_t* data, std::size_t size, AttitudeFrame& out) {
    if (!data || size < kAttitudeSize) return false;
    if (data[2] != attitudeCrc(data)) return false;
    return parseAttitude(data, size, out);
}

std::array<uint8_t, kAttitudeSize> packAttitude(const AttitudeFrame& f, bool with_crc) {
    std::array<uint8_t, kAttitudeSize> b{};
    b[0] = 0xAA;
    b[1] = 0xBB;
    constexpr double scale = 180.0 / 3.14159265358979323846 * 10000.0;
    writeI32(b.data() + 3, static_cast<int32_t>(std::lround(wrapPi(f.yaw) * scale)));
    writeI32(b.data() + 7, static_cast<int32_t>(std::lround(f.pitch * scale)));
    // Inverse of the parser's convention: the wire carries roll + pi.
    writeI32(b.data() + 11, static_cast<int32_t>(std::lround(wrapPi(f.roll + 3.14159265358979323846) * scale)));
    b[15] = static_cast<uint8_t>(f.color);
    b[16] = static_cast<uint8_t>(f.armor_or_buff);
    b[2] = with_crc ? attitudeCrc(b.data()) : 0;
    return b;
}

// 21-byte control frame: AA BB [yaw][pitch][yaw_vel][ui_x] found fire BB
// The old AutoArmor::Transmit put the yaw rate into the ui_x slot; keeping that
// mapping means the MCU does not need a new parser.
std::array<uint8_t, kControlSize> packControl(const SerialCommand& cmd, int32_t ui_x) {
    std::array<uint8_t, kControlSize> b{};
    b[0] = 0xAA;
    b[1] = 0xBB;
    writeI32(b.data() + 2, cmd.yaw);
    writeI32(b.data() + 6, cmd.pitch);
    writeI32(b.data() + 10, cmd.yaw_vel);
    writeI32(b.data() + 14, ui_x);
    b[18] = cmd.found;
    b[19] = cmd.fire;
    b[20] = 0xBB;
    return b;
}

}  // namespace sentry
