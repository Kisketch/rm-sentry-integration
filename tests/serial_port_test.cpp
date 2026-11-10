// End-to-end test of io::SerialPort over a PTY pair: attitude bytes in,
// SLERP-interpolated attitude out.  Verifies the wiring the G-02 alignment
// depends on - parser scale and roll offset, receipt-time stamping, bracketing
// and gap rejection - without serial hardware.  Linux-only, like the class.
#include <pty.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <thread>

#include "io/serial_port.hpp"

namespace {

int failures = 0;

#define CHECK(cond, ...)                                       \
    do {                                                       \
        if (!(cond)) {                                         \
            ++failures;                                        \
            std::printf("FAIL %d: ", __LINE__);                \
            std::printf(__VA_ARGS__);                          \
            std::printf("\n");                                 \
        }                                                      \
    } while (0)

double nowSeconds() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

void writeI32(uint8_t* p, int32_t v) {
    p[0] = static_cast<uint8_t>((v >> 24) & 0xff);
    p[1] = static_cast<uint8_t>((v >> 16) & 0xff);
    p[2] = static_cast<uint8_t>((v >> 8) & 0xff);
    p[3] = static_cast<uint8_t>(v & 0xff);
}

int32_t encodeAngle(double rad) {
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    return static_cast<int32_t>(std::lround(rad * kRadToDeg * 10000.0));
}

// One 17-byte MCU attitude frame, exactly as parseAttitude expects it
// (roll is sent as roll + pi; see serial_protocol.cpp).
void writeAttitude(int fd, double yaw, double pitch, double roll, int color) {
    std::array<uint8_t, sentry::kAttitudeSize> b{};
    b[0] = 0xAA;
    b[1] = 0xBB;
    writeI32(b.data() + 3, encodeAngle(yaw));
    writeI32(b.data() + 7, encodeAngle(pitch));
    writeI32(b.data() + 11, encodeAngle(roll + 3.14159265358979323846));
    b[15] = static_cast<uint8_t>(color);
    b[16] = 1;
    const ssize_t n = ::write(fd, b.data(), b.size());
    if (n != static_cast<ssize_t>(b.size())) std::printf("WARN: short write %zd\n", n);
}

}  // namespace

int main() {
    int master = -1, slave = -1;
    char name[128] = {0};
    if (openpty(&master, &slave, name, nullptr, nullptr) != 0) {
        std::printf("SKIP: openpty failed\n");
        return 0;
    }

    io::SerialPort port(name, 115200, /*stamp_delay_s=*/0.0, /*max_gap_s=*/0.25);
    CHECK(port.ok(), "serial port opened on %s", name);

    // Constant spin at 2 rad/s, constant pitch/roll, frames 150 ms apart.
    const double t0 = nowSeconds();
    writeAttitude(master, 0.0, 0.1, 0.2, 1);
    sleepMs(150);
    writeAttitude(master, 0.3, 0.1, 0.2, 1);
    sleepMs(150);
    writeAttitude(master, 0.6, 0.1, 0.2, 1);
    sleepMs(200);  // let the reader thread ingest the last frame

    sentry::AttitudeFrame out;
    // Midpoint between frames 2 and 3: yaw must be interpolated (~0.45),
    // neither stale (0.3) nor clamped to the newest (0.6).  The tolerance
    // absorbs the reader thread's receipt lag (poll period 20 ms).
    CHECK(port.query(t0 + 0.225, out), "midpoint query succeeds");
    CHECK(out.yaw > 0.33 && out.yaw < 0.51, "interpolated yaw %.3f", out.yaw);
    CHECK(std::abs(out.pitch - 0.1) < 0.03, "pitch roundtrip %.3f (sign convention)", out.pitch);
    CHECK(std::abs(out.roll - 0.2) < 0.03, "roll roundtrip %.3f (pi offset)", out.roll);
    CHECK(out.color == 1, "color %d", out.color);

    CHECK(!port.query(t0 - 1.0, out), "query before coverage fails");
    CHECK(!port.query(t0 + 10.0, out), "query beyond newest sample fails");

    // A 400 ms silence, then two close frames: a query across the silence
    // must fail (gap > max_gap), not interpolate across the dropout, while a
    // query between the two new frames works again.
    sleepMs(400);
    writeAttitude(master, 0.9, 0.1, 0.2, 1);
    sleepMs(150);
    writeAttitude(master, 0.9, 0.1, 0.2, 1);
    sleepMs(200);
    CHECK(!port.query(t0 + 0.65, out), "query across a dropout gap fails");
    CHECK(port.query(t0 + 0.975, out), "query between post-gap frames succeeds");
    CHECK(out.yaw > 0.84 && out.yaw < 0.96, "yaw after gap %.3f", out.yaw);

    CHECK(port.receivedFrames() >= 5, "received %llu frames",
          static_cast<unsigned long long>(port.receivedFrames()));
    CHECK(port.failedQueries() >= 3, "failed queries counted: %llu",
          static_cast<unsigned long long>(port.failedQueries()));

    ::close(master);
    ::close(slave);

    // Raw-mode regression: a payload byte 0x0D must arrive unchanged.  The old
    // termios setup left ICRNL on, so 0x0D became 0x0A (6.8877 deg read back
    // as 6.8106 deg).  Three frames written in ONE burst must also all be
    // parsed, each with its own (back-dated) stamp.
    {
        int m2 = -1, s2 = -1;
        char name2[128] = {0};
        if (openpty(&m2, &s2, name2, nullptr, nullptr) == 0) {
            io::SerialPort port2(name2, 115200, 0.0, /*max_gap_s=*/0.25, false, /*max_extrap_s=*/0.5);
            CHECK(port2.ok(), "second port opened");
            const double yaw_deg = 68877 / 10000.0;  // raw 0x00010D0D
            const double yaw = yaw_deg * 3.14159265358979323846 / 180.0;
            std::array<uint8_t, 3 * sentry::kAttitudeSize> burst{};
            for (int k = 0; k < 3; ++k) {
                uint8_t* b = burst.data() + k * sentry::kAttitudeSize;
                b[0] = 0xAA, b[1] = 0xBB;
                writeI32(b + 3, 68877);
                writeI32(b + 7, encodeAngle(0.1));
                writeI32(b + 11, encodeAngle(0.2 + 3.14159265358979323846));
                b[15] = 0, b[16] = 1;
            }
            const ssize_t n = ::write(m2, burst.data(), burst.size());
            CHECK(n == static_cast<ssize_t>(burst.size()), "burst write");
            sleepMs(150);
            CHECK(port2.receivedFrames() == 3, "burst frames parsed: %llu",
                  static_cast<unsigned long long>(port2.receivedFrames()));
            CHECK(port2.samplesBuffered() == 3, "distinct stamps per burst frame: %zu", port2.samplesBuffered());
            sentry::AttitudeFrame o2;
            const bool q2 = port2.query(nowSeconds() - 0.1, o2);
            CHECK(q2, "query near the burst (extrapolation enabled)");
            CHECK(q2 && std::abs(o2.yaw - yaw) < 2e-5, "0x0D bytes survive: yaw %.6f deg (want %.4f)",
                  o2.yaw * 180.0 / 3.14159265358979323846, yaw_deg);
            ::close(m2);
            ::close(s2);
        }
    }
    if (!failures) std::printf("SERIAL PORT TESTS PASSED\n");
    return failures ? 1 : 0;
}
