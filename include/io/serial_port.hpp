// Minimal Linux serial transport for the old 17-byte attitude / 21-byte control
// protocol.  The vision side is transport-agnostic; this class is only used by
// the robot executable.
#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "sentry/serial_protocol.hpp"

namespace io {

class SerialPort {
public:
    SerialPort(std::string device, int baud = 115200);
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool ok() const { return fd_ >= 0; }
    void close();

    // Blocking write of a 21-byte frame.  Reconnects once on failure.
    bool send(const std::array<uint8_t, sentry::kControlSize>& frame);

    // Interpolate the attitude at `stamp` (seconds, steady clock).  Returns
    // false if no frame is available yet.
    bool query(double stamp, sentry::AttitudeFrame& out);

private:
    bool openDevice();
    void readerLoop();
    void reconnect();

    std::string device_;
    int baud_;
    int fd_ = -1;
    std::atomic<bool> quit_{false};
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<sentry::AttitudeFrame> queue_;
};

}  // namespace io
