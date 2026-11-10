// Minimal Linux serial transport for the old 17-byte attitude / 21-byte control
// protocol.  The vision side is transport-agnostic; this class is only used by
// the robot executable.
//
// Attitude timing (G-02, docs/timestamp_design.md in rm-autoaim): the 17-byte
// MCU frame carries no timestamp, so each frame is stamped on host receipt
// minus `stamp_delay_s` - an estimate of the MCU-sample -> host-receipt lag
// (wire time alone for 17 bytes at 115200 baud is ~1.5 ms).  Samples are kept
// as quaternions in an autoaim::PoseBuffer and query() interpolates with SLERP
// at the requested stamp.  The euler convention matches
// CameraModel::setGimbalAttitude: R = Rz(yaw) * Ry(-pitch) * Rx(roll),
// positive pitch looking down.
//
// query() returns false when the stamp is not bracketed by samples or the
// bracketing gap exceeds `max_gap_s` (serial dropout).  That is deliberate:
// the previous implementation clamped to the newest sample however stale it
// was, which silently fed the vision core a wrong attitude.  Callers must
// treat false as "no attitude for this frame".
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "autoaim/pose_buffer.hpp"
#include "sentry/serial_protocol.hpp"

namespace io {

class SerialPort {
public:
    // require_crc: reject attitude frames whose byte-2 CRC-8 does not check
    // out.  Only enable once the MCU firmware fills byte 2 (see
    // serial_protocol.hpp); the legacy sender leaves it 0.
    SerialPort(std::string device, int baud = 115200, double stamp_delay_s = 0.0015,
               double max_gap_s = 0.05, bool require_crc = false);
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool ok() const { return fd_ >= 0; }
    void close();

    // Blocking write of a 21-byte frame.  Reconnects once on failure.
    bool send(const std::array<uint8_t, sentry::kControlSize>& frame);

    // Attitude interpolated (SLERP) at `stamp` (seconds, steady clock).
    // False when no bracketing samples exist or the gap is too large.
    bool query(double stamp, sentry::AttitudeFrame& out);

    // Diagnostics for the G-02 bench checks.
    std::uint64_t receivedFrames() const { return received_.load(); }
    std::uint64_t failedQueries() const { return failed_queries_.load(); }
    std::uint64_t crcFailures() const { return crc_failures_.load(); }
    std::size_t samplesBuffered() const { return poses_.size(); }

private:
    bool openDevice();
    void readerLoop();
    void reconnect();

    std::string device_;
    int baud_;
    int fd_ = -1;
    std::atomic<bool> quit_{false};
    std::thread thread_;

    double stamp_delay_s_;
    autoaim::PoseBuffer poses_;

    // color / armor_or_buff are not interpolated; keep the recent values with
    // their stamps so query() can report the newest one at or before `stamp`.
    struct Meta {
        double stamp = 0.0;
        int color = 0;
        int armor_or_buff = 1;
    };
    std::mutex mutex_;
    std::deque<Meta> meta_;

    std::atomic<std::uint64_t> received_{0};
    std::atomic<std::uint64_t> failed_queries_{0};
    std::atomic<std::uint64_t> crc_failures_{0};
    bool require_crc_ = false;
};

}  // namespace io
