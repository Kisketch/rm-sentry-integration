#include "io/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>

namespace io {
namespace {

speed_t baudToSpeed(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return B115200;
    }
}

double nowSeconds() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// Extract (yaw, pitch, roll) in the setGimbalAttitude convention
// R = Rz(yaw) * Ry(-pitch) * Rx(roll) from a rotation matrix built that way.
void yawPitchRollFromMat(const cv::Matx33d& R, double& yaw, double& pitch, double& roll) {
    yaw = std::atan2(R(1, 0), R(0, 0));
    const double theta = std::atan2(-R(2, 0), std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
    pitch = -theta;  // the Ry angle is -pitch in this convention
    roll = std::atan2(R(2, 1), R(2, 2));
}

}  // namespace

SerialPort::SerialPort(std::string device, int baud, double stamp_delay_s, double max_gap_s,
                       bool require_crc)
    : device_(std::move(device)),
      baud_(baud),
      stamp_delay_s_(stamp_delay_s),
      poses_(512, max_gap_s),
      require_crc_(require_crc) {
    if (openDevice()) thread_ = std::thread(&SerialPort::readerLoop, this);
}

SerialPort::~SerialPort() {
    quit_ = true;
    if (thread_.joinable()) thread_.join();
    close();
}

bool SerialPort::openDevice() {
    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;
    termios tio{};
    if (tcgetattr(fd_, &tio) < 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    cfsetispeed(&tio, baudToSpeed(baud_));
    cfsetospeed(&tio, baudToSpeed(baud_));
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8 | CLOCAL | CREAD;
#ifdef CRTSCTS
    tio.c_cflag &= ~CRTSCTS;
#endif
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_oflag &= ~OPOST;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    tcsetattr(fd_, TCSANOW, &tio);
    tcflush(fd_, TCIOFLUSH);
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void SerialPort::reconnect() {
    close();
    for (int i = 0; i < 20 && !quit_; ++i) {
        if (openDevice()) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void SerialPort::readerLoop() {
    std::array<uint8_t, 128> buf{};
    std::size_t used = 0;
    while (!quit_) {
        if (fd_ < 0) {
            reconnect();
            continue;
        }
        pollfd pfd{fd_, POLLIN, 0};
        const int pr = poll(&pfd, 1, 20);
        if (pr <= 0) continue;
        const ssize_t n = ::read(fd_, buf.data() + used, buf.size() - used);
        if (n <= 0) {
            reconnect();
            continue;
        }
        used += static_cast<std::size_t>(n);
        std::size_t consumed = 0;
        while (used - consumed >= sentry::kAttitudeSize) {
            if (buf[consumed] != 0xAA || buf[consumed + 1] != 0xBB) {
                ++consumed;
                continue;
            }
            sentry::AttitudeFrame a;
            const bool parsed =
                require_crc_
                    ? sentry::parseAttitudeCrc(buf.data() + consumed, sentry::kAttitudeSize, a)
                    : sentry::parseAttitude(buf.data() + consumed, sentry::kAttitudeSize, a);
            if (!parsed && require_crc_) ++crc_failures_;
            if (parsed) {
                // The protocol carries no MCU timestamp: stamp on receipt,
                // minus the estimated sample->receipt lag.  The quaternion
                // uses the setGimbalAttitude convention (Ry angle = -pitch).
                a.stamp = nowSeconds() - stamp_delay_s_;
                poses_.push({a.stamp, autoaim::quatFromYawPitchRoll(a.yaw, -a.pitch, a.roll)});
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    meta_.push_back({a.stamp, a.color, a.armor_or_buff});
                    while (meta_.size() > 128) meta_.pop_front();
                }
                ++received_;
                consumed += sentry::kAttitudeSize;
            } else {
                ++consumed;
            }
        }
        if (consumed > 0) {
            std::memmove(buf.data(), buf.data() + consumed, used - consumed);
            used -= consumed;
        }
        if (used == buf.size()) used = 0;
    }
}

bool SerialPort::send(const std::array<uint8_t, sentry::kControlSize>& frame) {
    if (fd_ < 0) {
        reconnect();
        if (fd_ < 0) return false;
    }
    const ssize_t n = ::write(fd_, frame.data(), frame.size());
    if (n != static_cast<ssize_t>(frame.size())) {
        reconnect();
        return false;
    }
    return true;
}

bool SerialPort::query(double stamp, sentry::AttitudeFrame& out) {
    autoaim::Quat q;
    if (!poses_.query(stamp, q)) {
        ++failed_queries_;
        return false;
    }
    yawPitchRollFromMat(autoaim::quatToMat(q), out.yaw, out.pitch, out.roll);
    out.stamp = stamp;
    out.color = 0;
    out.armor_or_buff = 1;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Newest metadata at or before the stamp; the earliest if the stamp
        // precedes all of them (can only happen right at startup).
        for (const auto& m : meta_) {
            if (m.stamp <= stamp) {
                out.color = m.color;
                out.armor_or_buff = m.armor_or_buff;
            } else {
                break;
            }
        }
        if (!meta_.empty() && stamp < meta_.front().stamp) {
            out.color = meta_.front().color;
            out.armor_or_buff = meta_.front().armor_or_buff;
        }
    }
    return true;
}

}  // namespace io
