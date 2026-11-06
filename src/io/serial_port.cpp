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

}  // namespace

SerialPort::SerialPort(std::string device, int baud) : device_(std::move(device)), baud_(baud) {
    if (openDevice()) thread_ = std::thread(&SerialPort::readerLoop, this);
}

SerialPort::~SerialPort() {
    quit_ = true;
    cv_.notify_all();
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
            if (sentry::parseAttitude(buf.data() + consumed, sentry::kAttitudeSize, a)) {
                a.stamp = nowSeconds();
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    queue_.push_back(a);
                    while (queue_.size() > 64) queue_.pop_front();
                }
                cv_.notify_all();
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
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) return false;
    while (queue_.size() > 2 && queue_[1].stamp < stamp) queue_.pop_front();
    if (queue_.size() == 1 || stamp <= queue_.front().stamp) {
        out = queue_.front();
        return true;
    }
    const auto& a = queue_[0];
    const auto& b = queue_[1];
    const double span = b.stamp - a.stamp;
    const double u = span > 1e-9 ? std::clamp((stamp - a.stamp) / span, 0.0, 1.0) : 1.0;
    out = a;
    out.stamp = stamp;
    out.yaw = a.yaw + u * std::remainder(b.yaw - a.yaw, 2.0 * M_PI);
    out.pitch = a.pitch + u * (b.pitch - a.pitch);
    out.roll = a.roll + u * (b.roll - a.roll);
    out.color = b.color;
    out.armor_or_buff = b.armor_or_buff;
    return true;
}

}  // namespace io
