#include "arm_interface/device_link.hpp"

#include <cerrno>
#include <cstring>

#if defined(__unix__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#define ARM_INTERFACE_HAS_SERIAL 1
#else
#define ARM_INTERFACE_HAS_SERIAL 0
#endif

namespace arm_interface {

SimulatedLink::SimulatedLink(const ak::ArmModel& model, const SimulationConfig& config)
    : device_(model, config), step_s_(config.step_s) {}

bool SimulatedLink::write(const uint8_t* data, std::size_t length) {
    device_.receive(data, length);
    return true;
}

std::vector<uint8_t> SimulatedLink::read() { return device_.take_transmitted(); }

void SimulatedLink::advance(double seconds) {
    // Whole steps only, with the remainder carried forward. Rounding to the
    // nearest step instead would make the simulated clock drift against the
    // caller's, slowly, in a way that shows up as a timing result rather than
    // as a bug.
    carry_ += seconds;
    while (carry_ >= step_s_) {
        device_.step();
        carry_ -= step_s_;
    }
}

SerialLink::SerialLink(std::string port, uint32_t baud)
    : port_(std::move(port)), baud_(baud) {}

SerialLink::~SerialLink() { close(); }

std::string SerialLink::description() const {
    return "serial " + port_ + " at " + std::to_string(baud_) + " baud";
}

#if ARM_INTERFACE_HAS_SERIAL

namespace {

speed_t baud_constant(uint32_t baud) {
    switch (baud) {
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return B0;
    }
}

}  // namespace

bool SerialLink::open() {
    close();
    // O_NONBLOCK so that opening a port whose other end asserts no carrier
    // does not hang and so that read() can be polled from a timer.
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        last_error_ = std::string("open ") + port_ + ": " + std::strerror(errno);
        return false;
    }

    termios options{};
    if (tcgetattr(fd_, &options) != 0) {
        last_error_ = std::string("tcgetattr: ") + std::strerror(errno);
        close();
        return false;
    }

    const speed_t speed = baud_constant(baud_);
    if (speed == B0) {
        last_error_ = "unsupported baud rate " + std::to_string(baud_);
        close();
        return false;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // Raw, eight bits, no parity, one stop bit, no flow control. Every one of
    // those matters: the framing carries its own delimiter and CRC and any
    // line discipline that rewrites bytes, expands newlines or interprets
    // XON and XOFF would corrupt frames in a way the CRC would report as
    // link noise.
    cfmakeraw(&options);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~static_cast<tcflag_t>(CSTOPB | PARENB | CRTSCTS);
    options.c_cflag &= ~static_cast<tcflag_t>(CSIZE);
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
        last_error_ = std::string("tcsetattr: ") + std::strerror(errno);
        close();
        return false;
    }
    tcflush(fd_, TCIOFLUSH);
    last_error_.clear();
    return true;
}

void SerialLink::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialLink::write(const uint8_t* data, std::size_t length) {
    if (fd_ < 0) return false;
    std::size_t written = 0;
    while (written < length) {
        const ssize_t n = ::write(fd_, data + written, length - written);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            last_error_ = std::string("write: ") + std::strerror(errno);
            return false;
        }
        written += static_cast<std::size_t>(n);
    }
    return true;
}

std::vector<uint8_t> SerialLink::read() {
    std::vector<uint8_t> out;
    if (fd_ < 0) return out;
    uint8_t buffer[512];
    while (true) {
        const ssize_t n = ::read(fd_, buffer, sizeof(buffer));
        if (n > 0) {
            out.insert(out.end(), buffer, buffer + n);
            if (static_cast<std::size_t>(n) < sizeof(buffer)) break;
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        break;
    }
    return out;
}

#else

bool SerialLink::open() {
    last_error_ = "serial ports are only implemented for POSIX hosts";
    return false;
}
void SerialLink::close() {}
bool SerialLink::write(const uint8_t*, std::size_t) { return false; }
std::vector<uint8_t> SerialLink::read() { return {}; }

#endif

std::unique_ptr<DeviceLink> make_link(const std::string& backend, const std::string& port,
                                      uint32_t baud, const ak::ArmModel& model,
                                      const SimulationConfig& config) {
    if (backend == "serial") return std::make_unique<SerialLink>(port, baud);
    return std::make_unique<SimulatedLink>(model, config);
}

}  // namespace arm_interface
