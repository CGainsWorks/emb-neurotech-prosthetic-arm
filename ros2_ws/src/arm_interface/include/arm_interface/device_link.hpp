// The byte transport between the host and the board.
//
// Two implementations. The simulated one runs the board in this process and is
// what every test and the demo use. The serial one opens a real port and has
// never been connected to anything, because no board exists; it is here so
// that the day one does, the only untested code is the part that cannot be
// tested any other way.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "arm_interface/simulated_device.hpp"

namespace arm_interface {

class DeviceLink {
  public:
    virtual ~DeviceLink() = default;

    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool is_open() const = 0;

    // Send bytes towards the board.
    virtual bool write(const uint8_t* data, std::size_t length) = 0;

    // Everything that has arrived since the last call. Never blocks.
    virtual std::vector<uint8_t> read() = 0;

    // Let time pass. The simulated link steps its device; the serial one does
    // nothing, because a real board keeps its own time.
    virtual void advance(double seconds) = 0;

    virtual std::string description() const = 0;
};

// The board, in this process, driven by the same bytes a cable would carry.
class SimulatedLink : public DeviceLink {
  public:
    SimulatedLink(const ak::ArmModel& model, const SimulationConfig& config);

    bool open() override { return true; }
    void close() override {}
    bool is_open() const override { return true; }

    bool write(const uint8_t* data, std::size_t length) override;
    std::vector<uint8_t> read() override;
    void advance(double seconds) override;
    std::string description() const override { return "simulated device"; }

    SimulatedDevice& device() { return device_; }
    const SimulatedDevice& device() const { return device_; }

  private:
    SimulatedDevice device_;
    double step_s_;
    double carry_{0.0};
};

// A POSIX serial port. Never opened against hardware.
class SerialLink : public DeviceLink {
  public:
    SerialLink(std::string port, uint32_t baud);
    ~SerialLink() override;

    bool open() override;
    void close() override;
    bool is_open() const override { return fd_ >= 0; }

    bool write(const uint8_t* data, std::size_t length) override;
    std::vector<uint8_t> read() override;
    void advance(double) override {}
    std::string description() const override;

    const std::string& last_error() const { return last_error_; }

  private:
    std::string port_;
    uint32_t baud_;
    int fd_{-1};
    std::string last_error_;
};

std::unique_ptr<DeviceLink> make_link(const std::string& backend, const std::string& port,
                                      uint32_t baud, const ak::ArmModel& model,
                                      const SimulationConfig& config);

}  // namespace arm_interface
