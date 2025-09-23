#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include "I2C_Handle.h"


namespace i2c {

    static std::string dev_path_for_bus(int bus) {
        return "/dev/i2c-" + std::to_string(bus);
    }

    I2cHandle::I2cHandle(int bus) : bus_(bus) {
        const std::string dev = dev_path_for_bus(bus_);
        fd_ = ::open(dev.c_str(), O_RDWR | O_CLOEXEC);
        if (fd_ < 0) {
            throw std::runtime_error("I2cHandle: konnte Bus nicht öffnen: " + dev);
        }
    }

    I2cHandle::~I2cHandle() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    I2cHandle::I2cHandle(I2cHandle&& other) noexcept {
        bus_ = other.bus_;
        fd_ = other.fd_;
        other.fd_ = -1;
    }

    I2cHandle& I2cHandle::operator=(I2cHandle&& other) noexcept {
        if (this != &other) {
            if (fd_ >= 0) ::close(fd_);
            bus_ = other.bus_;
            fd_  = other.fd_;
            other.fd_ = -1;
        }
        return *this;
    }

} // namespace i2c