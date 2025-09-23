#pragma once
#include <cstdint>
#include <string>

namespace i2c {

class I2cHandle {
public:
    explicit I2cHandle(int bus);
    ~I2cHandle();

    // Kopieren verbieten, Verschieben erlauben
    I2cHandle(const I2cHandle&) = delete;
    I2cHandle& operator=(const I2cHandle&) = delete;
    I2cHandle(I2cHandle&&) noexcept;
    I2cHandle& operator=(I2cHandle&&) noexcept;

    int bus() const { return bus_; }
    int fd()  const { return fd_;  }

private:
    int bus_ = -1;
    int fd_  = -1;
};

} // namespace i2c
