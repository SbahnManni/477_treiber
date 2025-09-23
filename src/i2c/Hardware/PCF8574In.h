#pragma once
#include <cstdint>
#include <functional>
#include <array>
#include <chrono>

namespace i2c {

  class I2C_Manager;

  class PCF8574In {
    public:
        PCF8574In(i2c::I2C_Manager& i2c, uint8_t i2c_addr7);
        ~PCF8574In();

        uint8_t read_port() const;
        
        void setOnChange(std::function<void(uint8_t)> onChange);
        // Wird vom Manager aufgerufen, wenn ein Edge kam:
        void onInterrupt();
        void setDebounceMs(unsigned ms) { debounce_ = std::chrono::milliseconds(ms); }
    private:
        i2c::I2C_Manager& i2c_;
        uint8_t           addr_;
        uint8_t           mask_;
        std::function<void(uint8_t)> on_change_;
        bool initialized_;
        uint8_t last_;
        std::array<std::chrono::steady_clock::time_point, 8> last_edge_{};
        std::chrono::milliseconds debounce_{10}; // Default 10 ms
  };

} // namespace i2c
