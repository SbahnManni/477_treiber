#pragma once
#include <cstdint>
#include <mutex>

struct DeviceEvent;
namespace i2c {

  class I2C_Manager;
  

  class AnalogOut {
    public:
        AnalogOut(i2c::I2C_Manager& i2c, uint8_t i2c_addr7);
        ~AnalogOut();

        bool begin();
        void writeChannel(uint8_t channel, int value_0_to_1000);
        
        // Handler, den der I2C-Manager Aufruft
        void handleOut(const DeviceEvent& e);
        
    private:
        i2c::I2C_Manager& i2c_;
        uint8_t addr_;
        std::mutex mtx_;
        static inline uint16_t clamp1000(int v) {
            if (v < 0) return 0;
            if (v > 1000) return 1000;
            return static_cast<uint16_t>(v);
        }
  };

} // namespace i2c
