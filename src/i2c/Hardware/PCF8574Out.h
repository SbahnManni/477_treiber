#pragma once
#include <cstdint>
#include <functional>
#include <array>
#include <chrono>
#include <mutex>

struct DeviceEvent;
namespace i2c {

  class I2C_Manager;
  

  class PCF8574Out {
    public:
        PCF8574Out(i2c::I2C_Manager& i2c, uint8_t i2c_addr7);
        ~PCF8574Out();
        // Lies aktuellen Portzustand als Startwert (optional); bei Fehler -> Shadow=0xFF
        bool begin(bool read_initial = true);

        // Setzt einzelnes Bit (0..7). level=true => High (Leitung „loslassen“)
        void writeBit(uint8_t pin, bool level);

        // Handler, den der I2C-Manager Aufruft
        void handleOut(const DeviceEvent& e);
        
    private:
        i2c::I2C_Manager& i2c_;
        uint8_t addr_;
        uint8_t shadow_{0xFF};     // PCF startet typ. High (Pullup)
        std::mutex mtx_;

        void flushLocked_();       // schreibt shadow_ raus (mtx_ muss gehalten werden)
  };

} // namespace i2c
