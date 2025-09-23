#include "PCF8574In.h"
#include "../core/I2C_Manager.h"
#include <iostream>

namespace i2c {

    PCF8574In::PCF8574In(i2c::I2C_Manager& i2c, uint8_t i2c_addr7)
        : i2c_(i2c), addr_(i2c_addr7)
    {
        i2c_.write_byte(addr_, 0xFF);
    }
    PCF8574In::~PCF8574In() {}   // Destruktor
    
    void PCF8574In::setOnChange(std::function<void(uint8_t)> onChange){
        on_change_ = std::move(onChange);
    }

    void PCF8574In::onInterrupt(){
        const uint8_t val = read_port();
        const auto now = std::chrono::steady_clock::now();
        if(!initialized_) {
            initialized_ = true;
            last_ = val;
            for (auto& t : last_edge_) t = now;   // Basiszeit setzen
            return;
        }

        const uint8_t delta = static_cast<uint8_t>(val ^ last_);
        if(delta == 0) return;

        uint8_t accepted_mask = 0;
        
        for (uint8_t pin = 0; pin < 8; ++pin) {
        const uint8_t bit = (1u << pin);
        if ((delta & bit) == 0) continue;

        // Debounce: akzeptiere nur, wenn seit letzter Annahme genug Zeit verging
        if (now - last_edge_[pin] < debounce_) continue;

        last_edge_[pin] = now;               // Fenster neu starten
        accepted_mask  |= bit;

        DeviceEvent ev{};
        ev.bus   = 1;                     // bei dir: statisch 1
        ev.addr  = addr_;
        ev.pin   = pin;
        ev.analog = 0;
        ev.value = (val >> pin) & 0x1u;
        i2c_.enqueue(std::move(ev));
        }
        
        // Nur Bits übernehmen, die wir akzeptiert haben
        if (accepted_mask) {
            last_ = static_cast<uint8_t>((last_ & ~accepted_mask) | (val & accepted_mask));
        }

    }

    uint8_t PCF8574In::read_port() const {
        return static_cast<uint8_t>(~i2c_.read_byte(addr_));
    }

} // namespace i2c