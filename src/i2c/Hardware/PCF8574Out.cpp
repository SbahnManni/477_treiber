#include "PCF8574Out.h"
#include "../core/I2C_Manager.h"
#include <iostream>

namespace i2c {

    PCF8574Out::PCF8574Out(i2c::I2C_Manager& i2c, uint8_t i2c_addr7)
        : i2c_(i2c), addr_(i2c_addr7)
    {
        i2c_.write_byte(addr_, 0x00);
    }
    PCF8574Out::~PCF8574Out() {}   // Destruktor
    
    bool PCF8574Out::begin(bool read_initial) {
        std::lock_guard<std::mutex> lk(mtx_);

        if (read_initial) {
            uint8_t b = 0xFF;
            const int n = i2c_.read_bytes(addr_, &b, 1);
            shadow_ = (n == 1) ? b : 0xFF;
            // kein Flush: wir übernehmen den tatsächlichen Zustand
        } else {
            // sicherer Default: alles AUS (HIGH) und sofort schreiben
            shadow_ = 0xFF;
            flushLocked_();
        }

        i2c_.registerOut(addr_, [this](const DeviceEvent& e){ this->handleOut(e); });
        return true;
    }

    void PCF8574Out::writeBit(uint8_t pin, bool level) {
        if (pin > 7) return;
        std::lock_guard<std::mutex> lk(mtx_);
        const uint8_t mask = static_cast<uint8_t>(1u << pin);
        const uint8_t neu  = !level ? static_cast<uint8_t>(shadow_ | mask)
                                : static_cast<uint8_t>(shadow_ & ~mask);
        if (neu != shadow_) {
            shadow_ = neu;
            flushLocked_();
        }
    }

    void PCF8574Out::handleOut(const DeviceEvent& e) {
        // defensive: nur unsere Adresse/Bustyp
        if (e.addr != addr_) return;
        if (e.analog) return; // digitaler PCF: analog ignorieren (später DAC)
        writeBit(static_cast<uint8_t>(e.pin), e.value != 0);
    }

    void PCF8574Out::flushLocked_() {
        (void)i2c_.write_bytes(addr_, &shadow_, 1);
    }
    

} // namespace i2c
