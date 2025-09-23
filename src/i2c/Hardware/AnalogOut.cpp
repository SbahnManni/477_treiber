#include "AnalogOut.h"
#include "../core/I2C_Manager.h"
#include <iostream>

namespace i2c {

    AnalogOut::AnalogOut(i2c::I2C_Manager& i2c, uint8_t i2c_addr7) : i2c_(i2c), addr_(i2c_addr7){}
    AnalogOut::~AnalogOut() {}   // Destruktor
    
    bool AnalogOut::begin() {
        // Beim Manager registrieren – identisch zum PCF8574Out-Pattern
        i2c_.registerOut(addr_, [this](const DeviceEvent& e){ this->handleOut(e); });
        return true;
    }

    void AnalogOut::writeChannel(uint8_t channel, int value_0_to_1000) {
        if (channel > 3) return;

        const uint16_t v = clamp1000(value_0_to_1000);
        const uint8_t high = static_cast<uint8_t>(v / 256);            // wie Vorlage
        const uint8_t low  = static_cast<uint8_t>(v - high * 256);      // wie Vorlage

        uint8_t buf[3] = { channel, low, high };                        // [ch, low, high]
        std::lock_guard<std::mutex> lock(mtx_);
        (void)i2c_.write_bytes(addr_, buf, 3);
    }

    void AnalogOut::handleOut(const DeviceEvent& e) {
        if (!e.analog) return;          // nur analoge Events
        if (e.addr != addr_) return;    // nur unsere Adresse
        const uint8_t ch = static_cast<uint8_t>(e.pin);   // pin = DAC-Kanal 0..3
        writeChannel(ch, e.value);      // wert erwartet 0..1000; wird intern geklemmt
    }
    

} // namespace i2c
