#include "AnalogIn.h"
#include "i2c/core/I2C_Manager.h"
#include <thread>
#include <iostream>

namespace i2c {

    AnalogIn::AnalogIn(I2C_Manager& i2c, uint8_t addr, std::array<int,4> kanal_map, std::chrono::milliseconds period)
    : i2c_(i2c), addr_(addr), kanal_map_(kanal_map), period_(period)
    {
        job_id_ = i2c_.addPeriodic(period_, [this]{ this->tick_(); });
    }

    AnalogIn::~AnalogIn() {
        if (job_id_ >= 0) {
            i2c_.cancel(job_id_);
            job_id_ = -1;
        }
    }

    void AnalogIn::setCallback(uint8_t pin, SampleCb cb) {
        if (pin < 4) cb_[pin] = std::move(cb);
    }

    uint8_t AnalogIn::makeConfigCont_(uint8_t pin) const {
        const uint8_t ch   = (pin & 0x03) << 5; // Bits 6:5
        const uint8_t mode = 1 << 4;            // Continuous
        const uint8_t res  = 0 << 2;       // 12-bit
        const uint8_t pga  = 0;                 // x1
        const uint8_t rdy  = 0 << 7;            // Start
        return static_cast<uint8_t>(rdy | ch | mode | res | pga);
    }

    bool AnalogIn::selectChannelCont_(uint8_t pin) {
        const uint8_t cfg = makeConfigCont_(pin);
        return i2c_.write_bytes(addr_, &cfg, 1) == 1;
    }

    bool AnalogIn::readFrame_(uint8_t* buf, size_t n) {
        const int got = i2c_.read_bytes(addr_, buf, static_cast<int>(n));
        return got == static_cast<int>(n);
    }

    bool AnalogIn::decode16_(const uint8_t* in, int16_t& code, uint8_t& cfg) {
        code = static_cast<int16_t>((in[0] << 8) | in[1]);
        cfg  = in[2];
        return true;
    }

    int64_t AnalogIn::codeToMicrovolts12_(int16_t code) {
        // 1000 µV/LSB @ PGA1
        return static_cast<int64_t>(code) * 1000000LL / 1000LL;
    }

    void AnalogIn::tick_() {
        // nacheinander ch0..ch3
        for (uint8_t pin = 0; pin < 4; ++pin) {
            if (!selectChannelCont_(pin)) continue;

            uint8_t buf[3]{};
            (void)readFrame_(buf, 3); // dummy

            const auto t0 = std::chrono::steady_clock::now();
            while (true) {
                if (!readFrame_(buf, 3)) break;

                int16_t code = 0; uint8_t cfg = 0;
                if (!decode16_(buf, code, cfg)) break;

                if (ready_(cfg) && respChannel_(cfg) == pin) {
                    const auto uv = codeToMicrovolts12_(code);
                    if (cb_[pin]) cb_[pin](uv);
                    const int      pin_i  = static_cast<int>(pin);     // 0..3, sauber als int
                    const int      addr_i = static_cast<int>(static_cast<unsigned>(addr_));   // 0x69 -> 105 (dezimal)
                    const int64_t  uv_i   = static_cast<int>(uv);  // µV als 64-bit Integer
                    DeviceEvent ev{};
                    ev.bus   = 1;
                    ev.addr  = addr_i;
                    ev.pin   = pin_i;
                    ev.analog = 1;
                    ev.value = uv_i;
                    //std::cout << "  AI:  pin=" << pin_i << " addr=" << addr_i << " uv=" << uv_i << "\n";
                    i2c_.enqueue(std::move(ev));
                    break;
                }
                if (std::chrono::steady_clock::now() - t0 > std::chrono::milliseconds(400))
                    break;

                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
} // namespace i2c
