#pragma once
#include <cstdint>
#include <functional>
#include <chrono>
#include <array>

namespace i2c {

    class I2C_Manager;

    class AnalogIn {
    public:
        using SampleCb = std::function<void(int64_t /*uV*/)>;

        
        AnalogIn(I2C_Manager& i2c, uint8_t addr, std::array<int,4> kanal_map, std::chrono::milliseconds period = std::chrono::milliseconds(100));
        AnalogIn(i2c::I2C_Manager& i2c, uint8_t addr, std::array<int,4> kanal_map, int period_ms) 
        : AnalogIn(i2c, addr, kanal_map, std::chrono::milliseconds(period_ms)) {}
        AnalogIn(i2c::I2C_Manager& i2c, uint8_t addr, std::array<int,4> kanal_map, unsigned period_ms)
        : AnalogIn(i2c, addr, kanal_map, std::chrono::milliseconds(static_cast<int>(period_ms))) {}
        ~AnalogIn();

        void setCallback(uint8_t pin, SampleCb cb);   // pin = 0..3

    private:
        // Poll-Schritt: liest nacheinander ch0..ch3 und feuert Callbacks
        void tick_();
        // --- MCP3424 „inline“-Funktionalität (nur das Nötigste) ---
        // Wir nutzen feste Defaults: OneShot, 16-bit, PGA x1
        uint8_t makeConfigCont_(uint8_t pin) const;
        bool selectChannelCont_(uint8_t pin);
        bool readFrame_(uint8_t* buf, size_t n);        // rohes Lesen
        bool ready_(uint8_t cfg) const { return (cfg & 0x80) == 0; } // RDY=0 => fertig
        bool decode16_(const uint8_t* in, int16_t& code, uint8_t& cfg);
        static int64_t codeToMicrovolts12_(int16_t code); // 62.5 uV/LSB @ PGA1
        static uint8_t respChannel_(uint8_t cfg) { return (cfg >> 5) & 0x03; }


        I2C_Manager& i2c_;
        uint8_t addr_{0x69};
        std::array<int,4> kanal_map_{ {-1,-1,-1,-1} };
        std::array<SampleCb,4> cb_{};
        int job_id_{-1};
        std::chrono::milliseconds period_{100};
    };

} // namespace i2c
