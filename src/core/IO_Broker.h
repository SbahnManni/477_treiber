#pragma once
#include <cstdint>
#include <string>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <cctype>
#include <functional>
#include <tuple>
#include "Telegram.h"
#include "../i2c/core/I2C_Manager.h"   // für DeviceEvent

class IoBroker {
    public:
        explicit IoBroker(i2c::I2C_Manager& mgr);

        void addPcfInputMapping(int bus, int addr, int pin, int kanal, std::string name);
        void addAnalogInputMapping(int bus, int addr, int pin, int kanal, std::string name);
        void addDigiOutMapping   (int bus, int addr, int pin, int kanal, std::string name);
        void addAnalogOutMapping (int bus, int addr, int ch,  int kanal, std::string name);
        void addImuMapping       (int bus, int addr, int pin_id, int kanal, std::string name); //Lagesensor
        
        
        // Eigener Thread
        void setOnTelegram(std::function<void(const Telegram&)> cb); // registrieren
        void start();                                                // Worker starten
        void stop();                                                 // Worker stoppen


        // HW -> Logic
        bool tryPopIn(Telegram& out);
        bool waitPopIn(Telegram& out, int ms);
        bool waitPopIn(Telegram& out);


        // Logic -> HW (Outputs kommen später)
        void publishOut(const Telegram& t);


        const char* nameOf(int kanal) const;

        // Json Loader
        bool loadConfig(const std::string& json_path);
        const std::string& lastError() const {return last_error_;}

    private:
        i2c::I2C_Manager& mgr_;
        static uint32_t key(int bus, uint8_t addr, uint8_t pin, bool analog);
        std::unordered_map<uint32_t, int> ain_rate_by_dev_;
        std::unordered_map<uint32_t, int> in_map_;   // (bus,addr,pin,analog) -> kanal
        using OutTuple = std::tuple<int, int, int, bool>;
        std::unordered_map<int, OutTuple> out_map_;   // (bus,addr,pin,analog) -> kanal
        std::unordered_map<int, std::string> names_; // kanal -> name
        std::string last_error_;
        std::function<void(const Telegram&)> on_telegram_;
        std::thread worker_;
        std::atomic<bool> running_{false};
};