#include "IO_Broker.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cctype>
#include <nlohmann/json.hpp>

// Fuer MCP9250 Lagesensor
static inline int imu_pin_id_from(const std::string& s) {
    if (s == "ax") return 10;
    if (s == "ay") return 11;
    if (s == "az") return 12;
    if (s == "gx") return 20;
    if (s == "gy") return 21;
    if (s == "gz") return 22;
    if (s == "pitch") return 30;
    if (s == "roll")  return 31;
    if (s == "yaw")   return 32;
    return -1;
}

IoBroker::IoBroker(i2c::I2C_Manager& mgr) : mgr_(mgr) {}

void IoBroker::addPcfInputMapping(int bus, int addr, int pin, int kanal, std::string name) {
    in_map_[key(bus, addr, pin, 0)] = kanal;
    if (!name.empty()) names_by_ch_[chan_key(kanal,  /*analog=*/ false)] = std::move(name);
}
void IoBroker::addAnalogInputMapping(int bus, int addr, int pin, int kanal, std::string name){
    in_map_[key(bus, addr, pin, 1)] = kanal;
    std::cout << "bus " << bus << " addr " << addr << " pin " << pin << " kanal " << kanal << " name " << name << "\n";
    if (!name.empty()) names_by_ch_[chan_key(kanal, /*analog=*/ true)] = std::move(name);
}


void IoBroker::addDigiOutMapping   (int bus, int addr, int pin, int kanal, std::string name){
        out_map_[chan_key(kanal, /*analog=*/false)] = OutTuple{bus, addr, pin, false} ;
    if (!name.empty()) names_by_ch_[chan_key(kanal, /*analog=*/false)] = std::move(name);
}
void IoBroker::addAnalogOutMapping (int bus, int addr, int pin,  int kanal, std::string name){
    out_map_[chan_key(kanal, /*analog=*/true)] = OutTuple{bus, addr, pin, true} ;
    if (!name.empty()) names_by_ch_[chan_key(kanal, /*analog=*/true)] = std::move(name);
    }
    
    void IoBroker::addImuMapping(int bus, int addr, int pin_id, int kanal, std::string name) {
        in_map_[key(bus, static_cast<uint8_t>(addr), static_cast<uint8_t>(pin_id), /*analog=*/1)] = kanal;
        if (!name.empty()) names_[kanal] = std::move(name);
    }

void IoBroker::setOnTelegram(std::function<void(const Telegram&)> cb) {
    on_telegram_ = std::move(cb);
}

void IoBroker::start() {
    if (running_) return;
    running_ = true;
    worker_ = std::thread([this]{
        Telegram t{};
        while (running_) {
            if (this->waitPopIn(t, 100)) {              // blockiert bis Event/100ms
                if (on_telegram_) on_telegram_(t);       // Event nach oben melden
                // optional: Burst leerziehen
                while (this->tryPopIn(t)) {
                    if (on_telegram_) on_telegram_(t);
                }
            }
        }
    });
}

void IoBroker::stop() {
    if (!running_) return;
    running_ = false;
    // falls gerade blockiert, weck den Worker über einen Dummy-Event
    DeviceEvent dummy{}; dummy.bus=1; dummy.addr=0; dummy.pin=0; dummy.analog=0; dummy.value=false;
    mgr_.enqueue(dummy);
    if (worker_.joinable()) worker_.join();
}

bool IoBroker::tryPopIn(Telegram& out) {
    DeviceEvent ev{};
    if (!mgr_.try_pop(ev)) return false;

    const uint32_t k = key(ev.bus, ev.addr, ev.pin, ev.analog);
    auto it = in_map_.find(k);
    if (it == in_map_.end()) {
        //std::printf("IoBroker: unmapped input bus=%d addr=0x%02X pin=%u -> ignored\n",ev.bus, ev.addr, ev.pin);
        std::cout<< " IoBroker: unmapped input bus=" << ev.bus << " addr=" << ev.addr << " pin=" << ev.pin << " Analog=" << ev.analog << " -> ignored!\n";
        return false;
    }

    out.kanal = it->second;
    out.analog  = ev.analog;             // digital
    out.wert  = ev.value;
    return true;
}

bool IoBroker::waitPopIn(Telegram& out, int ms) {
    DeviceEvent ev{};
    if (!mgr_.wait_pop(ev, ms)) return false;

    const uint32_t k = key(ev.bus, ev.addr, ev.pin, ev.analog);
    auto it = in_map_.find(k);
    if (it == in_map_.end()) {
        //std::printf("IoBroker: unmapped input bus=%d addr=0x%02X pin=%u -> ignored\n", ev.bus, ev.addr, ev.pin);
        std::cout<< " IoBroker: unmapped input bus=" << ev.bus << " addr=" << ev.addr << " pin=" << ev.pin << " Analog=" << ev.analog << " -> ignored!\n";
        return false;
    }

    out.kanal = it->second;
    out.analog  = ev.analog;
    out.wert  = ev.value;
    return true;
}

bool IoBroker::waitPopIn(Telegram& out) {
    DeviceEvent ev{};
    if (!mgr_.wait_pop(ev)) return false; // kommt immer true zurück
    const uint32_t k = key(ev.bus, ev.addr, ev.pin, ev.analog);
    auto it = in_map_.find(k);
    if (it == in_map_.end()) return false; // selten, dann weiter warten
    out.kanal = it->second;
    out.analog  = ev.analog;
    out.wert  = ev.value;
    return true;
}

void IoBroker::publishOut(const Telegram& t) {
    const uint32_t ck = chan_key(t.kanal, t.analog);
    auto it = out_map_.find(ck);
    if(it == out_map_.end()){
        std::cout << "IoBroker: kein OUT-Mapping fuer Kanal " << t.kanal << " (analog=" << t.analog <<") \n";
        return;
    }
    const auto& [bus, addr, pin, analog] = it ->second;

    DeviceEvent ev{};
    ev.bus = bus;
    ev.addr = addr;
    ev.pin = pin;
    ev.value = t.wert;
    ev.analog = analog;
    mgr_.processOut(ev);
}


const char* IoBroker::nameOf(int kanal) const {
    auto it = names_.find(kanal);
    return (it != names_.end()) ? it->second.c_str() : nullptr;
}

const char* IoBroker::nameOf(int kanal, bool analog) const {
    const uint32_t ck = chan_key(kanal, analog);
    if (auto it = names_by_ch_.find(ck); it != names_by_ch_.end())
        return it->second.c_str();
    // Fallback: anderer Typ?
    if (auto it2 = names_by_ch_.find(chan_key(kanal, !analog)); it2 != names_by_ch_.end())
        return it2->second.c_str();
    return nullptr;
}

uint32_t IoBroker::key(int bus, uint8_t addr, uint8_t pin, bool analog) {
    return ( (static_cast<uint32_t>(bus)  & 0xFF) << 24 ) |
           ( (static_cast<uint32_t>(addr) & 0xFF) << 16 ) |
           ( (static_cast<uint32_t>(pin)  & 0xFF) << 8  ) |
           ( static_cast<uint32_t>(analog ? 1u : 0u)      );
}

using nlohmann::json;
bool IoBroker::loadConfig(const std::string& json_path) {
    last_error_.clear();

    std::ifstream in(json_path);
    if (!in) { last_error_ = "IoBroker: cannot open " + json_path; return false; }

    json root;
    try { in >> root; }
    catch (const std::exception& e) {
        last_error_ = std::string("IoBroker: JSON parse error: ") + e.what();
        return false;
    }

    // --- Helfer ---
    auto parse_addr = [](const json& j, uint8_t& out)->bool {
        if (j.is_string()) {
            std::string s = j.get<std::string>();
            try {
                unsigned long v = std::stoul(s, nullptr, 0); // "0x69" oder "105"
                if (v > 0xFF) return false;
                out = static_cast<uint8_t>(v);
                return true;
            } catch (...) { return false; }
        } else if (j.is_number_integer()) {
            long v = j.get<long>();
            if (v < 0 || v > 0xFF) return false;
            out = static_cast<uint8_t>(v);
            return true;
        }
        return false;
    };
    auto get_arr = [&](std::initializer_list<const char*> keys) -> const json* {
        for (auto k : keys) {
            if (root.contains(k) && root[k].is_array()) return &root[k];
        }
        return nullptr;
    };

    int n_pcf_in = 0, n_ai = 0, n_do = 0, n_ao = 0, n_imu = 0;

    // --- PCF8574 Inputs (wie bisher, nur optional) ---
    if (const json* arr = get_arr({"pcf8574_inputs"})) {
        for (const auto& e : *arr) {
            if (!e.contains("addr") || !e.contains("pin") || !e.contains("kanal")) continue;
            uint8_t addr{};
            if (!parse_addr(e["addr"], addr)) continue;
            if (!e["pin"].is_number_integer() || !e["kanal"].is_number_integer()) continue;
            int bus   = e.value("bus", 1);
            int pin   = e["pin"].get<int>();
            int kanal = e["kanal"].get<int>();
            if (pin < 0 || pin > 7) continue;
            std::string name = e.value("name", std::string{});
            addPcfInputMapping(bus, addr, static_cast<uint8_t>(pin), kanal, std::move(name));
            ++n_pcf_in;
        }
        std::printf("IoBroker: loaded %d PCF8574 input mappings\n", n_pcf_in);
    }

    // --- Analog Inputs (MCP342x) ---
    // akzeptiere "Analog_inputs" (dein JSON) und "analog_inputs"
    if (const json* arr = get_arr({"Analog_inputs", "analog_inputs"})) {
        for (const auto& e : *arr) {
            if (!e.contains("addr") || !e.contains("pin") || !e.contains("kanal")) continue;
            uint8_t addr{};
            if (!parse_addr(e["addr"], addr)) continue;
            if (!e["pin"].is_number_integer() || !e["kanal"].is_number_integer()) continue;
            int bus   = e.value("bus", 1);
            int pin   = e["pin"].get<int>();     // 0..3
            int kanal = e["kanal"].get<int>();
            if (pin < 0 || pin > 3) continue;
            std::string name = e.value("name", std::string{});
            addAnalogInputMapping(bus, addr, static_cast<uint8_t>(pin), kanal, std::move(name));
            ++n_ai;
        }
        std::printf("IoBroker: loaded %d Analog input mappings\n", n_ai);
    }

    // --- Digital Outputs (PCF8574 Out) – vorbereiten ---
    if (const json* arr = get_arr({"pcf8574_outputs", "digi_outputs"})) {
        for (const auto& e : *arr) {
            if (!e.contains("addr") || !e.contains("pin") || !e.contains("kanal")) continue;
            uint8_t addr{};
            if (!parse_addr(e["addr"], addr)) continue;
            if (!e["pin"].is_number_integer() || !e["kanal"].is_number_integer()) continue;
            int bus   = e.value("bus", 1);
            int pin   = e["pin"].get<int>();     // 0..7
            int kanal = e["kanal"].get<int>();
            if (pin < 0 || pin > 7) continue;
            std::string name = e.value("name", std::string{});
            addDigiOutMapping(bus, addr, static_cast<uint8_t>(pin), kanal, std::move(name));
            ++n_do;
        }
        std::printf("IoBroker: loaded %d Digital output mappings\n", n_do);
    }

    // --- Analog Outputs (DAC) – vorbereiten ---
    if (const json* arr = get_arr({"analog_outputs"})) {
        for (const auto& e : *arr) {
            if (!e.contains("addr") || !e.contains("pin") || !e.contains("kanal")) continue;
            uint8_t addr{};
            if (!parse_addr(e["addr"], addr)) continue;
            if (!e["pin"].is_number_integer() || !e["kanal"].is_number_integer()) continue;
            int bus     = e.value("bus", 1);
            int pin = e["pin"].get<int>(); // 0..3 typischer 4ch-DAC
            int kanal   = e["kanal"].get<int>();
            if (pin < 0 || pin > 3) continue;
            std::string name = e.value("name", std::string{});
            addAnalogOutMapping(bus, addr, static_cast<uint8_t>(pin), kanal, std::move(name));
            ++n_ao;
        }
        std::printf("IoBroker: loaded %d Analog output mappings\n", n_ao);
    }

    if (const json* arr = get_arr({"imu_mpu9250", "mpu9250"})) {
        for (const auto& e : *arr) {
            uint8_t addr{};
            if (!e.contains("addr") || !parse_addr(e["addr"], addr)) continue;
            int bus = e.value("bus", 1);
            const auto* chans = e.contains("channels") && e["channels"].is_array()
                                ? &e["channels"] : nullptr;
            if (!chans) continue;

            for (const auto& c : *chans) {
                if (!c.contains("pin") || !c.contains("kanal")) continue;
                std::string pin_str = c["pin"].get<std::string>();
                int pin_id = imu_pin_id_from(pin_str);
                if (pin_id < 0) continue;
                int kanal = c["kanal"].get<int>();
                std::string name = c.value("name", pin_str);
                addImuMapping(bus, addr, pin_id, kanal, std::move(name));
            }
            ++n_imu;
        }
        std::printf("IoBroker: loaded %d MPU-9250 mappings\n", n_imu);
    }
    const int total = n_pcf_in + n_ai + n_do + n_ao + n_imu;
    if (total == 0) {
        last_error_ = "IoBroker: no supported mapping blocks found in " + json_path;
        return false;
    }
    return true;
    
} // Json parser