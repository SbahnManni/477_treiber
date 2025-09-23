#pragma once
#include <cstdint>
#include <chrono>
#include <mutex>
#include <stdexcept>
#include <functional>
#include <vector>
#include <thread>
#include <atomic>
#include <deque>
#include <condition_variable>
#include <cstdint>
#include <string>
#include <memory>
#include <functional>
#include <chrono>
#include <map>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

struct DeviceEvent{
    int bus;
    int addr;
    int pin;
    int value;
    bool analog;
    std::chrono::steady_clock::time_point ts;
};

namespace i2c {

    class I2cHandle; // fwd

    class I2C_Manager {
        public:
            explicit I2C_Manager(int bus);
            ~I2C_Manager();

            void write_byte(uint8_t addr7, uint8_t value);
            uint8_t read_byte(uint8_t addr7);
            int write_bytes(uint8_t addr7, const uint8_t* data, int n);
            int read_bytes(uint8_t addr7, uint8_t* out, int n);
            void enqueue(DeviceEvent e);
            bool try_pop(DeviceEvent& out);
            bool wait_pop(DeviceEvent& out, int ms);    //Timeout
            bool wait_pop(DeviceEvent& out);    // Wartet bis die Sonne untergeht

            void registerOut(uint8_t addr7, std::function<void(const DeviceEvent&)> handler);
            void unregisterOut(uint8_t addr7);
            void processOut(const DeviceEvent& e);   // Telegram -> DeviceEvent kam vom Broker

            // === Periodische Jobs (für Polling) ===
            // Rückgabe: Job-ID (>0). Bei Fehler: -1
            int addPeriodic(std::chrono::milliseconds period, std::function<void()> fn);
            void cancel(int job_id);

            int bus() const;
            enum class Edge { Rising, Falling, Both };

            // Interruptauswertung fuer PCF8574 Inputs
            // Interrupt-Leitung einmal global aktivieren (z. B. GPIO17, Rising, optional Debounce)
            void setGlobalInterrupt(int gpioPin, Edge edge = Edge::Rising, int debounceMs = 0);
            // Einen Handler registrieren, der bei jedem Edge aufgerufen wird
            void subscribeInterrupt(std::function<void()> handler);
            // Optional: alle Handler löschen/INT stoppen (für später)
            void clearInterruptHandlers();


        private:
            void set_slave_(uint8_t addr7);

            std::mutex   mtx_;
            std::mutex q_mtx_;
            std::condition_variable q_cv_;
            std::deque<DeviceEvent> q_;

            std::mutex out_mtx_;
            std::unordered_map<uint32_t, std::function<void(const DeviceEvent&)>> out_handlers_;

            
            // --- Bus ---
            int bus_{-1};
            std::unique_ptr<I2cHandle> handle_;
            std::string bus_path_; // z.B. "/dev/i2c-1"
            int fd_{-1};
            std::mutex i2c_mtx_;
            // INT-Thread
            int                 int_gpio_ = -1;
            Edge                int_edge_ = Edge::Rising;
            int                 int_debounce_ms_ = 0;
            std::vector<std::function<void()>> int_handlers_;
            std::thread         int_thread_;
            std::atomic<bool>   int_running_{false};

            // --- Scheduler intern ---
            struct PeriodicJob {
                int id;
                std::chrono::milliseconds period;
                std::chrono::steady_clock::time_point next;
                std::function<void()> fn;
                bool active{true};
            };

            void schedulerLoop_();

            // --- Scheduler-Thread & Jobs ---
            std::thread sched_;
            std::mutex jobs_mtx_;
            std::condition_variable jobs_cv_;
            std::map<int, PeriodicJob> jobs_;  // key = job_id
            int next_job_id_{1};
            std::atomic<bool> stop_{false};
            };

} // namespace i2c
