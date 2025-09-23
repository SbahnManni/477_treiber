#include "I2C_Manager.h"
#include "I2C_Handle.h"

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <string>
#include <poll.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <fstream>
#include <sstream>
#include <thread>
#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <algorithm>
#include <iostream>
#include <functional>


// GPIO-Helper // Implementierung unten
namespace {
    bool path_exists(const std::string& p);
    std::string gpio_path(int gpio, const std::string& leaf);
    void write_file(const std::string& path, const std::string& val);
    const char* edge_to_cstr(i2c::I2C_Manager::Edge e);
    int gpio_open_value_fd(int gpio, const char* edge) ;
    bool gpio_wait_edge(int fd, int timeout_ms);
} // namespace

namespace i2c {

    I2C_Manager::I2C_Manager(int bus) : bus_(bus), handle_(std::make_unique<I2cHandle>(bus)) {
        bus_path_ = "/dev/i2c-" + std::to_string(bus_);
        
        fd_ = ::open(bus_path_.c_str(), O_RDWR | O_CLOEXEC);
        if(fd_ < 0){
            throw std::runtime_error("open " + bus_path_ + " failed: " + std::strerror(errno));
        }

        stop_ = false;
        sched_ = std::thread(&I2C_Manager::schedulerLoop_, this);
    }
    I2C_Manager::~I2C_Manager() { 
        // INT-Thread sauber beenden
        if (int_running_) {
            int_running_ = false;
            if (int_thread_.joinable()) int_thread_.join();
        }
        stop_ = true;
        jobs_cv_.notify_all();
        if (sched_.joinable()) sched_.join();
        if(fd_ > 0) ::close(fd_);
    }

    void I2C_Manager::set_slave_(uint8_t addr7) {
        if (ioctl(handle_->fd(), I2C_SLAVE, addr7) < 0) {
            throw std::runtime_error(std::string("I2C: ioctl(I2C_SLAVE) fehlgeschlagen: ")
                                    + std::strerror(errno));
        }
    }

        // AnalogIN
    // ===== Periodische Jobs =====
    int I2C_Manager::addPeriodic(std::chrono::milliseconds period, std::function<void()> fn) {
        if (!fn || period.count() <= 0) return -1;

        std::lock_guard<std::mutex> lk(jobs_mtx_);
        const int id = next_job_id_++;
        PeriodicJob job;
        job.id = id;
        job.period = period;
        job.next = std::chrono::steady_clock::now() + period;
        job.fn = std::move(fn);
        job.active = true;
        jobs_.emplace(id, std::move(job));
        jobs_cv_.notify_all();
        return id;
    }

    void I2C_Manager::cancel(int job_id) {
        std::lock_guard<std::mutex> lk(jobs_mtx_);
        auto it = jobs_.find(job_id);
        if (it != jobs_.end()) {
            it->second.active = false;
            jobs_.erase(it);
            jobs_cv_.notify_all();
        }
    }

        // Outputs

    void I2C_Manager::registerOut(uint8_t addr7, std::function<void(const DeviceEvent&)> handler) {
        const uint32_t key = (static_cast<uint32_t>(bus_) << 8) | (addr7 & 0xFFu);
        std::lock_guard<std::mutex> lk(out_mtx_);
        out_handlers_[key] = std::move(handler);
    }

    void I2C_Manager::unregisterOut(uint8_t addr7) {
        const uint32_t key = (static_cast<uint32_t>(bus_) << 8) | (addr7 & 0xFFu);
        std::lock_guard<std::mutex> lk(out_mtx_);
        out_handlers_.erase(key);
    }
    
    void I2C_Manager::processOut(const DeviceEvent& e) {
        //std::cout << "I2C_Manager::processOut bus="<< e.bus << " addr="<< e.addr << " pin=" << e.pin << " val=" << e.value <<" analog=" << e.analog << "\n";
        if (e.bus != bus_) {
                std::fprintf(stderr, "I2C_Manager(bus=%d): ignoriert OUT für bus=%d addr=0x%02X\n",
                            bus_, e.bus, e.addr);
                return;
            }

        std::function<void(const DeviceEvent&)> handler;
        {
            const uint32_t key = (static_cast<uint32_t>(e.bus) << 8) | (static_cast<uint32_t>(e.addr) & 0xFFu);
            std::lock_guard<std::mutex> lk(out_mtx_);
            auto it = out_handlers_.find(key);
            if (it != out_handlers_.end()) handler = it->second;
        }
        if (handler) {
            handler(e); // Gerät kümmert sich (z.B. PCF8574Out)
        } else {
            std::fprintf(stderr, "I2C_Manager: kein Out-Handler für addr=0x%02X (bus=%d)\n", e.addr, e.bus);
        }
    }

    // ===== Scheduler-Loop =====
    void I2C_Manager::schedulerLoop_() {
        std::unique_lock<std::mutex> lk(jobs_mtx_);

        while (!stop_) {
            if (jobs_.empty()) {
                jobs_cv_.wait(lk, [this]{ return stop_.load() || !jobs_.empty(); });
                if (stop_) break;
            }

            // nächste Fälligkeit bestimmen
            auto now = std::chrono::steady_clock::now();
            auto next_tp = now + std::chrono::hours(24); // weit in der Zukunft

            // due jobs sammeln (aber Callback außerhalb Lock ausführen)
            std::vector<std::function<void()>> due;
            for (auto &kv : jobs_) {
                auto &j = kv.second;
                if (!j.active) continue;
                if (j.next <= now) {
                    due.push_back(j.fn);
                    // nächste Fälligkeit vorziehen (keine Drift)
                    j.next = now + j.period;
                }
                next_tp = std::min(next_tp, j.next);
            }

            if (due.empty()) {
                // bis zum nächsten Job schlafen (oder Abbruch)
                jobs_cv_.wait_until(lk, next_tp, [this]{ return stop_.load(); });
                continue;
            }

            // Callbacks ohne Lock ausführen
            lk.unlock();
            for (auto &fn : due) {
                try {
                    fn();
                } catch (...) {
                    // bewusst schlucken; System soll „einfach laufen“
                    // optional: std::cerr << "Periodic job threw\n";
                }
            }
            lk.lock();
        }
    }

    void I2C_Manager::write_byte(uint8_t addr7, uint8_t value) {
        std::lock_guard<std::mutex> lock(mtx_);
        set_slave_(addr7);
        uint8_t buf[1] = { value };
        const ssize_t n = ::write(handle_->fd(), buf, 1);
        if (n != 1) {
            throw std::runtime_error(std::string("I2C: write fehlgeschlagen: ")
                                    + std::strerror(errno));
        }
    }

    uint8_t I2C_Manager::read_byte(uint8_t addr7) {
        std::lock_guard<std::mutex> lock(mtx_);
        set_slave_(addr7);
        uint8_t b = 0xFF;
        const ssize_t n = ::read(handle_->fd(), &b, 1);
        if (n != 1) {
            throw std::runtime_error(std::string("I2C: read fehlgeschlagen: ")
                                    + std::strerror(errno));
        }
        return b;
    }

    int I2C_Manager::write_bytes(uint8_t addr7, const uint8_t* data, int n) {
        std::lock_guard<std::mutex> guard(i2c_mtx_);
        if (!data || n <= 0) return 0;

        struct i2c_msg msg;
        msg.addr  = addr7;
        msg.flags = 0;             // Write
        msg.len   = static_cast<__u16>(n);
        msg.buf   = const_cast<uint8_t*>(data); // Kernel-API verlangt non-const

        struct i2c_rdwr_ioctl_data xfer;
        xfer.msgs  = &msg;
        xfer.nmsgs = 1;

        const int rc = ::ioctl(fd_, I2C_RDWR, &xfer);
        if (rc != 1) {
            // optional: Logging
            return -1;
        }
        return n; // genau n Bytes geschrieben
    }

    int I2C_Manager::read_bytes(uint8_t addr7, uint8_t* out, int n) {
        std::lock_guard<std::mutex> guard(i2c_mtx_);
        if (!out || n <= 0) return 0;

        // Eine einzige Read-Message
        struct i2c_msg msg;
        msg.addr  = addr7;         // 7-bit Adresse (0x69, NICHT <<1)
        msg.flags = I2C_M_RD;      // Read
        msg.len   = static_cast<__u16>(n);
        msg.buf   = out;

        struct i2c_rdwr_ioctl_data xfer;
        xfer.msgs  = &msg;
        xfer.nmsgs = 1;

        const int rc = ::ioctl(fd_, I2C_RDWR, &xfer);
        if (rc != 1) {
            // optional: Logging mit errno
            // std::cerr << "I2C_RDWR read failed: " << std::strerror(errno) << "\n";
            return -1; // Fehler
        }
        return n; // genau n Bytes gelesen
    }



    void I2C_Manager::enqueue(DeviceEvent e){
        //std::cout << " _IM:  pin=" << e.pin << " addr=" << e.addr << " uv=" << e.value << "\n";
        {std::lock_guard<std::mutex> lk(q_mtx_); q_.push_back(std::move(e));}
        q_cv_.notify_one();
    }
    bool I2C_Manager::try_pop(DeviceEvent& out){
        std::lock_guard<std::mutex> lk(q_mtx_);
        if(q_.empty()) return false;
        out= std::move(q_.front());
        q_.pop_front();
        return true;
    }
    bool I2C_Manager::wait_pop(DeviceEvent& out, int ms){
        std::unique_lock<std::mutex> lk(q_mtx_);
        if(!q_cv_.wait_for(lk, std::chrono::milliseconds(ms),[this]{return !q_.empty();})){
            return false;
        }
        out = std::move(q_.front());
        q_.pop_front();
        return true;
    }
    bool I2C_Manager::wait_pop(DeviceEvent& out) {
        std::unique_lock<std::mutex> lk(q_mtx_);
        q_cv_.wait(lk, [this]{ return !q_.empty(); }); // weckt nur bei neuem Event
        out = std::move(q_.front());
        q_.pop_front();
        return true;
    }


    int I2C_Manager::bus() const { return handle_->bus(); }

    // --- Minimal-Interrupt-API (nur damit es linkt) ---
    void i2c::I2C_Manager::subscribeInterrupt(std::function<void()> handler) {
        int_handlers_.push_back(std::move(handler));
    }

    void i2c::I2C_Manager::clearInterruptHandlers() {
        int_handlers_.clear();
    }

    void i2c::I2C_Manager::setGlobalInterrupt(int gpioPin, Edge edge, int debounceMs) {
    // Falls bereits aktiv: stoppen und neu starten
    if (int_running_) {
        int_running_ = false;
        if (int_thread_.joinable()) int_thread_.join();
    }

    int_gpio_        = gpioPin;
    int_edge_        = edge;
    int_debounce_ms_ = debounceMs;

    int_running_ = true;
    int_thread_ = std::thread([this]{
        int fd = gpio_open_value_fd(int_gpio_, edge_to_cstr(int_edge_));

        while (int_running_) {
            if (!gpio_wait_edge(fd, -1)) continue;
            
            if (int_debounce_ms_ > 0) {
               std::this_thread::sleep_for(std::chrono::milliseconds(int_debounce_ms_));
            }

            // Alle registrierten Handler ausführen (jede PCF-Instanz liest selbst)
            for (auto& h : int_handlers_) {
                if (h) h();
            }
        }

        ::close(fd);
    });
}

} // namespace i2c

// GPIO-Helper fuer Interupterkennung
namespace {
    bool path_exists(const std::string& p) {
        struct stat st{};
        return ::stat(p.c_str(), &st) == 0;
    }

    std::string gpio_path(int gpio, const std::string& leaf) {
        std::ostringstream oss;
        oss << "/sys/class/gpio/gpio" << gpio << "/" << leaf;
        return oss.str();
    }

    void write_file(const std::string& path, const std::string& val) {
        std::ofstream f(path);
        if (!f) throw std::runtime_error("GPIO: open failed: " + path);
        f << val;
        if (!f) throw std::runtime_error("GPIO: write failed: " + path);
    }

    const char* edge_to_cstr(i2c::I2C_Manager::Edge e) {
        switch (e) {
            case i2c::I2C_Manager::Edge::Rising:  return "rising";
            case i2c::I2C_Manager::Edge::Falling: return "falling";
            case i2c::I2C_Manager::Edge::Both:    return "both";
        }
        return "none";
    }

    int gpio_open_value_fd(int gpio, const char* edge) {
        const std::string base = "/sys/class/gpio/gpio" + std::to_string(gpio);
        if (!path_exists(base)) {
            write_file("/sys/class/gpio/export", std::to_string(gpio));
        }
        write_file(gpio_path(gpio, "direction"), "in");
        write_file(gpio_path(gpio, "edge"), edge);

        int fd = ::open(gpio_path(gpio, "value").c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) throw std::runtime_error("GPIO: open value failed");

        // prime: alten Zustand „verbrauchen“
        char buf;
        ::lseek(fd, 0, SEEK_SET);
        ::read(fd, &buf, 1);
        return fd;
    }

    bool gpio_wait_edge(int fd, int timeout_ms) {
        struct pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLPRI | POLLERR;
        int r = ::poll(&pfd, 1, timeout_ms);
        if (r < 0) throw std::runtime_error("GPIO: poll() failed");
        if (r == 0) return false;
        // Ereignis „verbrauchen“
        char buf;
        ::lseek(fd, 0, SEEK_SET);
        ::read(fd, &buf, 1);
        return (pfd.revents & (POLLPRI | POLLERR)) != 0;
    }
} // namespace