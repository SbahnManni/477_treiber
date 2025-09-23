#include "MPU9250.h"
#include <cmath>
#include <cstdio>
#include <chrono>

namespace i2c {

    MPU9250::MPU9250(I2C_Manager& i2c, uint8_t addr7, int period)
    : i2c_(i2c), addr_(addr7), period_(period) {
        // Standard: alle „sinnvollen“ Kanäle aktivieren (ohne Yaw)
        enabled_pins_ = { AX, AY, AZ, GX, GY, GZ, PITCH, ROLL };
    }

    void MPU9250::setEnabledPins(const std::vector<uint8_t>& pins) {
        std::lock_guard<std::mutex> lk(mtx_);
        enabled_pins_ = pins;
    }

    bool MPU9250::begin() {
        if (running_) return true;
        if (!hwInit_()) return false;

        //using namespace std::chrono;
        //auto period = milliseconds(std::max(1, 1000 / std::max(1, rate_hz_)));

        running_ = true;
        job_id_ = i2c_.addPeriodic(std::chrono::milliseconds(period_), [this]{ this->tick_(); });
        return job_id_ > 0;
    }

    void MPU9250::stop() {
        if (!running_) return;
        running_ = false;
        if (job_id_ > 0) {
            i2c_.cancel(job_id_);
            job_id_ = -1;
        }
    }

    bool MPU9250::hwInit_() {
        // Check WHO_AM_I
        uint8_t who = 0;
        if (!readRegs_(REG_WHO_AM_I, &who, 1)) {
            std::fprintf(stderr, "MPU9250: WHO_AM_I read failed\n");
            return false;
        }
        if (who != WHO_AM_I_EXPECT) {
            std::fprintf(stderr, "MPU9250: WHO_AM_I=0x%02X (erwartet 0x%02X)\n", who, WHO_AM_I_EXPECT);
            // nicht abbrechen – einige Clones melden 0x73/0x70
        }

        // Wakeup + Taktquelle PLL
        writeReg_(REG_PWR_MGMT_1, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        writeReg_(REG_PWR_MGMT_1, 0x01); // PLL mit X-Gyro

        // DLPF & Rate
        writeReg_(REG_CONFIG, 0x03);        // DLPF ~44/42Hz
        writeReg_(REG_SMPLRT_DIV, 0x00);    // 1kHz

        // Gyro ±250 dps, Acc ±2g
        writeReg_(REG_GYRO_CONFIG,  0x00);
        writeReg_(REG_ACCEL_CONFIG, 0x00);
        writeReg_(REG_ACCEL_CONFIG2,0x03);  // Acc DLPF

        return true;
    }

    bool MPU9250::writeReg_(uint8_t reg, uint8_t val) {
        uint8_t buf[2] = { reg, val };
        return i2c_.write_bytes(addr_, buf, 2) == 2;
    }

    bool MPU9250::readRegs_(uint8_t reg, uint8_t* out, int n) {
        if (i2c_.write_bytes(addr_, &reg, 1) != 1) return false;
        return i2c_.read_bytes(addr_, out, n) == n;
    }

    int MPU9250::pitch_deg_from_acc(int ax_mg, int ay_mg, int az_mg) {
        // Pitch ≈ atan2(-Ax, sqrt(Ay^2 + Az^2))
        const float ax = ax_mg / 1000.0f, ay = ay_mg / 1000.0f, az = az_mg / 1000.0f;
        const float pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az)) * 180.0f / float(M_PI);
        return static_cast<int>(std::lround(pitch));
    }
    int MPU9250::roll_deg_from_acc(int ax_mg, int ay_mg, int az_mg) {
        // Roll  ≈ atan2(Ay, Az)
        const float ay = ay_mg / 1000.0f, az = az_mg / 1000.0f;
        const float roll = std::atan2(ay, az) * 180.0f / float(M_PI);
        return static_cast<int>(std::lround(roll));
    }

    void MPU9250::tick_() {
        // 14 Bytes ab 0x3B: AXH, AXL, AYH, AYL, AZH, AZL, TH, TL, GXH, GXL, GYH, GYL, GZH, GZL
        uint8_t buf[14]{};
        if (!readRegs_(REG_ACCEL_XOUT_H, buf, 14)) return;

        auto s16 = [](uint8_t hi, uint8_t lo)->int16_t { return int16_t((hi<<8) | lo); };

        const int16_t ax_raw = s16(buf[0],  buf[1]);
        const int16_t ay_raw = s16(buf[2],  buf[3]);
        const int16_t az_raw = s16(buf[4],  buf[5]);
        // const int16_t t_raw  = s16(buf[6],  buf[7]); // Temperatur ungenutzt
        const int16_t gx_raw = s16(buf[8],  buf[9]);
        const int16_t gy_raw = s16(buf[10], buf[11]);
        const int16_t gz_raw = s16(buf[12], buf[13]);

        const int ax_mg = to_mg(ax_raw);
        const int ay_mg = to_mg(ay_raw);
        const int az_mg = to_mg(az_raw);

        const int gx_dps = to_dps(gx_raw);
        const int gy_dps = to_dps(gy_raw);
        const int gz_dps = to_dps(gz_raw);

        const int pitch = pitch_deg_from_acc(ax_mg, ay_mg, az_mg);
        const int roll  = roll_deg_from_acc(ax_mg, ay_mg, az_mg);

        const auto now = std::chrono::steady_clock::now();

        // Kopie der Pins unter Lock ziehen
        std::vector<uint8_t> pins;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            pins = enabled_pins_;
        }

        for (uint8_t pin : pins) {
            DeviceEvent e{};
            e.bus    = 1;
            e.addr   = addr_;
            e.pin    = pin;
            e.analog = true;
            e.ts     = now;

            switch (pin) {
                case AX:    e.value = ax_mg; break;
                case AY:    e.value = ay_mg; break;
                case AZ:    e.value = az_mg; break;
                case GX:    e.value = gx_dps; break;
                case GY:    e.value = gy_dps; break;
                case GZ:    e.value = gz_dps; break;
                case PITCH: e.value = pitch; break;
                case ROLL:  e.value = roll;  break;
                default:    continue; // unbekannt => skip
            }
            i2c_.enqueue(e);
        }
    }

} // namespace i2c
