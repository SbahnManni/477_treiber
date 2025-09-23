#pragma once
#include <cstdint>
#include <mutex>
#include <atomic>
#include <vector>

#include "../core/I2C_Manager.h"

namespace i2c {

    class MPU9250 {
        public:
            // Pin-IDs für dein key(bus,addr,pin,analog)-Schema
            enum PinId : uint8_t {
                AX = 10, AY = 11, AZ = 12,
                GX = 20, GY = 21, GZ = 22,
                PITCH = 30, ROLL = 31,
                // YAW = 32, // optional später mit Magnetometer
            };

            // rate_hz = Pollfrequenz
            MPU9250(I2C_Manager& i2c, uint8_t addr7, int period = 100);

            // Startet: init Sensor + periodisches Polling
            bool begin();

            // Stoppt das Polling
            void stop();

            // Optional: nur bestimmte Kanäle publizieren (Standard: alle außer Yaw)
            void setEnabledPins(const std::vector<uint8_t>& pins);

        private:
            // I2C Register
            static constexpr uint8_t REG_WHO_AM_I     = 0x75;
            static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
            static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
            static constexpr uint8_t REG_CONFIG       = 0x1A;
            static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
            static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
            static constexpr uint8_t REG_ACCEL_CONFIG2= 0x1D;
            static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B; // 14 Bytes: AX..AZ, TEMP, GX..GZ
            static constexpr uint8_t WHO_AM_I_EXPECT  = 0x71;

            // Skalierungsfaktoren (±2g, ±250 dps)
            static constexpr float ACC_LSB_PER_G   = 16384.0f; // 2g
            static constexpr float GYRO_LSB_PER_DPS= 131.0f;   // 250 dps

            I2C_Manager& i2c_;
            const uint8_t addr_;
            const int period_;
            int job_id_{-1};
            std::mutex mtx_;
            std::atomic<bool> running_{false};

            // welche Pins veröffentlichen?
            std::vector<uint8_t> enabled_pins_;

            // low-level
            bool writeReg_(uint8_t reg, uint8_t val);
            bool readRegs_(uint8_t reg, uint8_t* out, int n);

            // init + tick
            bool hwInit_();
            void tick_();

            // Helper: rohdaten -> int-Werte
            static inline int to_mg(int16_t raw)  { return static_cast<int>(raw * 1000.0f / ACC_LSB_PER_G); }
            static inline int to_dps(int16_t raw) { return static_cast<int>(raw / GYRO_LSB_PER_DPS); }

            // Helper: Pitch/Roll aus Acc (grob, Grad)
            static int pitch_deg_from_acc(int ax_mg, int ay_mg, int az_mg);
            static int roll_deg_from_acc (int ax_mg, int ay_mg, int az_mg);
    };

} // namespace i2c
