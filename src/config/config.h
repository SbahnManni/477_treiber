#pragma once
#include <cstdint>
#include <array>
#include <chrono>
#include <string_view>

// Alles in einen Namespace packen, damit’s sauber bleibt.
namespace cfg {

// -------- UART --------
inline constexpr const char* UART_DEVICE = "/dev/ttyAMA0";
inline constexpr int UART_BAUD = 115200;

// -------- I2C --------
inline constexpr const char* I2C_DEVICE = "/dev/i2c-1";
inline constexpr uint8_t I2C_ADDR_MPU9250 = 0x68;

inline constexpr uint8_t I2C_ADDR_DigiIn = 0x38;
inline constexpr uint8_t I2C_ADDR_DigiOut = 0x20;

inline constexpr uint8_t I2C_ADDR_AnalogIn = 0x69;
inline constexpr uint8_t I2C_ADDR_AnalogOut = 0x58;



// PCF8574-Eingänge (Beispiel: 5 Boards)
//inline constexpr std::array<uint8_t, 5> I2C_ADDR_PCF8574_IN  = { 0x20, 0x21, 0x22, 0x23, 0x24 };
// PCF8574-Ausgänge (Beispiel: 3 Boards)
//inline constexpr std::array<uint8_t, 3> I2C_ADDR_PCF8574_OUT = { 0x26, 0x27, 0x28 };

// -------- Größen / Kanäle --------
//inline constexpr int NUM_AI = 4;   // Analoge Eingänge
//inline constexpr int NUM_AO = 4;   // Analoge Ausgänge
//inline constexpr int NUM_DI = 37;  // Digitale Eingänge (gesamt)
//inline constexpr int NUM_DO = 20;  // Digitale Ausgänge (gesamt)

// -------- Timings --------
//inline constexpr auto I2C_POLL_PERIOD = std::chrono::milliseconds(5);
//inline constexpr auto UART_TX_PERIOD  = std::chrono::milliseconds(10);

// -------- TCP/IPC --------
//inline constexpr uint16_t TCP_LISTEN_PORT = 5000;

// -------- Feature-Flags (zum schnellen Ein/Aus-Schalten) --------
//inline constexpr bool FEATURE_USE_IMU = true;
//inline constexpr bool FEATURE_LOG_DEBUG = true;

// -------- Sanity-Checks zur Compile-Zeit --------
//static_assert(NUM_DI >= 0 && NUM_DO >= 0, "Digitale Kanalzahlen dürfen nicht negativ sein");
//static_assert(NUM_AI >= 0 && NUM_AO >= 0, "Analoge Kanalzahlen dürfen nicht negativ sein");

} // namespace cfg