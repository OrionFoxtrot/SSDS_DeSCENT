#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#define Print_rxPin PB7
#define Print_txPin PB6
#define GPS_rxPin PC1
#define GPS_txPin PC0
#include <SoftwareSerial.h>

extern HardwareSerial Print_tx_rx;
extern SoftwareSerial GPS_tx_rx;

// tbd
constexpr uint32_t DUTY_CYCLE_PERIOD_MS = 20000; // 20s total → 10s on, 10s off
// constexpr float GROUND_ACCEL_THRESHOLD = 0.3f;   // m/s², tune to your IMU scale
// constexpr uint32_t GROUND_CONFIRM_MS = 3000;
// constexpr float GROUND_ACCEL_THRESHOLD = 0.3f; // m/s² — reasonable for "at rest"
// constexpr float GROUND_GYRO_THRESHOLD = 0.05f; // rad/s ≈ 3 deg/s — reasonable for "not tumbling"
// constexpr float ALT_DROP_THRESHOLD = 0.5f;     // meters — if altitude dropped by more than this since last check, still falling
// constexpr uint32_t GROUND_CONFIRM_MS = 3000;
#endif