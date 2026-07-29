#pragma once

#include <Arduino.h>

// Central project-wide debug selector.
// Change ChipSatDebug::level in the main .ino file only.
namespace ChipSatDebug {

enum Level : uint8_t {
  kNone = 0,        // No prints.
  kBoot = 1,        // Boot and initialization prints only.
  kSensorData = 2,  // Compact sensor data only.
  kAll = 3,         // Every print category.
  kPacket = 4,      // Packet and telemetry prints only.
  kErrors = 5,      // Errors, resets, warnings, and failures only.
  kRadio = 6        // Radio/TX results and cadence only.
};

// Defined once in the main .ino file.
extern uint8_t level;

inline bool enabled(Level selectedLevel) {
  return (level == kAll) || (level == selectedLevel);
}

inline bool boot() {
  return enabled(kBoot);
}

inline bool sensorData() {
  return enabled(kSensorData);
}

inline bool packet() {
  return enabled(kPacket);
}

inline bool errors() {
  return enabled(kErrors);
}

inline bool radio() {
  return enabled(kRadio);
}

inline bool allOnly() {
  return level == kAll;
}

inline bool bootOrErrors() {
  return boot() || errors();
}

inline bool radioOrErrors() {
  return radio() || errors();
}

}  // namespace ChipSatDebug
