#include <Arduino.h>
#include "../../config.h"
#include "../../constants.h"

#if CHIPSAT_GPS_DRIVER == CHIPSAT_DRIVER_OWN

#include "../Gps.h"
#include "../../log/Log.h"

namespace ChipSatDevices
{

using ChipSatPlatform::Status;
using namespace ChipSatConfig;
using namespace ChipSatConstants;

static uint8_t frame[kNavPvtLength + 8];   // one NAV-PVT, framed
static uint16_t framePayloadLength = 0;
static uint8_t frameClass = 0;
static uint8_t frameId = 0;

Gps::Gps(ChipSatPlatform::Uart &port, ChipSatPlatform::System &system)
  : port_(port), system_(system)
{
}

void Gps::sendUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t length)
{
  const uint8_t header[4] = {cls, id, static_cast<uint8_t>(length), static_cast<uint8_t>(length >> 8)};

  // Fletcher-8 over the header and payload, everything between the sync bytes and the checksum
  uint8_t sum[2] = {0, 0};
  for (uint16_t i = 0; i < sizeof header; ++i) {
    sum[0] = static_cast<uint8_t>(sum[0] + header[i]);
    sum[1] = static_cast<uint8_t>(sum[1] + sum[0]);
  }
  for (uint16_t i = 0; i < length; ++i) {
    sum[0] = static_cast<uint8_t>(sum[0] + payload[i]);
    sum[1] = static_cast<uint8_t>(sum[1] + sum[0]);
  }

  const uint8_t sync[2] = {kUbxSync1, kUbxSync2};
  port_.write(sync, sizeof sync);
  port_.write(header, sizeof header);
  if (length != 0) {
    port_.write(payload, length);
  }
  port_.write(sum, sizeof sum);
}

// Reads one whole frame into the static buffer. Frames longer than the buffer are skipped, not
// truncated, so a corrupt length costs us the wait and nothing else
bool Gps::readUbx(uint32_t deadlineMs)
{
  enum class Step : uint8_t { Sync1, Sync2, Class, Id, LenLo, LenHi, Payload, CkA, CkB };
  Step step = Step::Sync1;
  uint16_t index = 0;
  uint16_t length = 0;
  bool skipping = false;
  uint8_t ckA = 0, ckB = 0;
  uint8_t sumA = 0, sumB = 0;

  while (system_.nowMs() < deadlineMs) {
    const int next = port_.read();
    if (next < 0) {
      continue;
    }
    const uint8_t byte = static_cast<uint8_t>(next);

    switch (step) {
      case Step::Sync1:
        step = (byte == kUbxSync1) ? Step::Sync2 : Step::Sync1;
        break;
      case Step::Sync2:
        step = (byte == kUbxSync2) ? Step::Class : Step::Sync1;
        break;
      case Step::Class:
        frameClass = byte;
        sumA = byte;
        sumB = byte;
        step = Step::Id;
        break;
      case Step::Id:
        frameId = byte;
        sumA = static_cast<uint8_t>(sumA + byte);
        sumB = static_cast<uint8_t>(sumB + sumA);
        step = Step::LenLo;
        break;
      case Step::LenLo:
        length = byte;
        sumA = static_cast<uint8_t>(sumA + byte);
        sumB = static_cast<uint8_t>(sumB + sumA);
        step = Step::LenHi;
        break;
      case Step::LenHi:
        length |= static_cast<uint16_t>(byte) << 8;
        sumA = static_cast<uint8_t>(sumA + byte);
        sumB = static_cast<uint8_t>(sumB + sumA);
        skipping = length > sizeof frame;
        index = 0;
        step = (length == 0) ? Step::CkA : Step::Payload;
        break;
      case Step::Payload:
        if (!skipping) {
          frame[index] = byte;
        }
        sumA = static_cast<uint8_t>(sumA + byte);
        sumB = static_cast<uint8_t>(sumB + sumA);
        if (++index >= length) {
          step = Step::CkA;
        }
        break;
      case Step::CkA:
        ckA = byte;
        step = Step::CkB;
        break;
      case Step::CkB:
        ckB = byte;
        if (ckA == sumA && ckB == sumB && !skipping) {
          framePayloadLength = length;
          return true;
        }
        step = Step::Sync1;   // bad checksum or a frame we could not hold, resynchronise
        break;
    }
  }

  return false;
}

// Sends one frame and waits for the matching reply. Anything else that arrives is parsed and dropped
bool Gps::waitForUbx(uint8_t cls, uint8_t id, uint16_t maxWaitMs)
{
  const uint32_t deadlineMs = system_.nowMs() + maxWaitMs;
  while (system_.nowMs() < deadlineMs) {
    if (!readUbx(deadlineMs)) {
      return false;
    }
    if (frameClass == cls && frameId == id) {
      // An acknowledgement carries the class and id it answers, so check it is ours
      if (cls != kUbxClassAck || (framePayloadLength == 2 && frame[0] == kUbxClassCfg && frame[1] == kUbxIdValset)) {
        return true;
      }
    }
    if (frameClass == kUbxClassAck && frameId == kUbxIdNak) {
      return false;
    }
  }
  return false;
}

// One CFG-VALSET, one key, one value, then wait for the acknowledgement
bool Gps::setKey(uint32_t key, uint32_t value, uint8_t valueBytes)
{
  uint8_t payload[12] = {0};
  payload[0] = 0x00;                 // version
  payload[1] = kUbxConfigLayers;     // RAM + battery backed
  payload[4] = static_cast<uint8_t>(key);
  payload[5] = static_cast<uint8_t>(key >> 8);
  payload[6] = static_cast<uint8_t>(key >> 16);
  payload[7] = static_cast<uint8_t>(key >> 24);
  for (uint8_t i = 0; i < valueBytes; ++i) {
    payload[8 + i] = static_cast<uint8_t>(value >> (8 * i));
  }

  while (port_.read() >= 0) {
    // drop anything already queued so the acknowledgement is the next thing we see
  }

  sendUbx(kUbxClassCfg, kUbxIdValset, payload, static_cast<uint16_t>(8 + valueBytes));
  return waitForUbx(kUbxClassAck, kUbxIdAck, kGpsAckWaitMs);
}

// CFG-RST is the one message the receiver does not acknowledge: it acts on it and restarts, so
// there is nothing to wait for. Clearing only the stored position keeps the almanac and ephemeris,
// so the next fix is still a warm start
void Gps::clearStoredPosition()
{
  const uint8_t payload[4] = {static_cast<uint8_t>(kUbxBbrMaskPositionOnly),
                              static_cast<uint8_t>(kUbxBbrMaskPositionOnly >> 8),
                              kUbxResetModeGnssOnly, 0x00};
  sendUbx(kUbxClassCfg, kUbxIdReset, payload, sizeof payload);
  system_.waitMs(kGpsResetSettleMs);

  while (port_.read() >= 0) {
    // anything the receiver said while restarting is from before the wipe
  }

  LOG_I(Gps, "wipe") { line.fieldHex("bbr", kUbxBbrMaskPositionOnly, 4); }
}

bool Gps::configureAirborne()
{
  return setKey(kUbxCfgNavspgDynmodel, kGpsDynamicModel, 1);
}

bool Gps::configureNormalContinuous()
{
  rateOkay_ = setKey(kUbxCfgRateMeas, kGpsMeasurementIntervalMs, 2);
  navOkay_ = setKey(kUbxCfgRateNav, kGpsNavigationRate, 2);
  modeOkay_ = setKey(kUbxCfgPmOperateMode, kGpsOperateModeFull, 1);
  return rateOkay_ && navOkay_ && modeOkay_;
}

Status Gps::begin()
{
  LOG_I(Gps, "start");
  const uint32_t startMs = system_.nowMs();

  if (kGpsClearStoredPosition) {
    clearStoredPosition();
  }

  for (uint8_t attempt = 0; attempt < kInitAttempts; attempt++) {
    // Separate bools so every step still runs if one fails
    const bool airborneOkay = configureAirborne();
    const bool normalModeOkay = configureNormalContinuous();

    ready_ = airborneOkay && normalModeOkay;

    if (ready_) {
      LOG_I(Gps, "init") {
        line.field("ok", true);
        line.field("attempts", attempt + 1);
        line.field("dyn", airborneOkay);
        line.field("rate", rateOkay_);
        line.field("nav", navOkay_);
        line.field("mode", modeOkay_);
        line.field("dynmodel", kGpsDynamicModel);
        line.field("ms", system_.nowMs() - startMs);
      }
      return Status::Ok;
    }

    system_.waitMs(kInitRetryDelayMs);
  }

  // No retry left, setup halts on this
  LOG_E(Gps, "init") {
    line.field("ok", false);
    line.field("reason", "config");
    line.field("attempts", kInitAttempts);
    line.field("dyn", false);
    line.field("rate", rateOkay_);
    line.field("nav", navOkay_);
    line.field("mode", modeOkay_);
    line.field("ms", system_.nowMs() - startMs);
  }
  return Status::NoAck;
}

static int32_t readS32(const uint8_t *from)
{
  return static_cast<int32_t>(static_cast<uint32_t>(from[0]) | static_cast<uint32_t>(from[1]) << 8 |
                              static_cast<uint32_t>(from[2]) << 16 | static_cast<uint32_t>(from[3]) << 24);
}

Status Gps::read(uint16_t maxWaitMs)
{
  if (!ready_) {
    return Status::NotReady;
  }

  fixType_ = 0;
  llhChecked_ = false;
  invalidLlh_ = false;

  while (port_.read() >= 0) {
    // an old frame would answer the poll below with stale numbers
  }

  sendUbx(kUbxClassNav, kUbxIdNavPvt, nullptr, 0);

  const bool arrived = waitForUbx(kUbxClassNav, kUbxIdNavPvt, maxWaitMs);
  if (!arrived || framePayloadLength != kNavPvtLength) {
    data_.valid = false;   // old coordinates stay
    LOG_W(Gps, "read") {
      line.field("ok", false);
      line.field("reason", arrived ? "short" : "pvt");   // wrong shape, or nothing came back
    }
    return arrived ? Status::BadData : Status::Timeout;
  }

  const int32_t longitudeE7 = readS32(&frame[kNavPvtLon]);
  const int32_t latitudeE7 = readS32(&frame[kNavPvtLat]);
  const int32_t altitudeMm = readS32(&frame[kNavPvtHeightMsl]);
  satellites_ = frame[kNavPvtNumSatellites];

  // Airborne models don't do 2D fixes ("No 2D position fixes supported", DYN_MODEL_AIRBORNE4g in the u-blox library header)
  fixType_ = frame[kNavPvtFixType];
  if (fixType_ >= 3) {
    const uint16_t flags3 = static_cast<uint16_t>(frame[kNavPvtFlags3]) |
                            static_cast<uint16_t>(frame[kNavPvtFlags3 + 1]) << 8;
    invalidLlh_ = (flags3 & kNavPvtInvalidLlh) != 0;
    llhChecked_ = true;
  }

  const bool inRange = latitudeE7 >= -kMaxLatitudeE7 && latitudeE7 <= kMaxLatitudeE7 &&
                       longitudeE7 >= -kMaxLongitudeE7 && longitudeE7 <= kMaxLongitudeE7 &&
                       altitudeMm >= kMinAltitudeMm && altitudeMm <= kMaxAltitudeMm;
  const bool trustworthy = fixType_ >= 3 && !invalidLlh_ && satellites_ >= kGpsMinSatellites && inRange;

  // Only a fix we trust replaces the stored position. Anything else leaves the last good one in
  // place, marked invalid, so the packet never carries a converging solution as if it were real
  if (trustworthy) {
    data_.longitudeE7 = longitudeE7;
    data_.latitudeE7 = latitudeE7;
    data_.altitudeMSLmm = altitudeMm;
    data_.updatedMs = system_.nowMs();
    data_.valid = true;
    return Status::Ok;
  }

  data_.valid = false;
  LOG_W(Gps, "read") {
    line.field("ok", false);
    line.field("reason", inRange ? "nofix" : "range");
    line.field("fix", fixType_);
    line.field("sats", satellites_);
  }
  return Status::BadData;
}

} // namespace ChipSatDevices

#endif
