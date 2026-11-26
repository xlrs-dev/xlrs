#pragma once

#include <stdint.h>

#include "rc_handset/power/Battery.h"

namespace rc_handset {
namespace power {

constexpr uint8_t kBq2562xI2cAddress = 0x6A;

enum class Bq2562xResult : int8_t {
    Ok = 0,
    Io = -1,
    Timeout = -2,
    Busy = -3,
    InvalidArgument = -4,
};

enum class Bq2562xChargeStatus : uint8_t {
    NotCharging = 0,
    ConstantCurrent = 1,
    ConstantVoltage = 2,
    TopOff = 3,
};

// Values match BQ25895's VBUS_STAT[7:5] field (REG0B) exactly -- see
// Bq2562x.cpp for why this driver targets a BQ25895 despite its name.
enum class Bq2562xVbusStatus : uint8_t {
    NoInput = 0,
    UsbSdp = 1,
    UsbCdp = 2,
    UsbDcp = 3,
    Hvdcp = 4,
    UnknownAdapter = 5,
    NonStandardAdapter = 6,
    OtgMode = 7,
};

class Bq2562xBus {
public:
    virtual ~Bq2562xBus() = default;
    virtual bool probe(uint8_t address) = 0;
    virtual bool read8(uint8_t address, uint8_t reg, uint8_t& value) = 0;
    virtual bool write8(uint8_t address, uint8_t reg, uint8_t value) = 0;
    virtual bool read16(uint8_t address, uint8_t reg, uint16_t& value) = 0;
    virtual bool write16(uint8_t address, uint8_t reg, uint16_t value) = 0;
    virtual void delayMs(uint32_t ms) = 0;
};

class Bq2562x {
public:
    explicit Bq2562x(Bq2562xBus& bus, uint8_t address = kBq2562xI2cAddress);

    Bq2562xResult begin();
    Bq2562xResult resetWatchdog();
    Bq2562xResult setChargeEnabled(bool enabled);
    Bq2562xResult readChargeStatus(Bq2562xChargeStatus& status);
    Bq2562xResult readVbusStatus(Bq2562xVbusStatus& status);
    Bq2562xResult readBatteryVoltageMv(uint16_t& voltageMv);

    // Non-blocking split of the same one-shot ADC conversion readBatteryVoltageMv()
    // busy-waits for -- see Bq2562x.cpp for why the blocking version is unsuitable
    // for a per-second poll on the main loop. Drive as: start once, then poll
    // isBatteryVoltageConversionDone() on later ticks until it reports done,
    // then call readBatteryVoltageResultMv() exactly once.
    Bq2562xResult startBatteryVoltageConversion();
    Bq2562xResult isBatteryVoltageConversionDone(bool& done);
    Bq2562xResult readBatteryVoltageResultMv(uint16_t& voltageMv);

    Bq2562xResult readSnapshot(BatterySnapshot& snapshot);
    Bq2562xResult requestShipModeForPowerOff();
    Bq2562xResult readChargerControl3(uint8_t& value);

    static uint16_t decodeBatteryVoltageMv(uint16_t rawRegister);
    static ChargeStatus toBatteryChargeStatus(Bq2562xChargeStatus status);
    static VbusStatus toBatteryVbusStatus(Bq2562xVbusStatus status);

private:
    Bq2562xResult update8(uint8_t reg, uint8_t mask, uint8_t value);
    Bq2562xResult read8(uint8_t reg, uint8_t& value);
    Bq2562xResult write8(uint8_t reg, uint8_t value);
    Bq2562xResult read16(uint8_t reg, uint16_t& value);

    Bq2562xBus& bus_;
    uint8_t address_;
};

}  // namespace power
}  // namespace rc_handset
