#include "rc_handset/power/Bq2562x.h"

namespace rc_handset {
namespace power {

namespace {

constexpr uint8_t kRegChargerControl1 = 0x16;
constexpr uint8_t kRegChargerControl3 = 0x18;
constexpr uint8_t kRegChargerStatus0 = 0x1D;
constexpr uint8_t kRegChargerStatus1 = 0x1E;
constexpr uint8_t kRegAdcControl = 0x26;
constexpr uint8_t kRegVbatAdc = 0x30;
constexpr uint8_t kRegPartInformation = 0x38;

constexpr uint8_t kWatchdogResetMask = 1u << 2;
constexpr uint8_t kChargeEnableMask = 1u << 5;
constexpr uint8_t kBatfetCtrlMask = 0x3u;
constexpr uint8_t kBatfetDelayMask = 1u << 2;
constexpr uint8_t kBatfetCtrlWithVbusMask = 1u << 3;
constexpr uint8_t kAdcEnableMask = 1u << 7;
constexpr uint8_t kAdcOneShotMask = 1u << 6;
constexpr uint8_t kAdcDoneMask = 1u << 6;
constexpr uint8_t kChargeStatusMask = 0x3u << 3;
constexpr uint8_t kVbusStatusMask = 0x7u;
constexpr uint8_t kPartNumberMask = 0x7u << 3;
constexpr uint32_t kAdcTimeoutMs = 100;

uint8_t fieldPrep(uint8_t mask, uint8_t value) {
    uint8_t shift = 0;
    while (((mask >> shift) & 1u) == 0u && shift < 8u) ++shift;
    return static_cast<uint8_t>((value << shift) & mask);
}

uint8_t fieldGet(uint8_t mask, uint8_t value) {
    uint8_t shift = 0;
    while (((mask >> shift) & 1u) == 0u && shift < 8u) ++shift;
    return static_cast<uint8_t>((value & mask) >> shift);
}

}  // namespace

Bq2562x::Bq2562x(Bq2562xBus& bus, uint8_t address) : bus_(bus), address_(address) {}

Bq2562xResult Bq2562x::begin() {
    if (!bus_.probe(address_)) return Bq2562xResult::Io;

    uint8_t part = 0;
    Bq2562xResult result = read8(kRegPartInformation, part);
    if (result != Bq2562xResult::Ok) return result;
    if (fieldGet(kPartNumberMask, part) > 1u) return Bq2562xResult::Io;

    return resetWatchdog();
}

Bq2562xResult Bq2562x::resetWatchdog() {
    return update8(kRegChargerControl1, kWatchdogResetMask, kWatchdogResetMask);
}

Bq2562xResult Bq2562x::setChargeEnabled(bool enabled) {
    return update8(kRegChargerControl1, kChargeEnableMask, enabled ? kChargeEnableMask : 0u);
}

Bq2562xResult Bq2562x::readChargeStatus(Bq2562xChargeStatus& status) {
    uint8_t reg = 0;
    Bq2562xResult result = read8(kRegChargerStatus1, reg);
    if (result != Bq2562xResult::Ok) return result;
    status = static_cast<Bq2562xChargeStatus>(fieldGet(kChargeStatusMask, reg));
    return Bq2562xResult::Ok;
}

Bq2562xResult Bq2562x::readVbusStatus(Bq2562xVbusStatus& status) {
    uint8_t reg = 0;
    Bq2562xResult result = read8(kRegChargerStatus1, reg);
    if (result != Bq2562xResult::Ok) return result;
    status = static_cast<Bq2562xVbusStatus>(fieldGet(kVbusStatusMask, reg));
    return Bq2562xResult::Ok;
}

Bq2562xResult Bq2562x::readBatteryVoltageMv(uint16_t& voltageMv) {
    uint16_t stale = 0;
    Bq2562xResult result = read16(kRegVbatAdc, stale);
    if (result != Bq2562xResult::Ok) return result;

    result = update8(kRegAdcControl, kAdcEnableMask | kAdcOneShotMask,
                     kAdcEnableMask | kAdcOneShotMask);
    if (result != Bq2562xResult::Ok) return result;

    uint8_t status = 0;
    for (uint32_t elapsed = 0; elapsed < kAdcTimeoutMs; ++elapsed) {
        result = read8(kRegChargerStatus0, status);
        if (result != Bq2562xResult::Ok) return result;
        if ((status & kAdcDoneMask) != 0) break;
        bus_.delayMs(1);
    }
    if ((status & kAdcDoneMask) == 0) return Bq2562xResult::Timeout;

    uint16_t raw = 0;
    result = read16(kRegVbatAdc, raw);
    if (result != Bq2562xResult::Ok) return result;
    voltageMv = decodeBatteryVoltageMv(raw);
    return Bq2562xResult::Ok;
}

Bq2562xResult Bq2562x::readSnapshot(BatterySnapshot& snapshot) {
    snapshot = {};
    snapshot.chargerPresent = true;

    uint16_t voltageMv = 0;
    Bq2562xResult firstError = Bq2562xResult::Ok;
    Bq2562xResult result = readBatteryVoltageMv(voltageMv);
    if (result == Bq2562xResult::Ok) {
        snapshot.batteryVoltageValid = true;
        snapshot.batteryVoltageMv = voltageMv;
        snapshot.batteryPercent = batteryPercentFromVoltageMv(voltageMv);
    } else {
        firstError = result;
    }

    Bq2562xChargeStatus charge = Bq2562xChargeStatus::NotCharging;
    result = readChargeStatus(charge);
    if (result == Bq2562xResult::Ok) {
        snapshot.chargeStatus = toBatteryChargeStatus(charge);
    } else if (firstError == Bq2562xResult::Ok) {
        firstError = result;
    }

    Bq2562xVbusStatus vbus = Bq2562xVbusStatus::NoInput;
    result = readVbusStatus(vbus);
    if (result == Bq2562xResult::Ok) {
        snapshot.vbusStatus = toBatteryVbusStatus(vbus);
    } else if (firstError == Bq2562xResult::Ok) {
        firstError = result;
    }

    return firstError;
}

Bq2562xResult Bq2562x::requestShipModeForPowerOff() {
    const uint8_t shipMode = 2u;
    const uint8_t value = fieldPrep(kBatfetCtrlMask, shipMode) | kBatfetCtrlWithVbusMask;
    return update8(kRegChargerControl3,
                   kBatfetCtrlMask | kBatfetDelayMask | kBatfetCtrlWithVbusMask,
                   value);
}

Bq2562xResult Bq2562x::readChargerControl3(uint8_t& value) {
    return read8(kRegChargerControl3, value);
}

uint16_t Bq2562x::decodeBatteryVoltageMv(uint16_t rawRegister) {
    const uint16_t shifted = static_cast<uint16_t>((rawRegister & 0x1FFEu) >> 1);
    return static_cast<uint16_t>((static_cast<uint32_t>(shifted) * 199u + 50u) / 100u);
}

ChargeStatus Bq2562x::toBatteryChargeStatus(Bq2562xChargeStatus status) {
    switch (status) {
    case Bq2562xChargeStatus::NotCharging:
        return ChargeStatus::NotCharging;
    case Bq2562xChargeStatus::ConstantCurrent:
        return ChargeStatus::ConstantCurrent;
    case Bq2562xChargeStatus::ConstantVoltage:
        return ChargeStatus::ConstantVoltage;
    case Bq2562xChargeStatus::TopOff:
        return ChargeStatus::TopOff;
    }
    return ChargeStatus::Unknown;
}

VbusStatus Bq2562x::toBatteryVbusStatus(Bq2562xVbusStatus status) {
    switch (status) {
    case Bq2562xVbusStatus::NoInput:
        return VbusStatus::NoInput;
    case Bq2562xVbusStatus::UsbSdp:
        return VbusStatus::UsbSdp;
    case Bq2562xVbusStatus::UsbCdp:
        return VbusStatus::UsbCdp;
    case Bq2562xVbusStatus::UsbDcp:
        return VbusStatus::UsbDcp;
    case Bq2562xVbusStatus::UnknownAdapter:
        return VbusStatus::UnknownAdapter;
    case Bq2562xVbusStatus::NonStandardAdapter:
        return VbusStatus::NonStandardAdapter;
    case Bq2562xVbusStatus::Hvdcp:
        return VbusStatus::Hvdcp;
    case Bq2562xVbusStatus::OtgMode:
        return VbusStatus::OtgMode;
    }
    return VbusStatus::Unknown;
}

Bq2562xResult Bq2562x::update8(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t current = 0;
    Bq2562xResult result = read8(reg, current);
    if (result != Bq2562xResult::Ok) return result;
    current = static_cast<uint8_t>((current & ~mask) | (value & mask));
    return write8(reg, current);
}

Bq2562xResult Bq2562x::read8(uint8_t reg, uint8_t& value) {
    return bus_.read8(address_, reg, value) ? Bq2562xResult::Ok : Bq2562xResult::Io;
}

Bq2562xResult Bq2562x::write8(uint8_t reg, uint8_t value) {
    return bus_.write8(address_, reg, value) ? Bq2562xResult::Ok : Bq2562xResult::Io;
}

Bq2562xResult Bq2562x::read16(uint8_t reg, uint16_t& value) {
    return bus_.read16(address_, reg, value) ? Bq2562xResult::Ok : Bq2562xResult::Io;
}

}  // namespace power
}  // namespace rc_handset
