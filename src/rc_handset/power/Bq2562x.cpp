#include "rc_handset/power/Bq2562x.h"

namespace rc_handset {
namespace power {

namespace {

// Register map for TI BQ25895 (datasheet SLUSC02) -- the actual charger IC
// on this board (rc pcb.pdf, "BQ25895 Charger" block, U11 BQ25895RTWR).
// This driver was previously written against a different TI charger
// family's register layout (addresses up to 0x38, a 16-bit VBAT ADC
// register, a separate ADC "done" status bit). None of that exists on a
// real BQ25895 -- it only defines REG00-REG14, all 8-bit -- which is why
// chip init and battery voltage reads were never succeeding. Verify against
// the datasheet before relying on this for anything safety-relevant; it
// hasn't been checked against a physical copy of it, only working memory
// of the part.
constexpr uint8_t kRegPowerOnConfig = 0x03;      // WD_RST[6], CHG_CONFIG[4]
constexpr uint8_t kRegAdcControl = 0x02;         // CONV_START[7], CONV_RATE[6]
constexpr uint8_t kRegStatus = 0x0B;             // VBUS_STAT[7:5], CHRG_STAT[4:3]
constexpr uint8_t kRegBatteryVoltageAdc = 0x0E;  // THERM_STAT[7], BATV[6:0]
constexpr uint8_t kRegPartInformation = 0x14;    // PN[5:3], DEV_REV[1:0]
constexpr uint8_t kRegBatfetControl = 0x09;      // BATFET_DIS[5] -- ship mode

constexpr uint8_t kWatchdogResetMask = 1u << 6;
constexpr uint8_t kChargeEnableMask = 1u << 4;
constexpr uint8_t kBatfetDisableMask = 1u << 5;
constexpr uint8_t kAdcConvStartMask = 1u << 7;
constexpr uint8_t kAdcConvRateMask = 1u << 6;  // 0 = one-shot, 1 = continuous
constexpr uint8_t kChargeStatusMask = 0x3u << 3;
constexpr uint8_t kVbusStatusMask = 0x7u << 5;
constexpr uint8_t kPartNumberMask = 0x7u << 3;
// One-shot ADC conversion takes ~1s on real hardware per the datasheet, not
// the 100ms this driver previously waited -- that alone was enough to time
// out on every read even with the register addresses fixed.
constexpr uint32_t kAdcTimeoutMs = 1500;
constexpr uint32_t kAdcPollIntervalMs = 50;

uint8_t fieldGet(uint8_t mask, uint8_t value) {
    uint8_t shift = 0;
    while (((mask >> shift) & 1u) == 0u && shift < 8u) ++shift;
    return static_cast<uint8_t>((value & mask) >> shift);
}

}  // namespace

Bq2562x::Bq2562x(Bq2562xBus& bus, uint8_t address) : bus_(bus), address_(address) {}

Bq2562xResult Bq2562x::begin() {
    if (!bus_.probe(address_)) return Bq2562xResult::Io;

    // Confirm the part-info register is actually readable, rather than
    // gating on a specific expected PN value -- that value hasn't been
    // confirmed against a physical datasheet copy, and a wrong expected
    // constant here would reject a genuinely-present chip the exact same
    // way the old wrong register address did.
    uint8_t part = 0;
    Bq2562xResult result = read8(kRegPartInformation, part);
    if (result != Bq2562xResult::Ok) return result;

    return resetWatchdog();
}

Bq2562xResult Bq2562x::resetWatchdog() {
    return update8(kRegPowerOnConfig, kWatchdogResetMask, kWatchdogResetMask);
}

Bq2562xResult Bq2562x::setChargeEnabled(bool enabled) {
    return update8(kRegPowerOnConfig, kChargeEnableMask, enabled ? kChargeEnableMask : 0u);
}

Bq2562xResult Bq2562x::readChargeStatus(Bq2562xChargeStatus& status) {
    uint8_t reg = 0;
    Bq2562xResult result = read8(kRegStatus, reg);
    if (result != Bq2562xResult::Ok) return result;
    status = static_cast<Bq2562xChargeStatus>(fieldGet(kChargeStatusMask, reg));
    return Bq2562xResult::Ok;
}

Bq2562xResult Bq2562x::readVbusStatus(Bq2562xVbusStatus& status) {
    uint8_t reg = 0;
    Bq2562xResult result = read8(kRegStatus, reg);
    if (result != Bq2562xResult::Ok) return result;
    status = static_cast<Bq2562xVbusStatus>(fieldGet(kVbusStatusMask, reg));
    return Bq2562xResult::Ok;
}

Bq2562xResult Bq2562x::startBatteryVoltageConversion() {
    // Trigger a one-shot ADC conversion. CONV_START self-clears once the
    // conversion completes -- there's no separate "done" status bit at a
    // different register the way the old (wrong-chip) driver assumed.
    return update8(kRegAdcControl, kAdcConvStartMask | kAdcConvRateMask, kAdcConvStartMask);
}

Bq2562xResult Bq2562x::isBatteryVoltageConversionDone(bool& done) {
    uint8_t adcControl = kAdcConvStartMask;
    Bq2562xResult result = read8(kRegAdcControl, adcControl);
    if (result != Bq2562xResult::Ok) return result;
    done = (adcControl & kAdcConvStartMask) == 0;
    return Bq2562xResult::Ok;
}

Bq2562xResult Bq2562x::readBatteryVoltageResultMv(uint16_t& voltageMv) {
    uint8_t raw = 0;
    Bq2562xResult result = read8(kRegBatteryVoltageAdc, raw);
    if (result != Bq2562xResult::Ok) return result;
    voltageMv = decodeBatteryVoltageMv(raw);
    return Bq2562xResult::Ok;
}

Bq2562xResult Bq2562x::readBatteryVoltageMv(uint16_t& voltageMv) {
    // Blocking convenience wrapper around the three steps above -- busy-waits
    // for the whole ~1s (worse while actively charging, observed close to
    // kAdcTimeoutMs) conversion inline. Fine for the one-time boot read in
    // Rp2350Power::begin(), before anything is running yet; NOT fine for a
    // per-second poll on the main loop, which is why rcPowerService()'s
    // periodic path drives startBatteryVoltageConversion() /
    // isBatteryVoltageConversionDone() / readBatteryVoltageResultMv()
    // directly instead, spread non-blockingly across service() ticks.
    Bq2562xResult result = startBatteryVoltageConversion();
    if (result != Bq2562xResult::Ok) return result;

    bool done = false;
    for (uint32_t elapsed = 0; elapsed < kAdcTimeoutMs; elapsed += kAdcPollIntervalMs) {
        bus_.delayMs(kAdcPollIntervalMs);
        result = isBatteryVoltageConversionDone(done);
        if (result != Bq2562xResult::Ok) return result;
        if (done) break;
    }
    if (!done) return Bq2562xResult::Timeout;

    return readBatteryVoltageResultMv(voltageMv);
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
    // Deliberately never called -- see the safety note beside its only
    // near-caller (powerOff() in Rp2350Power.cpp) for why: no confirmed
    // hardware path back from ship mode into POWMAN's GPIO wake, so using
    // this risks bricking the board until a physical reset. Kept
    // register-correct for a real BQ25895 (REG09 BATFET_DIS) in case that
    // decision is revisited, not because it's on any current code path.
    return update8(kRegBatfetControl, kBatfetDisableMask, kBatfetDisableMask);
}

Bq2562xResult Bq2562x::readChargerControl3(uint8_t& value) {
    return read8(kRegBatfetControl, value);
}

uint16_t Bq2562x::decodeBatteryVoltageMv(uint16_t rawRegister) {
    // REG0E is a single 8-bit register: THERM_STAT[7], BATV[6:0].
    // mV = 2304 (offset) + BATV * 20 (LSB), per the BQ25895 datasheet.
    const uint16_t batv = static_cast<uint16_t>(rawRegister & 0x7Fu);
    return static_cast<uint16_t>(2304u + batv * 20u);
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
