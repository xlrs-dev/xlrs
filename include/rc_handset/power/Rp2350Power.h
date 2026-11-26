#pragma once

#include <stdint.h>

#include "rc_handset/power/Battery.h"

namespace rc_handset {
namespace power {

// Called immediately before the RP2350 enters deep sleep (power-off confirmed,
// or the boot hold was released early). Lets the caller drop peripherals such
// as the OLED into their own low-power state first; deep sleep only gates the
// RP2350 itself, not other I2C devices on the bus.
void rcPowerSetSleepHook(void (*hook)());

// onBootHoldProgress, if given, is called on every poll of the hold-to-boot
// gate with the current hold percent (0..100) and whether it has reached the
// "release to confirm" point, so the caller can drive a boot-progress
// display. Boot only completes once the button is released after reaching
// 100%, mirroring the power-off hold-then-release confirmation below.
void rcPowerBegin(void (*onBootHoldProgress)(uint8_t percent, bool readyToRelease) = nullptr);

// onPowerOffProgress, if given, is called on every poll while a power-off
// hold is active, with the current hold percent (0..100), whether it has
// reached the "release to confirm" point, and whether it's being refused
// because rxLinkActive is true, so the caller can drive a power-off countdown
// (or blocked) display.
//
// rxLinkActive: pass true whenever the handset has a live RX link. Powering
// off while true is refused -- see PowerButtonController::updatePowerOff.
void rcPowerService(void (*onPowerOffProgress)(uint8_t percent, bool readyToRelease, bool blocked) = nullptr,
                    bool rxLinkActive = false);

// True while a power-off hold is in progress (from the most recent
// rcPowerService() call) -- callers should skip their own normal display
// updates while this is true, so the power-off countdown owns the screen.
bool rcPowerOffHoldActive();

void rcPowerPrintStatus();
BatterySnapshot rcPowerSnapshot();

}  // namespace power
}  // namespace rc_handset
