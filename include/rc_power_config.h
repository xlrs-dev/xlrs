/**
 * RC power / boot options (RP2350 “Dhanush”: POWER_BUTTON_PIN hold-to-power-on).
 *
 * To boot immediately without holding the power button through the ~5 s splash:
 *   - rc-rp2350 / rc-rp2350-debug enable this via platformio.ini by default
 *     (USB-powered boards often leave QON floating HIGH → normal gate would deep-sleep).
 *   - Or set RC_SKIP_POWER_ON_SEQUENCE to 1 below, or pass -DRC_SKIP_POWER_ON_SEQUENCE=1.
 *
 * For production Dhanush (battery + QON), remove -DRC_SKIP_POWER_ON_SEQUENCE=1 from
 * platformio.ini and rely on the hold-to-boot gate.
 *
 * Power-off (long press) and deep-sleep wakeup on the same GPIO are unchanged.
 */
#pragma once

#ifndef RC_SKIP_POWER_ON_SEQUENCE
#define RC_SKIP_POWER_ON_SEQUENCE 0
#endif
