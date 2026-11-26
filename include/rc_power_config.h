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

#ifndef RC_POWER_BUTTON_PIN
#define RC_POWER_BUTTON_PIN 22
#endif

#ifndef RC_POWER_ON_HOLD_MS
// TESTING: shortened from 5000u to 2000u to speed up boot-screen iteration.
// Revert to 5000u before real/production use.
#define RC_POWER_ON_HOLD_MS 2000u
#endif

#ifndef RC_POWER_OFF_HOLD_MS
#define RC_POWER_OFF_HOLD_MS 3000u
#endif

#ifndef RC_POWER_I2C_SDA_PIN
#define RC_POWER_I2C_SDA_PIN 4
#endif

#ifndef RC_POWER_I2C_SCL_PIN
#define RC_POWER_I2C_SCL_PIN 5
#endif

#ifndef RC_POWER_I2C_HZ
#define RC_POWER_I2C_HZ 100000u
#endif

#ifndef RC_POWER_BATTERY_POLL_MS
#define RC_POWER_BATTERY_POLL_MS 1000u
#endif

// Buzzer feedback. Assumed to be a self-oscillating "active" buzzer (drive
// HIGH to sound, LOW to stop) -- the standard part for a simple confirmation
// tone. If it's a passive piezo needing a driven frequency instead, this
// needs a square-wave drive, not a steady digitalWrite.
#ifndef RC_BUZZER_PIN
#define RC_BUZZER_PIN 24
#endif

// Power-on / power-off confirmation: one long beep. Blocking (see call sites)
// -- power-off in particular must not return until the buzzer is silenced,
// since nothing services it once deep sleep is entered.
#ifndef RC_BUZZER_POWER_BEEP_MS
#define RC_BUZZER_POWER_BEEP_MS 1000u
#endif

// Connected/disconnected beeps -- paused for now (too noisy during bench
// testing). Flip to 1 to re-enable; serviceConnectionBuzzer() itself is
// unchanged, just not called while this is 0. See rc_rp2350_crsf_main.cpp.
#ifndef RC_BUZZER_CONNECTION_ENABLED
#define RC_BUZZER_CONNECTION_ENABLED 0
#endif

// RX link just came up: one confirmation beep, non-blocking (serviced from
// the main loop -- see buzzerServiceNonBlocking in rc_rp2350_crsf_main.cpp).
#ifndef RC_BUZZER_CONNECTED_BEEP_MS
#define RC_BUZZER_CONNECTED_BEEP_MS 1000u
#endif

// RX link not up: short chirp repeated at RC_BUZZER_DISCONNECTED_INTERVAL_MS,
// also non-blocking. Stops entirely once connected.
#ifndef RC_BUZZER_DISCONNECTED_CHIRP_MS
#define RC_BUZZER_DISCONNECTED_CHIRP_MS 100u
#endif

#ifndef RC_BUZZER_DISCONNECTED_INTERVAL_MS
#define RC_BUZZER_DISCONNECTED_INTERVAL_MS 3000u
#endif

// OLED screen navigation buttons (PUSH1/CN3, PUSH2/CN4 on the PCB). Plain
// momentary switches to GND, read with an internal pull-up -- LOW = pressed.
// PUSH1 = next screen, PUSH2 = previous screen; see serviceScreenNavButtons
// in rc_rp2350_crsf_main.cpp.
#ifndef RC_PUSH1_PIN
#define RC_PUSH1_PIN 13
#endif

#ifndef RC_PUSH2_PIN
#define RC_PUSH2_PIN 14
#endif

#ifndef RC_NAV_BUTTON_DEBOUNCE_MS
#define RC_NAV_BUTTON_DEBOUNCE_MS 30u
#endif

// How long manual navigation sticks before the display resumes auto-rotating
// through the screens on its own.
#ifndef RC_NAV_MANUAL_TIMEOUT_MS
#define RC_NAV_MANUAL_TIMEOUT_MS 15000u
#endif

// Wireless "soft bind" (XLRS_ENABLE_WIRELESS_BIND builds only). How long
// PUSH1+PUSH2 must be held together on the Bind screen before release arms a
// WirelessBindStart request -- see serviceBindHoldGesture in
// rc_rp2350_crsf_main.cpp.
#ifndef RC_BIND_HOLD_MS
#define RC_BIND_HOLD_MS 2000u
#endif

// Must stay >= TX's kBindBroadcastWindowMs (src/main.cpp) -- this is only the
// RC-side give-up point for its own status polling, not what stops TX.
#ifndef RC_BIND_WINDOW_MS
#define RC_BIND_WINDOW_MS 30000u
#endif

#ifndef RC_BIND_POLL_INTERVAL_MS
#define RC_BIND_POLL_INTERVAL_MS 1000u
#endif

#ifndef RC_BUZZER_BIND_START_MS
#define RC_BUZZER_BIND_START_MS 150u
#endif

#ifndef RC_BUZZER_BIND_SUCCESS_MS
#define RC_BUZZER_BIND_SUCCESS_MS 400u
#endif

#ifndef RC_BUZZER_BIND_FAIL_MS
#define RC_BUZZER_BIND_FAIL_MS 800u
#endif
