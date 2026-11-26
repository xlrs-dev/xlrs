#ifndef UNIT_TEST

#include <Arduino.h>
#include <EEPROM.h>
#include <RadioLib.h>
#include <SPI.h>
#include <pico/time.h>
#include <string.h>
#ifdef XLRS_ENABLE_WIRELESS_BIND
#include <pico/unique_id.h>
#endif

#include "lora_link/protocol.h"
#include "lora_link/rf/scheduler.h"
#include "lora_link/role_store.h"

using namespace lora_link;

#ifdef TXRX_SWITCHABLE
// ---------------------------------------------------------------------------
// Switchable TX/RX module image: one binary contains BOTH role applications,
// each in its own namespace (so their file-static state never collides). The
// role persisted in EEPROM (see lora_link/role_store.h) decides, at boot, which
// one runs. The role code itself (role_app.inl) is exactly what the fixed-role
// firmware runs; only the dispatcher below is new.
// ---------------------------------------------------------------------------
#ifndef TXRX_FW_VERSION
#define TXRX_FW_VERSION "0.2.0"
#endif

#if defined(LORA_TX_ROLE) || defined(LORA_RX_ROLE)
#error "TXRX_SWITCHABLE builds select the role at runtime; do not pass LORA_TX_ROLE/LORA_RX_ROLE"
#endif

namespace txrole {
#define LORA_TX_ROLE 1
#define LORA_RX_ROLE 0
#include "role_app.inl"
#undef LORA_TX_ROLE
#undef LORA_RX_ROLE
}  // namespace txrole

namespace rxrole {
#define LORA_TX_ROLE 0
#define LORA_RX_ROLE 1
#include "role_app.inl"
#undef LORA_TX_ROLE
#undef LORA_RX_ROLE
}  // namespace rxrole

static ModuleRole g_activeRole = ModuleRole::Rx;

static ModuleRole loadStoredRole() {
    // Role used until one is stored: RX, unless built with -DTXRX_DEFAULT_ROLE_TX.
#ifdef TXRX_DEFAULT_ROLE_TX
    ModuleRole role = ModuleRole::Tx;
#else
    ModuleRole role = ModuleRole::Rx;
#endif
    EEPROM.begin(kEepromSize);
    RoleRecord record{};
    EEPROM.get(kRoleRecordOffset, record);
    ModuleRole stored;
    if (readRoleRecord(record, stored)) role = stored;
    EEPROM.end();  // role code re-begins EEPROM for its own config
    return role;
}

void setup() {
    g_activeRole = loadStoredRole();
    if (g_activeRole == ModuleRole::Rx) rxrole::setup();
    else txrole::setup();
}

void loop() {
    if (g_activeRole == ModuleRole::Rx) rxrole::loop();
    else txrole::loop();
}

#else  // fixed-role build (tx_lora_pico, rx_lora_pico, ...): unchanged behaviour

#include "role_app.inl"

#endif  // TXRX_SWITCHABLE

#endif // UNIT_TEST
