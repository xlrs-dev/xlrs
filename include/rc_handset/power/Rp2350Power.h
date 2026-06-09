#pragma once

#include "rc_handset/power/Battery.h"

namespace rc_handset {
namespace power {

void rcPowerBegin();
void rcPowerService();
void rcPowerPrintStatus();
BatterySnapshot rcPowerSnapshot();

}  // namespace power
}  // namespace rc_handset
