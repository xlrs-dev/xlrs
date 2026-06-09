#pragma once

#include <stdint.h>

namespace rc_handset {
namespace power {

struct PowerButtonConfig {
    bool holdToBootEnabled = true;
    uint32_t powerOnHoldMs = 5000;
    uint32_t powerOffHoldMs = 2500;
};

enum class BootGateState : uint8_t {
    Waiting = 0,
    Satisfied,
    Aborted,
};

enum class PowerOffState : uint8_t {
    Idle = 0,
    Holding,
    ReadyToRelease,
};

enum class PowerButtonEvent : uint8_t {
    None = 0,
    BootHoldSatisfied,
    BootHoldAborted,
    PowerOffStarted,
    PowerOffProgress,
    PowerOffReady,
    PowerOffCancelled,
    PowerOffConfirmed,
};

struct PowerButtonUpdate {
    PowerButtonEvent event = PowerButtonEvent::None;
    BootGateState bootGateState = BootGateState::Waiting;
    PowerOffState powerOffState = PowerOffState::Idle;
    uint8_t progressPercent = 0;
    bool powerUiActive = false;
};

class PowerButtonController {
public:
    explicit PowerButtonController(PowerButtonConfig config = {});

    void resetBootGate(uint32_t nowMs);
    PowerButtonUpdate updateBootGate(bool pressed, uint32_t nowMs);

    void resetPowerOff();
    PowerButtonUpdate updatePowerOff(bool pressed, uint32_t nowMs);

    BootGateState bootGateState() const { return bootGateState_; }
    PowerOffState powerOffState() const { return powerOffState_; }

private:
    uint8_t percent(uint32_t elapsedMs, uint32_t requiredMs) const;

    PowerButtonConfig config_;
    uint32_t bootStartMs_ = 0;
    BootGateState bootGateState_ = BootGateState::Waiting;
    uint32_t powerOffStartMs_ = 0;
    PowerOffState powerOffState_ = PowerOffState::Idle;
};

}  // namespace power
}  // namespace rc_handset
