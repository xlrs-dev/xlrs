#include "rc_handset/power/PowerButton.h"

namespace rc_handset {
namespace power {

PowerButtonController::PowerButtonController(PowerButtonConfig config) : config_(config) {}

void PowerButtonController::resetBootGate(uint32_t nowMs) {
    bootStartMs_ = nowMs;
    bootGateState_ = config_.holdToBootEnabled ? BootGateState::Waiting : BootGateState::Satisfied;
}

PowerButtonUpdate PowerButtonController::updateBootGate(bool pressed, uint32_t nowMs) {
    PowerButtonUpdate out{};
    out.bootGateState = bootGateState_;
    out.powerOffState = powerOffState_;

    if (!config_.holdToBootEnabled) {
        bootGateState_ = BootGateState::Satisfied;
        out.bootGateState = bootGateState_;
        out.progressPercent = 100;
        return out;
    }
    if (bootGateState_ != BootGateState::Waiting) {
        out.bootGateState = bootGateState_;
        out.progressPercent = bootGateState_ == BootGateState::Satisfied ? 100 : 0;
        return out;
    }
    if (!pressed) {
        bootGateState_ = BootGateState::Aborted;
        out.bootGateState = bootGateState_;
        out.event = PowerButtonEvent::BootHoldAborted;
        return out;
    }

    const uint32_t elapsed = nowMs - bootStartMs_;
    out.progressPercent = percent(elapsed, config_.powerOnHoldMs);
    out.powerUiActive = true;
    if (elapsed >= config_.powerOnHoldMs) {
        bootGateState_ = BootGateState::Satisfied;
        out.bootGateState = bootGateState_;
        out.progressPercent = 100;
        out.event = PowerButtonEvent::BootHoldSatisfied;
    }
    return out;
}

void PowerButtonController::resetPowerOff() {
    powerOffStartMs_ = 0;
    powerOffState_ = PowerOffState::Idle;
}

PowerButtonUpdate PowerButtonController::updatePowerOff(bool pressed, uint32_t nowMs) {
    PowerButtonUpdate out{};
    out.bootGateState = bootGateState_;
    out.powerOffState = powerOffState_;

    if (pressed) {
        if (powerOffState_ == PowerOffState::Idle) {
            powerOffStartMs_ = nowMs;
            powerOffState_ = PowerOffState::Holding;
            out.event = PowerButtonEvent::PowerOffStarted;
        }
        const uint32_t elapsed = nowMs - powerOffStartMs_;
        out.progressPercent = percent(elapsed, config_.powerOffHoldMs);
        out.powerUiActive = true;
        if (elapsed >= config_.powerOffHoldMs) {
            if (powerOffState_ != PowerOffState::ReadyToRelease) {
                out.event = PowerButtonEvent::PowerOffReady;
            } else if (out.event == PowerButtonEvent::None) {
                out.event = PowerButtonEvent::PowerOffProgress;
            }
            powerOffState_ = PowerOffState::ReadyToRelease;
            out.progressPercent = 100;
        } else if (out.event == PowerButtonEvent::None) {
            out.event = PowerButtonEvent::PowerOffProgress;
        }
        out.powerOffState = powerOffState_;
        return out;
    }

    if (powerOffState_ == PowerOffState::ReadyToRelease) {
        out.event = PowerButtonEvent::PowerOffConfirmed;
    } else if (powerOffState_ == PowerOffState::Holding) {
        out.event = PowerButtonEvent::PowerOffCancelled;
    }
    resetPowerOff();
    out.powerOffState = powerOffState_;
    out.progressPercent = 0;
    return out;
}

uint8_t PowerButtonController::percent(uint32_t elapsedMs, uint32_t requiredMs) const {
    if (requiredMs == 0 || elapsedMs >= requiredMs) return 100;
    return static_cast<uint8_t>((elapsedMs * 100u) / requiredMs);
}

}  // namespace power
}  // namespace rc_handset
