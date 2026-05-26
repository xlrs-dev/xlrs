#pragma once

#include "link/Link.h"

namespace xlrs::app {

inline bool persistenceAllowed(LinkState state, bool outputActive) {
    if (outputActive) return false;
    return state != LinkState::Connected && state != LinkState::Failsafe;
}

} // namespace xlrs::app
