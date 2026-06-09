#pragma once

#include "rc_handset/display/DisplayState.h"

namespace rc_handset {
namespace display {

class OledDisplay {
public:
    bool begin();
    void show(const DisplayState& state);
    bool available() const { return available_; }

private:
    bool available_ = false;
};

} // namespace display
} // namespace rc_handset
