#pragma once

#include "GamepadState.hpp"
#include "HardwareManager/HardwareTypes.hpp"

class ButtonProcessor final {
public:
    static void process_raw_buttons(DigitalButtons& buttons, DPadState& dpad, const NativeButtons& native_buttons, const ExpanderButtons& expander_buttons);
};
