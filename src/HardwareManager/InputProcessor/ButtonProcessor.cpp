#include "HardwareManager/InputProcessor/ButtonProcessor.hpp"

void ButtonProcessor::process_raw_buttons(DigitalButtons& buttons, DPadState& dpad, const NativeButtons& native_buttons, const ExpanderButtons& expander_buttons) {
    buttons.a = expander_buttons.a;
    buttons.b = expander_buttons.b;
    buttons.x = expander_buttons.x;
    buttons.y = expander_buttons.y;

    buttons.lb = native_buttons.lb;
    buttons.rb = native_buttons.rb;

    buttons.ls = native_buttons.ls;
    buttons.rs = native_buttons.rs;

    buttons.start = native_buttons.start;
    buttons.back = native_buttons.back;
    buttons.home = native_buttons.home;

    buttons.mode = expander_buttons.mode;
    buttons.vibr = expander_buttons.vibr;

    buttons.xd_switch = native_buttons.xd_switch;

    if (expander_buttons.dpad_up && !expander_buttons.dpad_right && !expander_buttons.dpad_down && !expander_buttons.dpad_left) {
        dpad = DPadState::Up;
    } else if (expander_buttons.dpad_up && expander_buttons.dpad_right && !expander_buttons.dpad_down && !expander_buttons.dpad_left) {
        dpad = DPadState::UpRight;
    } else if (!expander_buttons.dpad_up && expander_buttons.dpad_right && !expander_buttons.dpad_down && !expander_buttons.dpad_left) {
        dpad = DPadState::Right;
    } else if (!expander_buttons.dpad_up && expander_buttons.dpad_right && expander_buttons.dpad_down && !expander_buttons.dpad_left) {
        dpad = DPadState::DownRight;
    } else if (!expander_buttons.dpad_up && !expander_buttons.dpad_right && expander_buttons.dpad_down && !expander_buttons.dpad_left) {
        dpad = DPadState::Down;
    } else if (!expander_buttons.dpad_up && !expander_buttons.dpad_right && expander_buttons.dpad_down && expander_buttons.dpad_left) {
        dpad = DPadState::DownLeft;
    } else if (!expander_buttons.dpad_up && !expander_buttons.dpad_right && !expander_buttons.dpad_down && expander_buttons.dpad_left) {
        dpad = DPadState::Left;
    } else if (expander_buttons.dpad_up && !expander_buttons.dpad_right && !expander_buttons.dpad_down && expander_buttons.dpad_left) {
        dpad = DPadState::UpLeft;
    } else {
        dpad = DPadState::Centered;
    }
}
