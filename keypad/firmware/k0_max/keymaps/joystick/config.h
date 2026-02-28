#pragma once

#define JOYSTICK_BUTTON_COUNT 29  // adjust to match your key count
#define JOYSTICK_AXIS_COUNT 0

// Increase encoder step hold time so joystick button presses are long
// enough for applications polling at 60fps (~16ms) to reliably detect them.
#undef ENCODER_MAP_KEY_DELAY
#define ENCODER_MAP_KEY_DELAY 30