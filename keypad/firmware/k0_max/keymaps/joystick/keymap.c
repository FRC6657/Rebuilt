/* Copyright 2025 @ Keychron (https://www.keychron.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H
#include "keychron_common.h"

enum layers {
    BASE,
};
// clang-format off
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [BASE] = LAYOUT_tenkey_27(
        JS_0,    JS_1,    JS_2,    JS_3,    JS_4,
        JS_5,    JS_6,    JS_7,    JS_8,    JS_9,
        JS_10,   JS_11,   JS_12,   JS_13,   JS_14,
        JS_15,   JS_16,   JS_17,   JS_18,
        JS_19,   JS_20,   JS_21,   JS_22,
        JS_23,   JS_24,            JS_25,   JS_26 ),
};

// clang-format on
#ifdef RGB_MATRIX_ENABLE
bool rgb_matrix_indicators_user(void) {
    rgb_matrix_set_color_all(0, 255, 0);
    return true;
}
#endif

#if defined(ENCODER_MAP_ENABLE)
const uint16_t PROGMEM encoder_map[][NUM_ENCODERS][2] = {
    [BASE] = {ENCODER_CCW_CW(JS_27, JS_28)},
};
#endif // ENCODER_MAP_ENABLE
