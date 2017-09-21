/**
 * tvginput: Joystick.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_TVGINPUT_JOYSTICK
#define H_TVGINPUT_JOYSTICK

#include <stdint.h>
#include <limits>

namespace tvginput {

/**
 * \brief An enum mapping SDL's Joystick axes to mnemonic names.
 */
enum JoystickAxis
{
  JOYSTICK_AXIS_ANALOG_LEFT_X = 0,
  JOYSTICK_AXIS_ANALOG_LEFT_Y = 1,
  JOYSTICK_AXIS_ANALOG_RIGHT_X = 2,
  JOYSTICK_AXIS_ANALOG_RIGHT_Y = 3,
  JOYSTICK_AXIS_TRIGGER_L2 = 12,
  JOYSTICK_AXIS_TRIGGER_R2 = 13,
  JOYSTICK_AXIS_TRIGGER_L1 = 14,
  JOYSTICK_AXIS_TRIGGER_R1 = 15
};

/**
 * \brief An enum mapping SDL's Joystick buttons to mnemonic names.
 */
enum JoystickButton
{
  JOYSTICK_BUTTON_SELECT = 0,
  JOYSTICK_BUTTON_START = 3,
  JOYSTICK_BUTTON_TRIANGLE = 12,
  JOYSTICK_BUTTON_CIRCLE = 13,
  JOYSTICK_BUTTON_X = 14,
  JOYSTICK_BUTTON_SQUARE = 15
};

/**
 * \brief This function takes an axis state (int16, [-32768..32767]) and returns a normalised value in the [-1..1] range.
 *
 * \return The normalised value of the axis state.
 */
inline float joystick_normalise_signed_axis_state(int16_t state)
{
  return state < 0
      ? -static_cast<float>(state) / std::numeric_limits<int16_t>::min()
      :  static_cast<float>(state) / std::numeric_limits<int16_t>::max();
}

/**
 * \brief This function takes an axis state (int16, [-32768..32767]) and returns a normalised value in the [0..1] range.
 *
 * \return The normalised value of the axis state.
 */
inline float joystick_normalise_axis_state(int16_t state)
{
  return static_cast<float>(static_cast<int32_t>(state) + 32768) / std::numeric_limits<uint16_t>::max();
}

}

#endif
