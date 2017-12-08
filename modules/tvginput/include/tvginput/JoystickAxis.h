/**
 * tvginput: JoystickAxis.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_TVGINPUT_JOYSTICKAXIS
#define H_TVGINPUT_JOYSTICKAXIS

#include <limits>

namespace tvginput {

//#################### ENUMERATIONS ####################

/**
 * \brief The values of this enumeration map SDL's joystick axes to more generic names.
 */
enum JoystickAxis
{
  // PS3 Controller
  PS3_AXIS_ANALOG_LEFT_X = 0,
  PS3_AXIS_ANALOG_LEFT_Y = 1,
  PS3_AXIS_ANALOG_RIGHT_X = 2,
  PS3_AXIS_ANALOG_RIGHT_Y = 3,
  PS3_AXIS_TRIGGER_L2 = 12,
  PS3_AXIS_TRIGGER_R2 = 13,
  PS3_AXIS_TRIGGER_L1 = 14,
  PS3_AXIS_TRIGGER_R1 = 15
};

//#################### FUNCTIONS ####################

/**
 * \brief This function takes an axis state (short, [-32768..32767]) and returns a normalised value in the [0..1] range.
 *
 * \param state The axis state.
 * \return      The normalised value of the axis state.
 */
inline float joystick_normalise_axis_state(short state)
{
  return static_cast<float>(static_cast<int>(state) + 32768) / std::numeric_limits<unsigned short>::max();
}

/**
 * \brief This function takes an axis state (short, [-32768..32767]) and returns a normalised value in the [-1..1] range.
 *
 * \param state The axis state.
 * \return      The normalised value of the axis state.
 */
inline float joystick_normalise_signed_axis_state(short state)
{
  return state < 0
      ? -static_cast<float>(state) / std::numeric_limits<short>::min()
      :  static_cast<float>(state) / std::numeric_limits<short>::max();
}

}

#endif
