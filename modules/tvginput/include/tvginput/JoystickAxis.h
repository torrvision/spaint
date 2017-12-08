/**
 * tvginput: JoystickAxis.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_TVGINPUT_JOYSTICKAXIS
#define H_TVGINPUT_JOYSTICKAXIS

namespace tvginput {

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

}

#endif
