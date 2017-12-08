/**
 * tvginput: JoystickButton.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_TVGINPUT_JOYSTICKBUTTON
#define H_TVGINPUT_JOYSTICKBUTTON

namespace tvginput {

/**
 * \brief The values of this enumeration map SDL's joystick buttons to more generic names.
 */
enum JoystickButton
{
  // PS3 Controller
  PS3_BUTTON_SELECT = 0,
  PS3_BUTTON_START = 3,
  PS3_BUTTON_TRIANGLE = 12,
  PS3_BUTTON_CIRCLE = 13,
  PS3_BUTTON_X = 14,
  PS3_BUTTON_SQUARE = 15
};

}

#endif
