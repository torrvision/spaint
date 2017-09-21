/**
 * tvginput: InputState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGINPUT_INPUTSTATE
#define H_TVGINPUT_INPUTSTATE

#include <map>
#include <vector>

#include "Joystick.h"
#include "Keycode.h"
#include "MouseButton.h"

namespace tvginput {

/**
 * \brief An instance of this class maintains the current state of the keyboard and mouse.
 */
class InputState
{
  //#################### PRIVATE VARIABLES ####################
private:
  std::map<JoystickAxis,int16_t> m_joystickAxisState;
  std::map<JoystickButton,bool> m_joystickButtonDown;
  std::map<Keycode,bool> m_keyDown;
  std::map<MouseButton,bool> m_mouseButtonDown;
  float m_mousePositionX, m_mousePositionY;
  std::vector<float> m_mousePressedX, m_mousePressedY;

  //#################### CONSTRUCTORS ####################
public:
  InputState();

  //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
  InputState(const InputState&);
  InputState& operator=(const InputState&);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  int16_t joystick_axis_state(JoystickAxis axis) const;
  bool joystick_button_down(JoystickButton button) const;
  bool key_down(Keycode key) const;
  bool mouse_button_down(MouseButton button) const;
  bool mouse_position_known() const;
  float mouse_position_x() const;
  float mouse_position_y() const;
  float mouse_pressed_x(MouseButton button) const;
  float mouse_pressed_y(MouseButton button) const;
  void press_joystick_button(JoystickButton button);
  void press_key(Keycode key);
  void press_mouse_button(MouseButton button, float x, float y);
  void release_joystick_button(JoystickButton button);
  void release_key(Keycode key);
  void release_mouse_button(MouseButton button);
  void reset();
  void set_joystick_axis_state(JoystickAxis axis, int16_t value);
  void set_mouse_position(float x, float y);
};

}

#endif
