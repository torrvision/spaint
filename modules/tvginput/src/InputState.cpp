/**
 * tvginput: InputState.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "InputState.h"

#include <stdexcept>

namespace tvginput {

//#################### CONSTRUCTORS ####################

InputState::InputState()
{
  reset();
}

//#################### PUBLIC METHODS ####################

bool InputState::key_down(Keycode key) const
{
  std::map<Keycode,bool>::const_iterator it = m_keyDown.find(key);
  return it != m_keyDown.end() ? it->second : false;
}

bool InputState::mouse_button_down(MouseButton button) const
{
  std::map<MouseButton,bool>::const_iterator it = m_mouseButtonDown.find(button);
  return it != m_mouseButtonDown.end() ? it->second : false;
}

bool InputState::mouse_position_known() const
{
  return m_mousePositionX != -1.0f;
}

float InputState::mouse_position_x() const
{
  if(m_mousePositionX != -1.0f) return m_mousePositionX;
  else throw std::runtime_error("The mouse position is not yet known");
}

float InputState::mouse_position_y() const
{
  if(m_mousePositionY != -1.0f) return m_mousePositionY;
  else throw std::runtime_error("The mouse position is not yet known");
}

float InputState::mouse_pressed_x(MouseButton button) const
{
  if(m_mousePressedX[button] != -1.0f) return m_mousePressedX[button];
  else throw std::runtime_error("The specified mouse button is not currently pressed");
}

float InputState::mouse_pressed_y(MouseButton button) const
{
  if(m_mousePressedY[button] != -1.0f) return m_mousePressedY[button];
  else throw std::runtime_error("The specified mouse button is not currently pressed");
}

void InputState::press_key(Keycode key)
{
  m_keyDown[key] = true;
}

void InputState::press_mouse_button(MouseButton button, float x, float y)
{
  m_mouseButtonDown[button] = true;
  m_mousePressedX[button] = x;
  m_mousePressedY[button] = y;
}

void InputState::release_key(Keycode key)
{
  m_keyDown[key] = false;
}

void InputState::release_mouse_button(MouseButton button)
{
  m_mouseButtonDown[button] = false;
  m_mousePressedX[button] = m_mousePressedY[button] = -1.0f;
}

void InputState::reset()
{
  set_mouse_position(-1.0f, -1.0f);
  m_mousePressedX = m_mousePressedY = std::vector<float>(MOUSE_BUTTON_LAST, -1.0f);
}

void InputState::set_mouse_position(float x, float y)
{
  m_mousePositionX = x;
  m_mousePositionY = y;
}

}
