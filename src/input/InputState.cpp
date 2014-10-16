/**
 * spaint: InputState.cpp
 */

#include "input/InputState.h"

#include <exception>

namespace spaint {

//#################### CONSTRUCTORS ####################
InputState::InputState()
{
	reset();
}

//#################### PUBLIC METHODS ####################
bool InputState::key_down(SDL_Keycode key) const
{
  std::map<SDL_Keycode,bool>::const_iterator it = m_keyDown.find(key);
  return it != m_keyDown.end() ? it->second : false;
}

bool InputState::mouse_button_down(MouseButton button) const
{
  std::map<MouseButton,bool>::const_iterator it = m_mouseButtonDown.find(button);
  return it != m_mouseButtonDown.end() ? it->second : false;
}

int InputState::mouse_motion_x() const
{
	return m_mouseMotionX;
}

int InputState::mouse_motion_y() const
{
	return m_mouseMotionY;
}

bool InputState::mouse_position_known() const
{
	return m_mousePositionX != -1;
}

int InputState::mouse_position_x() const
{
	if(m_mousePositionX != -1) return m_mousePositionX;
	else throw std::runtime_error("The mouse position is not yet known");
}

int InputState::mouse_position_y() const
{
	if(m_mousePositionY != -1) return m_mousePositionY;
	else throw std::runtime_error("The mouse position is not yet known");
}

int InputState::mouse_pressed_x(MouseButton button) const
{
	if(m_mousePressedX[button] != -1) return m_mousePressedX[button];
	else throw std::runtime_error("The specified mouse button is not currently pressed");
}

int InputState::mouse_pressed_y(MouseButton button) const
{
	if(m_mousePressedY[button] != -1) return m_mousePressedY[button];
	else throw std::runtime_error("The specified mouse button is not currently pressed");
}

void InputState::press_key(SDL_Keycode key)
{
	m_keyDown[key] = true;
}

void InputState::press_mouse_button(MouseButton button, int x, int y)
{
	m_mouseButtonDown[button] = true;
	m_mousePressedX[button] = x;
	m_mousePressedY[button] = y;
}

void InputState::release_key(SDL_Keycode key)
{
	m_keyDown[key] = false;
}

void InputState::release_mouse_button(MouseButton button)
{
	m_mouseButtonDown[button] = false;
	m_mousePressedX[button] = m_mousePressedY[button] = -1;
}

void InputState::reset()
{
	set_mouse_motion(0, 0);
	set_mouse_position(-1, -1);
	m_mousePressedX = m_mousePressedY = std::vector<int>(MOUSE_BUTTON_LAST, -1);
}

void InputState::set_mouse_motion(int x, int y)
{
	m_mouseMotionX = x;
	m_mouseMotionY = y;
}

void InputState::set_mouse_position(int x, int y)
{
	m_mousePositionX = x;
	m_mousePositionY = y;
}

}
