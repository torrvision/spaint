/**
 * spaint: InputState.h
 */

#ifndef H_SPAINT_INPUTSTATE
#define H_SPAINT_INPUTSTATE

#include <map>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <SDL_keyboard.h>

#include "MouseButton.h"

namespace spaint {

/**
 * \brief An instance of this class maintains the current state of the keyboard and mouse.
 */
class InputState
{
	//#################### PRIVATE VARIABLES ####################
private:
	std::map<SDL_Keycode,bool> m_keyDown;
	std::map<MouseButton,bool> m_mouseButtonDown;
	int m_mouseMotionX, m_mouseMotionY;
	int m_mousePositionX, m_mousePositionY;
	std::vector<int> m_mousePressedX, m_mousePressedY;

	//#################### CONSTRUCTORS ####################
public:
	InputState();

	//#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
	InputState(const InputState&);
	InputState& operator=(const InputState&);

	//#################### PUBLIC METHODS ####################
public:
	bool key_down(SDL_Keycode key) const;
	bool mouse_button_down(MouseButton button) const;
	int mouse_motion_x() const;
	int mouse_motion_y() const;
	bool mouse_position_known() const;
	int mouse_position_x() const;
	int mouse_position_y() const;
	int mouse_pressed_x(MouseButton button) const;
	int mouse_pressed_y(MouseButton button) const;
	void press_key(SDL_Keycode key);
	void press_mouse_button(MouseButton button, int x, int y);
	void release_key(SDL_Keycode key);
	void release_mouse_button(MouseButton button);
	void reset();
	void set_mouse_motion(int x, int y);
	void set_mouse_position(int x, int y);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<InputState> InputState_Ptr;
typedef boost::shared_ptr<const InputState> InputState_CPtr;

}

#endif
