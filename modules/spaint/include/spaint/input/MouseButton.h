/**
 * spaint: MouseButton.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_MOUSEBUTTON
#define H_SPAINT_MOUSEBUTTON

#include <SDL_mouse.h>

namespace spaint {

enum MouseButton
{
	MOUSE_BUTTON_LEFT = SDL_BUTTON_LEFT,
	MOUSE_BUTTON_MIDDLE = SDL_BUTTON_MIDDLE,
	MOUSE_BUTTON_RIGHT = SDL_BUTTON_RIGHT,
	MOUSE_BUTTON_LAST
};

}

#endif
