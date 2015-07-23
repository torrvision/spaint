/**
 * spaint: WrappedGL.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifdef _WIN32
  #ifndef NOMINMAX
    #define NOMINMAX  // prevent the min and max macros in windows.h being defined (they interfere with the Standard C++ equivalents)
  #endif
  #include <windows.h>
#endif

#ifdef WITH_GLEW
  #include <GL/glew.h>
#endif

#ifndef __APPLE__
  #include <GL/gl.h>
  #include <GL/glu.h>
#else
  #include <OpenGL/gl.h>
  #include <OpenGL/glu.h>
#endif
