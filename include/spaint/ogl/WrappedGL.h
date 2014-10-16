/***
* spaint: WrappedGL.h
***/

#ifdef _WIN32
  #ifndef NOMINMAX
    #define NOMINMAX  // prevent the min and max macros in windows.h being defined (they interfere with the Standard C++ equivalents)
  #endif
  #include <windows.h>
#endif

#ifndef __APPLE__
  #include <GL/gl.h>
  #include <GL/glu.h>
#else
  #include <OpenGL/gl.h>
  #include <OpenGL/glu.h>
#endif
