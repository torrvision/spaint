/**
 * spaintrunner: Application.cpp
 */

#include "Application.h"

#include <spaint/ogl/WrappedGL.h>

//#################### CONSTRUCTORS ####################

Application::Application(const spaint::SpaintEngine_Ptr& spaintEngine)
{
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

  m_window.reset(
    SDL_CreateWindow(
      "Semantic Paint",
      SDL_WINDOWPOS_UNDEFINED,
      SDL_WINDOWPOS_UNDEFINED,
      640,
      480,
      SDL_WINDOW_OPENGL
    ),
    &SDL_DestroyWindow
  );

  m_context.reset(
    SDL_GL_CreateContext(m_window.get()),
    SDL_GL_DeleteContext
  );

  setup();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void Application::run()
{
  for(;;)
  {
    if(!process_events()) return;
    render();
  }
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

bool Application::process_events()
{
  SDL_Event event;
  while(SDL_PollEvent(&event))
  {
    switch(event.type)
    {
      case SDL_KEYDOWN:
        return false;
      default:
        break;
    }
  }

  return true;
}

void Application::render() const
{
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(20.0, -20.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);
  glBegin(GL_LINES);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(50.0, 0.0, 0.0);
    glColor3d(0.0, 1.0, 0.0);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(0.0, 50.0, 0.0);
    glColor3d(0.0, 0.0, 1.0);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(0.0, 0.0, 50.0);
  glEnd();

  SDL_GL_SwapWindow(m_window.get());
}

void Application::setup()
{
  int width = 640;
  int height = 480;
  double fovY = 60.0;
  double zNear = 0.1;
  double zFar = 1000.0;

  glViewport(0, 0, width, height);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(fovY, static_cast<double>(width) / height, zNear, zFar);
}
