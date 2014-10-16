/**
 * spaintrunner: main.cpp
 */

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <Engine/OpenNIEngine.h>
using namespace InfiniTAM::Engine;

#include <SDL.h>

#include <spaint/main/SpaintEngine.h>
#include <spaint/ogl/WrappedGL.h>
using namespace spaint;

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<void> SDL_GLContext_Ptr;
typedef boost::shared_ptr<SDL_Window> SDL_Window_Ptr;

//#################### FUNCTIONS ####################

bool process_events()
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

void quit(const std::string& message, int code = EXIT_FAILURE)
{
  std::cerr << message << '\n';
  SDL_Quit();
  exit(code);
}

void render(const SDL_Window_Ptr& window)
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

  SDL_GL_SwapWindow(window.get());
}

void run(const SDL_Window_Ptr& window)
{
  for(;;)
  {
    if(!process_events()) return;
    render(window);
  }
}

void setup()
{
  double width = 640;
  double height = 480;
  double fovY = 60.0;
  double zNear = 0.1;
  double zFar = 1000.0;

  glViewport(0, 0, width, height);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(fovY, width / height, zNear, zFar);
}

int main(int argc, char *argv[])
{
  // Parse the command-line arguments.
  if (argc > 4)
  {
    // Note: See the InfiniTAM code for argument details (we use the same arguments here for consistency).
    quit("Usage: spaint [<Calibration Filename> [<OpenNI Device URI> | <RGB Image Mask> <Depth Image Mask>]]");
  }

  std::string calibrationFilename = argc >= 2 ? argv[1] : "./Resources/DefaultCalibration.txt",
              openNIDeviceURI = argc == 3 ? argv[2] : "Default",
              rgbImageMask = argc == 4 ? argv[2] : "",
              depthImageMask = argc == 4 ? argv[3] : "";

  // Specify the InfiniTAM settings.
  ITMLibSettings settings;

  // Construct the spaint engine.
  SpaintEngine_Ptr spaintEngine;
  if(argc == 4)
  {
    std::cout << "[spaint] Reading images from disk: " << rgbImageMask << ' ' << depthImageMask << "\n\n";
    spaintEngine.reset(new SpaintEngine(calibrationFilename, rgbImageMask, depthImageMask, settings));
  }
  else
  {
#if WITH_OPENNI
    std::cout << "[spaint] Reading images from OpenNI device: " << openNIDeviceURI << "\n\n";
    spaintEngine.reset(new SpaintEngine(calibrationFilename, openNIDeviceURI == "Default" ? boost::none : boost::optional<std::string>(openNIDeviceURI), settings));
#else
    quit("Error: OpenNI support not currently available. Reconfigure in CMake with the WITH_OPENNI option set to ON.");
#endif
  }

  if(SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    quit("Error: Failed to initialise SDL.");
  }

  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

  SDL_Window_Ptr window(
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

  SDL_GLContext_Ptr context(
    SDL_GL_CreateContext(window.get()),
    SDL_GL_DeleteContext
  );

  setup();

  run(window);

  SDL_Quit();

  return 0;
}
