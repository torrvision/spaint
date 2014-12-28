/**
 * spaintgui: WindowedRenderer.cpp
 */

#include "WindowedRenderer.h"

#include <stdexcept>

#include <rigging/SimpleCamera.h>
using namespace rigging;

#include <spaint/ogl/WrappedGL.h>
#include <spaint/util/CameraPoseConverter.h>
using namespace spaint;

//#################### CONSTRUCTORS ####################

WindowedRenderer::WindowedRenderer(const SpaintEngine_Ptr& spaintEngine, const std::string& title, int width, int height)
: Renderer(spaintEngine), m_height(height), m_width(width)
{
  // Create the window into which to render.
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

  m_window.reset(
    SDL_CreateWindow(
      title.c_str(),
      SDL_WINDOWPOS_UNDEFINED,
      SDL_WINDOWPOS_UNDEFINED,
      width,
      height,
      SDL_WINDOW_OPENGL
    ),
    &SDL_DestroyWindow
  );

  m_context.reset(
    SDL_GL_CreateContext(m_window.get()),
    SDL_GL_DeleteContext
  );

  glViewport(0, 0, width, height);

  // Set up the camera.
  m_camera.reset(new SimpleCamera(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f), Eigen::Vector3f(0.0f, -1.0f, 0.0f)));

  // Set up the image and texture needed to render the reconstructed scene.
  m_image.reset(new ITMUChar4Image(spaintEngine->get_image_source_engine()->getDepthImageSize(), false));
  glGenTextures(1, &m_textureID);
}

//#################### DESTRUCTOR ####################

WindowedRenderer::~WindowedRenderer()
{
  glDeleteTextures(1, &m_textureID);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

rigging::MoveableCamera_Ptr WindowedRenderer::get_camera()
{
  return m_camera;
}

void WindowedRenderer::render() const
{
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Determine the camera pose.
  ITMPose pose;
  switch(m_cameraMode)
  {
    case CM_FOLLOW:
      pose = m_spaintEngine->get_pose();
      break;
    case CM_FREE:
      pose = CameraPoseConverter::camera_to_pose(*m_camera);
      break;
    default:
      // This should never happen.
      throw std::runtime_error("Error: Unknown camera mode");
  }

  // Render the reconstructed scene, then render a synthetic scene over the top of it.
  render_reconstructed_scene(pose);
  render_synthetic_scene(pose);

  SDL_GL_SwapWindow(m_window.get());
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

void WindowedRenderer::render_reconstructed_scene(const ITMPose& pose) const
{
  // Raycast the scene.
  switch(m_cameraMode)
  {
    case CM_FOLLOW:
      m_spaintEngine->get_default_raycast(m_image);
      break;
    case CM_FREE:
      m_spaintEngine->generate_free_raycast(m_image, pose);
      break;
    default:
      // This should never happen.
      throw std::runtime_error("Error: Unknown camera mode");
  }

  // Draw a quad textured with the raycasted scene.
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  {
    glLoadIdentity();
    glOrtho(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    {
      glLoadIdentity();

      glEnable(GL_TEXTURE_2D);
      {
        glBindTexture(GL_TEXTURE_2D, m_textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_image->noDims.x, m_image->noDims.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_image->GetData(false));
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glColor3f(1.0f, 1.0f, 1.0f);
        glBegin(GL_QUADS);
        {
          glTexCoord2f(0, 1); glVertex2f(0, 0);
          glTexCoord2f(1, 1); glVertex2f(1, 0);
          glTexCoord2f(1, 0); glVertex2f(1, 1);
          glTexCoord2f(0, 0); glVertex2f(0, 1);
        }
        glEnd();
      }
      glDisable(GL_TEXTURE_2D);
    }
    glPopMatrix();
  }
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
}

void WindowedRenderer::render_synthetic_scene(const ITMPose& pose) const
{
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  {
    set_projection_matrix(m_spaintEngine->get_intrinsics(), m_width, m_height);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    {
      set_modelview_matrix(pose);

      // Render the axes.
      glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f);  glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(1.0f, 0.0f, 0.0f);
        glColor3f(0.0f, 1.0f, 0.0f);  glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, 1.0f, 0.0f);
        glColor3f(0.0f, 0.0f, 1.0f);  glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, 0.0f, 1.0f);
      glEnd();
    }
    glPopMatrix();
  }
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
}

void WindowedRenderer::set_modelview_matrix(const ITMPose& pose)
{
  glLoadIdentity();

  // Note: InfiniTAM uses a right-handed coordinate system with z pointing into the screen.
  gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);

  // Post-multiply the current model-view matrix with the pose matrix.
  float m[16];
  int i = 0;
  for(int x = 0; x < 4; ++x)
  {
    for(int y = 0; y < 4; ++y)
    {
      m[i++] = pose.M(x,y);
    }
  }
  glMultMatrixf(m);
}

void WindowedRenderer::set_projection_matrix(const ITMIntrinsics& intrinsics, int width, int height)
{
  // TODO: Comment this.
  double halfWidth = width / 2.0;
  double halfHeight = height / 2.0;
  double fx = intrinsics.projectionParamsSimple.fx / halfWidth;
  double fy = intrinsics.projectionParamsSimple.fy / halfHeight;
  double cx = (halfWidth - intrinsics.projectionParamsSimple.px) / halfWidth;
  double cy = (halfHeight - intrinsics.projectionParamsSimple.py) / halfHeight;

  double nearVal = 0.1;
  double farVal = 1000.0;
  double leftVal = (cx - 1.0) * nearVal / fx;
  double rightVal = (cx + 1.0) * nearVal / fx;
  double bottomVal = (cy - 1.0) * nearVal / fy;
  double topVal = (cy + 1.0) * nearVal / fy;

  glLoadIdentity();
  glFrustum(leftVal, rightVal, bottomVal, topVal, nearVal, farVal);
}
