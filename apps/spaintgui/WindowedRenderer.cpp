/**
 * spaintgui: WindowedRenderer.cpp
 */

#include "WindowedRenderer.h"

#include <spaint/cameras/Camera.h>
#include <spaint/ogl/WrappedGL.h>

#include <ITMLib/Utils/ITMMath.h>

//#################### CONSTRUCTORS ####################

WindowedRenderer::WindowedRenderer(const spaint::SpaintEngine_Ptr& spaintEngine, const std::string& title, int width, int height,
                                   const spaint::InputState_CPtr& inputState)
: Renderer(spaintEngine), m_inputState(inputState)
{
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

  m_image.reset(new ITMUChar4Image(spaintEngine->get_image_source_engine()->getDepthImageSize(), false));
  glGenTextures(1, &m_textureID);
}

//#################### DESTRUCTOR ####################

WindowedRenderer::~WindowedRenderer()
{
  glDeleteTextures(1, &m_textureID);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void WindowedRenderer::render() const
{
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

#if 1
  static spaint::Camera camera(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f), Eigen::Vector3f(0.0f, -1.0f, 0.0f));
  const float SPEED = 0.1f;
  const float ANGULAR_SPEED = 0.05f;
  if(m_inputState->key_down(SDLK_w)) camera.move_n(SPEED);
  if(m_inputState->key_down(SDLK_s)) camera.move_n(-SPEED);
  if(m_inputState->key_down(SDLK_d)) camera.move_u(-SPEED);
  if(m_inputState->key_down(SDLK_a)) camera.move_u(SPEED);
  if(m_inputState->key_down(SDLK_q)) camera.move_v(SPEED);
  if(m_inputState->key_down(SDLK_e)) camera.move_v(-SPEED);

  Eigen::Vector3f up(0.0f, -1.0f, 0.0f);
  if(m_inputState->key_down(SDLK_RIGHT)) camera.rotate(up, -ANGULAR_SPEED);
  if(m_inputState->key_down(SDLK_LEFT)) camera.rotate(up, ANGULAR_SPEED);
  if(m_inputState->key_down(SDLK_UP)) camera.rotate(camera.u(), ANGULAR_SPEED);
  if(m_inputState->key_down(SDLK_DOWN)) camera.rotate(camera.u(), -ANGULAR_SPEED);
#endif

#if 0
  m_spaintEngine->get_default_raycast(m_image);
#else
  ITMPose pose = m_spaintEngine->get_pose();
  //pose.params.each.tx = tx;
  //pose.params.each.ty = ty;
  //pose.params.each.tz = tz;
  //pose.SetModelViewFromParams();
  //std::cout << pose.M << '\n';
  Eigen::Vector3f n = camera.n(), p = camera.p(), u = camera.u(), v = camera.v();
  pose.R(0,0) = -u.x();  pose.R(1,0) = -u.y();  pose.R(2,0) = -u.z();
  pose.R(0,1) = -v.x();  pose.R(1,1) = -v.y();  pose.R(2,1) = -v.z();
  pose.R(0,2) = n.x();  pose.R(1,2) = n.y();  pose.R(2,2) = n.z();
  pose.T.x = p.dot(u);
  pose.T.y = p.dot(v);
  pose.T.z = -p.dot(n);
  pose.SetParamsFromModelView();
  pose.SetModelViewFromParams();
  m_spaintEngine->generate_free_raycast(m_image, pose);
#endif

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

  const ITMIntrinsics& intrinsics = m_spaintEngine->get_intrinsics();
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  {
    glLoadIdentity();
    double fx = intrinsics.projectionParamsSimple.fx / (640.0 / 2.0);
    double fy = intrinsics.projectionParamsSimple.fy / (480.0 / 2.0);
    double cx = -(intrinsics.projectionParamsSimple.px - 640.0 / 2.0) / (640.0 / 2.0);
    double cy = -(intrinsics.projectionParamsSimple.py - 480.0 / 2.0) / (480.0 / 2.0);
    double nearVal = 0.1;
    double farVal = 1000.0;
    double leftVal = nearVal / fx * (cx - 1.0);
    //double leftVal = (-1.0 /*- intrinsics.projectionParamsSimple.px*/) * nearVal / intrinsics.projectionParamsSimple.fx;
    double rightVal = nearVal / fx * (cx + 1.0);
    //double rightVal = (1.0 /*- intrinsics.projectionParamsSimple.px*/) * nearVal / intrinsics.projectionParamsSimple.fx;
    double bottomVal = nearVal / fy * (cy - 1.0);
    double topVal = nearVal / fy * (cy + 1.0);
    //double bottomVal = (-1.0/* - intrinsics.projectionParamsSimple.py*/) * nearVal / intrinsics.projectionParamsSimple.fy;
    //double topVal = (1.0/* - intrinsics.projectionParamsSimple.py*/) * nearVal / intrinsics.projectionParamsSimple.fy;
    //glFrustum(2.0f * leftVal * 640.0f, 2.0f * rightVal * 640.0f, bottomVal * 480.0f, topVal * 480.f, nearVal, farVal);
    glFrustum(leftVal, rightVal, bottomVal, topVal, nearVal, farVal);
    //gluPerspective(45.0, 640.0/480.0, 0.1, 1000.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    {
      glLoadIdentity();
      const Matrix3f& R = pose.R;
      const Vector3f& T = pose.T;
      float m[] = {
        R(0,0),
        R(0,1),
        R(0,2),
        0.0,
        R(1,0),
        R(1,1),
        R(1,2),
        0.0,
        R(2,0),
        R(2,1),
        R(2,2),
        0.0,
        T.x,
        T.y,
        T.z,
        1.0
      };

      //std::cout << pose.M << '\n';
      gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);
      glMultMatrixf(m);

      glGetFloatv(GL_PROJECTION_MATRIX, m);
      for(int i = 0; i < 16; ++i)
      {
        int x = i % 4;
        int y = i / 4;
        std::cout << m[x * 4 + y] << ' ';
        if(i % 4 == 3) std::cout << '\n';
      }
      std::cout << '\n';
      //gluLookAt(10.0, 10.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

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

  SDL_GL_SwapWindow(m_window.get());
}
