/**
 * spaint: CameraRenderer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "ogl/CameraRenderer.h"

#include <rigging/SimpleCamera.h>
using namespace rigging;

#include "ogl/QuadricRenderer.h"

namespace spaint {

void CameraRenderer::render_camera(const Camera& cam, AxesType axesType, float axisScale, const boost::optional<Vector3f>& bodyColour, double bodyScale)
{
  const Eigen::Vector3f n = cam.n() * axisScale, p = cam.p(), u = cam.u() * axisScale, v = cam.v() * axisScale;

  // If a body colour was specified, render the camera's body as a wireframe sphere.
  if(bodyColour)
  {
    glColor3f(bodyColour->r, bodyColour->g, bodyColour->b);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    QuadricRenderer::render_sphere(p, bodyScale, 10, 10);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }

  // Render the camera's axes.
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTranslatef(p.x(), p.y(), p.z());

  glBegin(GL_LINES);
  switch(axesType)
  {
    case AXES_NUV:
      glColor3f(1.0f, 1.0f, 0.0f);  glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(u.x(), u.y(), u.z());
      glColor3f(0.0f, 1.0f, 1.0f);  glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(v.x(), v.y(), v.z());
      glColor3f(1.0f, 0.0f, 1.0f);  glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(n.x(), n.y(), n.z());
      break;
    default:  // AXES_XYZ
      glColor3f(1.0f, 0.0f, 0.0f);  glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(-u.x(), -u.y(), -u.z());
      glColor3f(0.0f, 1.0f, 0.0f);  glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(-v.x(), -v.y(), -v.z());
      glColor3f(0.0f, 0.0f, 1.0f);  glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(n.x(), n.y(), n.z());
      break;
  }
  glEnd();

  glPopMatrix();
}

void CameraRenderer::render_default_camera(AxesType axesType, float axisScale, const boost::optional<Vector3f>& bodyColour, double bodyScale)
{
  // FIXME: The default axes are also specified in Subwindow::reset_camera - the commonality should be factored out.
  SimpleCamera cam(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f), Eigen::Vector3f(0.0f, -1.0f, 0.0f));
  render_camera(cam, axesType, axisScale, bodyColour, bodyScale);
}

}
