/**
 * spaint: QuadricRenderer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "ogl/QuadricRenderer.h"

#include <rigging/SimpleCamera.h>
using namespace rigging;

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

void QuadricRenderer::render_cylinder(const Eigen::Vector3f& baseCentre, const Eigen::Vector3f& topCentre,
                                      double baseRadius, double topRadius, int slices, int stacks,
                                      const boost::optional<boost::shared_ptr<GLUquadric> >& optionalQuadric)
{
  boost::shared_ptr<GLUquadric> quadric = get_quadric(optionalQuadric);

  Eigen::Vector3f axis = topCentre - baseCentre;
  if(axis.norm() < 1e-3f) return;

  begin_oriented_quadric(baseCentre, axis);
  gluCylinder(quadric.get(), baseRadius, topRadius, axis.norm(), slices, stacks);
  end_oriented_quadric();
}

void QuadricRenderer::render_sphere(const Eigen::Vector3f& centre, double radius, int slices, int stacks,
                                    const boost::optional<boost::shared_ptr<GLUquadric> >& optionalQuadric)
{
  boost::shared_ptr<GLUquadric> quadric = get_quadric(optionalQuadric);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTranslatef(centre.x(), centre.y(), centre.z());

  gluSphere(quadric.get(), radius, slices, stacks);

  glPopMatrix();
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

void QuadricRenderer::begin_oriented_quadric(const Eigen::Vector3f& p, const Eigen::Vector3f& axis)
{
  Eigen::Vector3f n = axis.normalized();
  Eigen::Vector3f up(0.0f, -1.0f, 0.0f);
  SimpleCamera camera(p, n, up);
  const Eigen::Vector3f& u = camera.u();
  const Eigen::Vector3f& v = camera.v();

  Eigen::Matrix4f m;
  m(0,0) = -u.x(); m(0,1) = -v.x(); m(0,2) = n.x(); m(0,3) = p.x();
  m(1,0) = -u.y(); m(1,1) = -v.y(); m(1,2) = n.y(); m(1,3) = p.y();
  m(2,0) = -u.z(); m(2,1) = -v.z(); m(2,2) = n.z(); m(2,3) = p.z();
  m(3,0) = m(3,1) = m(3,2) = 0.0f;
  m(3,3) = 1.0f;

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glMultMatrixf(m.data());
}

void QuadricRenderer::end_oriented_quadric()
{
  glPopMatrix();
}

boost::shared_ptr<GLUquadric> QuadricRenderer::get_quadric(const boost::optional<boost::shared_ptr<GLUquadric> >& optionalQuadric)
{
  return optionalQuadric ? *optionalQuadric : boost::shared_ptr<GLUquadric>(gluNewQuadric(), &gluDeleteQuadric);
}

}
