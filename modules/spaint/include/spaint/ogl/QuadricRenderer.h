/**
 * spaint: QuadricRenderer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_QUADRICRENDERER
#define H_SPAINT_QUADRICRENDERER

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

#include "WrappedGL.h"

namespace spaint {

/**
 * \brief This class provides utility functions for rendering quadrics at arbitrary positions and orientations in space.
 */
class QuadricRenderer
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Renders a cylinder between the specified base centre and top centre points.
   *
   * \param baseCentre      The centre of the base of the cylinder.
   * \param topCentre       The centre of the top of the cylinder.
   * \param baseRadius      The radius of the base of the cylinder.
   * \param topRadius       The radius of the top of the cylinder.
   * \param slices          The number of subdivisions of the cylinder around its length.
   * \param stacks          The number of subdivisions of the cylinder along its length.
   * \param optionalQuadric An optional GLU quadric to use when rendering the cylinder (if none is specified, one will be created on the fly).
   */
  static void render_cylinder(const Eigen::Vector3f& baseCentre, const Eigen::Vector3f& topCentre,
                              double baseRadius, double topRadius, int slices, int stacks = 1,
                              const boost::optional<boost::shared_ptr<GLUquadric> >& optionalQuadric = boost::none);

  /**
   * \brief Renders a sphere of the specified radius at the specified position.
   *
   * \param centre          The position of the centre of the sphere.
   * \param radius          The radius of the sphere.
   * \param slices          The number of subdivisions of the sphere around its vertical axis (similar to lines of longitude).
   * \param stacks          The number of subdivisions of the sphere along its vertical axis (similar to lines of latitude).
   * \param optionalQuadric An optional GLU quadric to use when rendering the sphere (if none is specified, one will be created on the fly).
   */
  static void render_sphere(const Eigen::Vector3f& centre, double radius, int slices, int stacks,
                            const boost::optional<boost::shared_ptr<GLUquadric> >& optionalQuadric = boost::none);

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Sets the model-view matrix needed to render an oriented quadric at the specified position and orientation.
   *
   * By default, GLU quadrics are positioned at the origin and oriented along the z axis.
   * If we want to draw them elsewhere, we need to set the model-view matrix appropriately.
   *
   * \param p     The desired position of the quadric's anchor point (for cylinders, this is the centre of the base).
   * \param axis  The axis along which the quadric should be oriented.
   */
  static void begin_oriented_quadric(const Eigen::Vector3f& p, const Eigen::Vector3f& axis);

  /**
   * \brief Restores the model-view matrix after rendering an oriented quadric.
   */
  static void end_oriented_quadric();

  /**
   * \brief Gets the quadric to use when rendering.
   *
   * If optionalQuadric is valid, we extract and return the quadric it contains. If not, we create a temporary quadric.
   *
   * \param optionalQuadric An optional GLU quadric to use when rendering.
   * \return                The quadric to use when rendering.
   */
  static boost::shared_ptr<GLUquadric> get_quadric(const boost::optional<boost::shared_ptr<GLUquadric> >& optionalQuadric);
};

}

#endif
