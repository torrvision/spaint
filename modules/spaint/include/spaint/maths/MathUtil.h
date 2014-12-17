/**
 * spaint: MathUtil.h
 */

#ifndef H_SPAINT_MATHUTIL
#define H_SPAINT_MATHUTIL

#include <Eigen/Dense>

namespace spaint {

/**
 * \brief This class contains mathematical utility functions.
 */
struct MathUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Rotates vector v anticlockwise about the specified axis by the specified angle (in radians).
   *
   * \param v     The vector to rotate about the axis.
   * \param angle The angle (in radians) by which to rotate it).
   * \param axis  The axis about which to rotate it (must have a length >= 1e-3).
   * \return      A new vector containing the result of the rotation.
   */
  Eigen::Vector3f rotate_about_axis(const Eigen::Vector3f& v, float angle, const Eigen::Vector3f& axis);
};

}

#endif
