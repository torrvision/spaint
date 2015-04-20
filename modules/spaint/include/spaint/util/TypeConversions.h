/**
 * spaint: TypeConversions.h
 */

#ifndef H_PAINT_TYPE_CONVERSIONS
#define H_SPAINT_TYPE_CONVERSIONS

#include <ITMLib/Utils/ITMLibDefines.h>
#include <Eigen/Dense>

namespace spaint {

class TypeConversions
{
public:
  /**
   * \brief Converts an Eigen Vector to an InfiniTAM vector.
   *
   * \param v  The Eigen vector.
   * \return   The InfiniTAM vector.
   */
  static Vector3f to_itm(const Eigen::Vector3f& v);
};

}

#endif
