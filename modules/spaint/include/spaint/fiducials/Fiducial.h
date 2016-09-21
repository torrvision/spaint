/**
 * spaint: Fiducial.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_FIDUCIAL
#define H_SPAINT_FIDUCIAL

#include <boost/shared_ptr.hpp>

namespace spaint {

/**
 * \brief An instance of this class represents a fiducial (a reference marker in a 3D scene).
 */
class Fiducial
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The ID of the fiducial. */
  std::string m_id;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the ID of the fiducial.
   *
   * \return  The ID of the fiducial.
   */
  const std::string& id() const;

  /**
   * \brief Updates the fiducial based on information from a new measurement.
   *
   * \param newFiducial         The fiducial containing the new measurement.
   * \throws std::runtime_error If the two fiducials do not have the same ID.
   */
  void update(const Fiducial& newFiducial);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<Fiducial> Fiducial_Ptr;
typedef boost::shared_ptr<const Fiducial> Fiducial_CPtr;

}

#endif
