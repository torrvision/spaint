/**
 * spaint: SimpleFiducial.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SIMPLEFIDUCIAL
#define H_SPAINT_SIMPLEFIDUCIAL

#include "Fiducial.h"

namespace spaint {

/**
 * \brief An instance of this class represents a simple fiducial that only uses the most recent measurement it has been given.
 */
class SimpleFiducial : public Fiducial
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a simple fiducial.
   *
   * \param id    The ID of the fiducial.
   * \param pose  The pose of the fiducial in world space.
   */
  SimpleFiducial(const std::string& id, const ORUtils::SE3Pose& pose);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void integrate(const FiducialMeasurement& measurement);
};

}

#endif
