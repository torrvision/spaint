/**
 * spaint: AveragingFiducial.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_AVERAGINGFIDUCIAL
#define H_SPAINT_AVERAGINGFIDUCIAL

#include "Fiducial.h"

namespace spaint {

/**
 * \brief An instance of this class represents a fiducial that averages the measurements that it is given over time.
 */
class AveragingFiducial : public Fiducial
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an averaging fiducial.
   *
   * \param id    The ID of the fiducial.
   * \param pose  The pose of the fiducial in world space.
   */
  AveragingFiducial(const std::string& id, const ORUtils::SE3Pose& pose);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void integrate_sub(const FiducialMeasurement& measurement);
};

}

#endif
