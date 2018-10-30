/**
 * itmx: ViconTracker.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_ITMX_VICONTRACKER
#define H_ITMX_VICONTRACKER

#include <string>

#include "FallibleTracker.h"
#include "../util/ViconInterface.h"

namespace itmx {

/**
 * \brief An instance of this class can be used to track the camera pose using a Vicon system.
 *
 * Note that the Vicon tracker is capable of detecting tracking failures. These generally occur
 * if we move out of range of the cameras or occlude the markers.
 */
class ViconTracker : public FallibleTracker
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The name given to the camera subject in the Vicon software. */
  std::string m_subjectName;

  /** The Vicon interface. */
  ViconInterface_CPtr m_vicon;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a Vicon tracker.
   *
   * \param vicon       The Vicon interface.
   * \param subjectName The name given to the camera subject in the Vicon software.
   */
  ViconTracker(const ViconInterface_CPtr& vicon, const std::string& subjectName);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual bool requiresColourRendering() const;

  /** Override */
  virtual bool requiresDepthReliability() const;

  /** Override */
  virtual bool requiresPointCloudRendering() const;

  /** Override */
  virtual void TrackCamera(ITMLib::ITMTrackingState *trackingState, const ITMLib::ITMView *view);
};

}

#endif
