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
  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief The values of this enumeration denote the different tracking modes that the Vicon tracker can use.
   */
  enum TrackingMode
  {
    /**
     * Use the relative transformations from Original Camera ("world") space to Current Camera space as the poses - see comments in TrackCamera.
     * Fail until the relative transformation between Vicon and Original Camera space has been determined.
     */
    TM_ABSOLUTE,

    /**
     * Use the relative transformations from Original Marker space to Current Marker space as the poses - see comments in TrackCamera.
     */
    TM_RELATIVE,
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** The transformation from Original Camera ("world") space to Original Marker space (once computed) - see comments in TrackCamera. */
  boost::optional<Matrix4f> m_aTw;

  /** The ID of the scene for which the tracker will be used. */
  std::string m_sceneID;

  /** The name given in the Vicon software to the subject being tracked. */
  std::string m_subjectName;

  /** The tracking mode (absolute or relative). */
  TrackingMode m_trackingMode;

  /** The Vicon interface. */
  ViconInterface_CPtr m_vicon;

  /** The transformation from Original Marker space to Vicon space (once computed) - see comments in TrackCamera. */
  boost::optional<Matrix4f> m_vTa;

  /** The transformation from Original Marker space to Original Camera ("world") space (once computed) - see comments in TrackCamera. */
  boost::optional<Matrix4f> m_wTa;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a Vicon tracker.
   *
   * \param vicon         The Vicon interface.
   * \param sceneID       The ID of the scene for which the tracker will be used.
   * \param subjectName   The name given in the Vicon software to the subject being tracked.
   * \param trackingMode  The tracking mode (absolute or relative).
   */
  ViconTracker(const ViconInterface_CPtr& vicon, const std::string& sceneID, const std::string& subjectName, TrackingMode trackingMode);

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
