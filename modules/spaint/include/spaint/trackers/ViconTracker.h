/**
 * spaint: ViconTracker.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_VICONTRACKER
#define H_SPAINT_VICONTRACKER

#include <map>
#include <string>

#include <boost/optional.hpp>

#include <Eigen/Dense>

#include <vicon/Client.h>

#include "FallibleTracker.h"

namespace spaint {

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

  /** The Vicon client. */
  ViconDataStreamSDK::CPP::Client m_vicon;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a Vicon tracker.
   *
   * \param host        The host on which the Vicon software is running (e.g. "<IP address>:<port>").
   * \param subjectName The name given to the camera subject in the Vicon software.
   */
  ViconTracker(const std::string& host, const std::string& subjectName);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the Vicon tracker.
   */
  ~ViconTracker();

  //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
  /** Deliberately private and unimplemented. */
  ViconTracker(const ViconTracker&);
  ViconTracker& operator=(const ViconTracker&);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Attempts to get the positions of the markers for the Vicon subject with the specified name.
   *
   * This may fail if we move out of the range of the cameras or some of the markers are occluded.
   *
   * \param subjectName The name of the subject.
   * \return            The positions of the markers for the subject, indexed by name, or boost::none if they are temporarily unavailable.
   */
  boost::optional<std::map<std::string,Eigen::Vector3f> > try_get_marker_positions(const std::string& subjectName) const;
};

}

#endif
