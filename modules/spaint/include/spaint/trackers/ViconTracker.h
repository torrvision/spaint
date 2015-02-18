/**
 * spaint: ViconTracker.h
 */

#ifndef H_SPAINT_VICONTRACKER
#define H_SPAINT_VICONTRACKER

#include <map>
#include <string>

#include <boost/optional.hpp>
 
#include <Eigen/Dense>

#include <ITMLib/Engine/ITMTracker.h>

#include <vicon/Client.h>

namespace spaint {

/**
 * \brief An instance of this class can be used to track the camera pose using a Vicon system.
 */
class ViconTracker : public ITMLib::Engine::ITMTracker
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** A flag recording whether or not we have temporarily lost tracking. */
  bool m_lostTracking;

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
  /**
   * \brief Gets whether or not we have temporarily lost tracking.
   *
   * Tracking is most commonly lost due to either moving out of the range of the cameras or marker occlusion.
   *
   * \return  true, if we have temporarily lost tracking, or false otherwise.
   */
  bool lost_tracking() const;

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
