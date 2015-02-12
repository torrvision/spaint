/**
 * spaint: ViconTracker.h
 */
 
#ifndef H_SPAINT_VICONTRACKER
#define H_SPAINT_VICONTRACKER

#include <map>
#include <string>
 
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
  /** The name given to the camera subject in the Vicon software. */
  std::string m_subjectName;

  /** The Vicon client. */
  ViconDataStreamSDK::CPP::Client m_vicon;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a Vicon tracker.
   *
   * \param host        The IP address of the host on which the Vicon software is running.
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
   * \brief Gets the positions of the markers for the Vicon subject with the specified name.
   *
   * \param subjectName The name of the subject.
   * \return            The positions of the markers for the subject, indexed by name.
   */
  std::map<std::string,Eigen::Vector3f> get_marker_positions(const std::string& subjectName) const;
};

}

#endif
