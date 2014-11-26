/**
 * spaint: ViconTracker.h
 */
 
#ifndef H_SPAINT_VICONTRACKER
#define H_SPAINT_VICONTRACKER

#include <string>
 
#include <ITMLib/Engine/ITMTracker.h>

#include <vicon/Client.h>
 
namespace spaint {
 
/**
 * \brief TODO
 */
class ViconTracker : public ITMLib::Engine::ITMTracker
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** TODO */
  std::string m_subjectName;

  /** The Vicon client. */
  ViconDataStreamSDK::CPP::Client m_vicon;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  ViconTracker(const std::string& host, const std::string& subjectName);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief TODO
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
   * \brief TODO
   */
  int find_subject_index(const std::string& name) const;
};

}

#endif
