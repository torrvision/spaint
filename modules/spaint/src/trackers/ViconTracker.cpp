/**
 * spaint: ViconTracker.cpp
 */

#include "trackers/ViconTracker.h"
using namespace ViconDataStreamSDK::CPP;

#include <stdexcept>

namespace spaint {

//#################### CONSTRUCTORS ####################

ViconTracker::ViconTracker(const std::string& host)
{
  // TODO: Validate the IP address.

  if(m_vicon.Connect(host + ":801").Result != Result::Success)
  {
    throw std::runtime_error("Could not connect to the Vicon system");
  }

  m_vicon.EnableMarkerData();
  m_vicon.EnableSegmentData();
  m_vicon.EnableUnlabeledMarkerData();
  m_vicon.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);
  m_vicon.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up);
}

//#################### DESTRUCTOR ####################

ViconTracker::~ViconTracker()
{
  m_vicon.DisableUnlabeledMarkerData();
  m_vicon.DisableSegmentData();
  m_vicon.DisableMarkerData();
  m_vicon.Disconnect();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ViconTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
  // TODO
}

}
