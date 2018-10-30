/**
 * itmx: ViconInterface.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "util/ViconInterface.h"
using namespace ViconDataStreamSDK::CPP;

namespace itmx {

//#################### CONSTRUCTORS ####################

ViconInterface::ViconInterface(const std::string& host)
{
  // Connect to the Vicon system.
  if(m_vicon.Connect(host).Result != Result::Success || !m_vicon.IsConnected().Connected)
  {
    throw std::runtime_error("Could not connect to the Vicon system");
  }

  // Set up the Vicon client.
  m_vicon.EnableMarkerData();
  m_vicon.EnableSegmentData();
  m_vicon.EnableUnlabeledMarkerData();
  m_vicon.SetAxisMapping(Direction::Right, Direction::Down, Direction::Forward);
  m_vicon.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);
}

//#################### DESTRUCTOR ####################

ViconInterface::~ViconInterface()
{
  m_vicon.DisableMarkerData();
  m_vicon.DisableSegmentData();
  m_vicon.DisableUnlabeledMarkerData();
  m_vicon.Disconnect();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

unsigned int ViconInterface::get_frame_number() const
{
  return m_vicon.GetFrameNumber().FrameNumber;
}

boost::optional<std::map<std::string,Eigen::Vector3f> > ViconInterface::try_get_marker_positions(const std::string& subjectName) const
{
  // If there's no frame currently available, early out.
  if(m_vicon.GetFrame().Result != Result::Success) return boost::none;

  std::map<std::string,Eigen::Vector3f> result;

  const Output_GetMarkerCount markerCountOutput = m_vicon.GetMarkerCount(subjectName);
  if(markerCountOutput.Result != Result::Success) return boost::none;
  unsigned int markerCount = markerCountOutput.MarkerCount;

  for(unsigned int i = 0; i < markerCount; ++i)
  {
    // Get the name of the marker and its position in the Vicon coordinate system.
    std::string markerName = m_vicon.GetMarkerName(subjectName, i).MarkerName;
    Output_GetMarkerGlobalTranslation trans = m_vicon.GetMarkerGlobalTranslation(subjectName, markerName);

    // If we can't currently get the position of the marker, skip it.
    if(trans.Occluded) continue;

    // Transform the marker position from the Vicon coordinate system to our one (the Vicon coordinate system is in mm, whereas ours is in metres).
    Eigen::Vector3f pos(
      static_cast<float>(trans.Translation[0] / 1000),
      static_cast<float>(trans.Translation[1] / 1000),
      static_cast<float>(trans.Translation[2] / 1000)
    );

    // Record the position in the map.
    result.insert(std::make_pair(markerName, pos));
  }

  return result;
}

}
