/**
 * spaint: ViconTracker.cpp
 */

#include "trackers/ViconTracker.h"
using namespace ViconDataStreamSDK::CPP;

#include <fstream>
#include <iomanip>
#include <stdexcept>

#include <rigging/SimpleCamera.h>

#include "util/CameraPoseConverter.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

ViconTracker::ViconTracker(const std::string& host, const std::string& subjectName)
: m_subjectName(subjectName)
{
  // TODO: Validate the IP address.

  if(m_vicon.Connect(host + ":801").Result != Result::Success || !m_vicon.IsConnected().Connected)
  {
    throw std::runtime_error("Could not connect to the Vicon system");
  }

  m_vicon.EnableMarkerData();
  m_vicon.EnableSegmentData();
  m_vicon.EnableUnlabeledMarkerData();
  m_vicon.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);
  //m_vicon.SetAxisMapping(Direction::Up, Direction::Right, Direction::Forward);
  //m_vicon.SetAxisMapping(Direction::Up, Direction::Forward, Direction::Right);
  //m_vicon.SetAxisMapping(Direction::Right, Direction::Up, Direction::Forward);
  //m_vicon.SetAxisMapping(Direction::Right, Direction::Forward, Direction::Up);
  //m_vicon.SetAxisMapping(Direction::Forward, Direction::Right, Direction::Up);
  //m_vicon.SetAxisMapping(Direction::Forward, Direction::Up, Direction::Right);
  //m_vicon.SetAxisMapping(Direction::Right, Direction::Up, Direction::Forward);
  //m_vicon.SetAxisMapping(Direction::Right, Direction::Down, Direction::Forward);
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

Matrix4f to_matrix(const Output_GetSegmentGlobalRotationMatrix& r, const Output_GetSegmentGlobalTranslation& t)
{
  Matrix4f m;
  int k = 0;
  for(int y = 0; y < 3; ++y)
  {
    for(int x = 0; x < 3; ++x)
    {
      m(x,y) = r.Rotation[k++];
    }
    m(3,y) = t.Translation[y];
  }
  m(0,3) = m(1,3) = m(2,3) = 0;
  m(3,3) = 1;
  return m;
}

void ViconTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
  if(m_vicon.GetFrame().Result != Result::Success) return;

  int subjectIndex = find_subject_index(m_subjectName);
  if(subjectIndex == -1) return;

  //static std::ofstream fs("/Users/stuart/programs/spaint/foo.txt");
  std::ostream& fs = std::cout;

#if 1
  fs << "\n#####\n";
  fs << "Frame " << m_vicon.GetFrameNumber().FrameNumber << '\n';
#endif

  int segmentCount = m_vicon.GetSegmentCount(m_subjectName).SegmentCount;
  if(segmentCount == 0) return;

  const int SEGMENT_INDEX = 0;
  std::string segmentName = m_vicon.GetSegmentName(m_subjectName, SEGMENT_INDEX).SegmentName;
  Output_GetSegmentGlobalTranslation tr = m_vicon.GetSegmentGlobalTranslation(m_subjectName, segmentName);
#if 0
  fs << "Translation: " << tr.Translation[0] << ' ' << tr.Translation[1] << ' ' << tr.Translation[2] << '\n';
#endif

  Output_GetSegmentGlobalRotationMatrix rr = m_vicon.GetSegmentGlobalRotationMatrix(m_subjectName, segmentName);
#if 0
  std::cout << "Rotation:\n";
  for(int i = 0; i < 9; ++i)
  {
    fs << std::fixed << std::setprecision(3) << rr.Rotation[i] << ' ';
    if(i % 3 == 2) std::cout << '\n';
  }
#endif

  static Matrix4f initialPose;
  static Matrix4f invInitialPose;
  static bool done = false;
  if(!done)
  {
    initialPose = to_matrix(rr, tr);
    initialPose.inv(invInitialPose);
    done = true;
  }
  Matrix4f globalPose = to_matrix(rr, tr);
  //Matrix4f M = invInitialPose * globalPose;
  Matrix4f M = globalPose * invInitialPose;
  for(int y = 0; y < 3; ++y)
  {
    for(int x = 0; x < 3; ++x)
    {
      trackingState->pose_d->R(x,y) = M(x,y);
    }
    trackingState->pose_d->T[y] = M(3,y);
  }

  trackingState->pose_d->SetParamsFromModelView();
  trackingState->pose_d->SetModelViewFromParams();

  fs << std::fixed << std::setprecision(1) << trackingState->pose_d->M << "\n\n";

  rigging::SimpleCamera cam = CameraPoseConverter::pose_to_camera(*trackingState->pose_d);
  fs << cam.n() << "\n\n";
  fs.flush();

#if 0
  static Matrix4f oldInvV;
  static bool done = false;
  if(!done)
  {
    oldInvV.setIdentity();
    done = true;
  }

  Matrix4f V;
  V.setIdentity();

  for(int i = 0; i < 3; ++i) V(i,3) = tr.Translation[i];

  int k = 0;
  for(int y = 0; y < 3; ++y)
    for(int x = 0; x < 3; ++x)
    {
      V(x,y) = rr.Rotation[k++];
    }

  Matrix4f invV;
  V.inv(invV);

  Matrix4f deltaInvV;
  oldInvV.inv(deltaInvV);
  deltaInvV = deltaInvV * invV;

  //invM = invM * 

  /*Matrix4f invM;
  invM.setIdentity();

  for(int i = 0; i < 3; ++i) invM(i,3) = tr.Translation[i];

  int k = 0;
  for(int y = 0; y < 3; ++y)
    for(int x = 0; x < 3; ++x)
    {
      invM(x,y) = rr.Rotation[k++];
    }

  invM.inv(trackingState->pose_d->M);
  trackingState->pose_d->SetRTInvM_FromM();
  trackingState->pose_d->SetParamsFromModelView();
  trackingState->pose_d->SetModelViewFromParams();*/
  trackingState->pose_d->params.each.tx = tr.Translation[0];
  trackingState->pose_d->params.each.ty = tr.Translation[1];
  trackingState->pose_d->params.each.tz = tr.Translation[2];
  trackingState->pose_d->params.each.rx = rr.Rotation[0];
  trackingState->pose_d->params.each.ry = -rr.Rotation[1];
  trackingState->pose_d->params.each.rz = rr.Rotation[2];
  trackingState->pose_d->SetModelViewFromParams();
#endif
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

int ViconTracker::find_subject_index(const std::string& name) const
{
  int subjectCount = m_vicon.GetSubjectCount().SubjectCount;
  for(int i = 0; i < subjectCount; ++i)
  {
    std::string subjectName = m_vicon.GetSubjectName(i).SubjectName;
    if(subjectName == name)
    {
      return i;
    }
  }
  return -1;
}

}
