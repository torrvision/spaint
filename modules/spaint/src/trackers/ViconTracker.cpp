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

  m_vicon.SetAxisMapping(Direction::Right, Direction::Down, Direction::Forward);
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

  static bool firstPass = true;
  static Matrix4f initialPose, invInitialPose;
  Matrix4f globalPose;
  if(true)//firstPass)
  {
    //firstPass = false;

    std::map<std::string,Eigen::Vector3f> markerPositions = get_marker_positions(m_subjectName);
    for(std::map<std::string,Eigen::Vector3f>::const_iterator it = markerPositions.begin(), iend = markerPositions.end(); it != iend; ++it)
    {
      fs << it->first << ' ' << it->second.transpose() << '\n';
    }

    const Eigen::Vector3f& c = markerPositions["centre"];
    const Eigen::Vector3f& l = markerPositions["left"];
    const Eigen::Vector3f& r = markerPositions["right"];
    Eigen::Vector3f v = (r - c).cross(l - c).normalized();
    fs << v.transpose() << '\n';

    const Eigen::Vector3f& f = markerPositions["front"];
    Eigen::Vector3f n = f - r;
    n = (n - ((n.dot(v)) * v)).normalized();
    fs << n.transpose() << '\n';

    rigging::SimpleCamera cam(c, n, v);
    globalPose = CameraPoseConverter::camera_to_pose(cam).GetM();
    if(firstPass)
    {
      initialPose = globalPose;
      initialPose.inv(invInitialPose);
      firstPass = false;
    }

    //fs << initialPose << '\n';
  }

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

  Matrix4f M = globalPose * invInitialPose;
  //fs << std::fixed << std::setprecision(1) << M << '\n';
#if 0
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
#endif
  trackingState->pose_d->SetM(M);

  //fs << std::fixed << std::setprecision(1) << trackingState->pose_d->M << "\n\n";

  //rigging::SimpleCamera cam = CameraPoseConverter::pose_to_camera(*trackingState->pose_d);
  //fs << cam.n() << "\n\n";
  fs.flush();
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

std::map<std::string,Eigen::Vector3f> ViconTracker::get_marker_positions(const std::string& subjectName) const
{
  std::map<std::string,Eigen::Vector3f> result;
  unsigned int markerCount = m_vicon.GetMarkerCount(subjectName).MarkerCount;
  for(unsigned int i = 0; i < markerCount; ++i)
  {
    std::string markerName = m_vicon.GetMarkerName(subjectName, i).MarkerName;
    Output_GetMarkerGlobalTranslation tr = m_vicon.GetMarkerGlobalTranslation(subjectName, markerName);
    //result.insert(std::make_pair(markerName, Eigen::Vector3d(tr.Translation).cast<float>()));
    Eigen::Vector3f p(tr.Translation[0] / 1000, tr.Translation[1] / 1000, tr.Translation[2] / 1000);
    result.insert(std::make_pair(markerName, p));
  }
  return result;
}

}
