/**
 * test-vicon: main.cpp
 */

#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>

#include <vicon/Client.h>
using namespace ViconDataStreamSDK::CPP;

class Tracker
{
  //#################### PRIVATE VARIABLES ####################
private:
  Client m_vicon;

  //#################### CONSTRUCTORS ####################
public:
  explicit Tracker(const std::string& hostname)
  {
    if(m_vicon.Connect(hostname).Result != Result::Success)
    {
      throw std::runtime_error("Error: Could not connect to the Vicon system!");
    }

    std::cout << "Connected to the Vicon system!" << std::endl;

    m_vicon.EnableMarkerData();
    m_vicon.EnableSegmentData();
    m_vicon.EnableUnlabeledMarkerData();

    m_vicon.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);

    m_vicon.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up);
  }

  //#################### DESTRUCTOR ####################
public:
  ~Tracker()
  {
    m_vicon.DisableUnlabeledMarkerData();
    m_vicon.DisableSegmentData();
    m_vicon.DisableMarkerData();
    m_vicon.Disconnect();
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  void next_frame()
  {
    if(m_vicon.GetFrame().Result != Result::Success) return;

    std::string subjectName = "kinect";
    int subjectIndex = find_subject_index(subjectName);
    if(subjectIndex == -1) return;

    std::cout << "\n#####\n";
    std::cout << "Frame " << m_vicon.GetFrameNumber().FrameNumber << '\n';

    int segmentCount = m_vicon.GetSegmentCount(subjectName).SegmentCount;
    for(int i = 0; i < segmentCount; ++i)
    {
      std::string segmentName = m_vicon.GetSegmentName(subjectName, i).SegmentName;

      Output_GetSegmentGlobalTranslation tr = m_vicon.GetSegmentGlobalTranslation(subjectName, segmentName);
      std::cout << "Translation: " << tr.Translation[0] << ' ' << tr.Translation[1] << ' ' << tr.Translation[2] << '\n';

      Output_GetSegmentGlobalRotationMatrix rr = m_vicon.GetSegmentGlobalRotationMatrix(subjectName, segmentName);
      std::cout << "Rotation:\n";
      for(int j = 0; j < 9; ++j)
      {
        std::cout << rr.Rotation[j] << ' ';
        if(j % 3 == 2) std::cout << '\n';
      }
    }
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  int find_subject_index(const std::string& name) const
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
};

int main()
try
{
  std::string hostname = "192.168.0.107:801";
  Tracker tracker(hostname);
  for(;;)
  {
    tracker.next_frame();
  }
  return 0;
}
catch(std::exception& e)
{
  std::cerr << e.what() << '\n';
  return EXIT_FAILURE;
}
