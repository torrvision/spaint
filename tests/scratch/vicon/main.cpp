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
};

int main()
try
{
  std::string hostname = "192.168.0.107:801";
  Tracker tracker(hostname);
  return 0;
}
catch(std::exception& e)
{
  std::cerr << e.what() << '\n';
  return EXIT_FAILURE;
}
