/**
 * test-vicon: main.cpp
 */

#include <cstddef>
#include <iostream>
#include <string>

#include <vicon/Client.h>
using namespace ViconDataStreamSDK::CPP;

int main()
{
  std::string hostname = "localhost:801";

  Client client;
  while(!client.IsConnected().Connected)
  {
    if(client.Connect(hostname).Result != Result::Success)
    {
      std::cout << "Error: Could not connect to the Vicon system!" << std::endl;
      return EXIT_FAILURE;
    }
  }
  return 0;
}
