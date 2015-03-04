#include <iostream>

#include <Leap.h>

class Listener : public Leap::Listener
{
public:
  /** Override */
  virtual void onConnect(const Leap::Controller& controller)
  {
    std::cout << "Connected to the Leap!\n";
  }

  /** Override */
  virtual void onFrame(const Leap::Controller& controller)
  {
    const Leap::Frame frame = controller.frame();
    static int64_t lastFrameID = frame.id();
    if(frame.id() - lastFrameID < 10) return;
    lastFrameID = frame.id();
    std::cout << "Frame " << frame.id() << '\n'
              << "..Timestamp: " << frame.timestamp() << '\n'
              << "..Hands: " << frame.hands().count() << '\n'
              << "..Fingers: " << frame.fingers().count() << '\n'
              << "..Tools: " << frame.tools().count() << '\n'
              << "..Gestures: " << frame.gestures().count() << '\n';

    if(frame.hands().count() > 0)
    {
      std::cout << "..Tip Position: " << frame.hands()[0].fingers()[1].tipPosition() << '\n';
      std::cout << "..Point Direction: " << frame.hands()[0].fingers()[1].direction() << '\n';
    }
  }
};

int main()
{
  Leap::Controller controller;
  Listener listener;
  controller.addListener(listener);

  std::cout << "Press Enter to quit..." << std::endl;
  std::cin.get();

  controller.removeListener(listener);
  return 0;
}
