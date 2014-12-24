#include <iostream>

#include <spaint/cameras/CompositeCamera.h>
#include <spaint/cameras/DerivedCamera.h>
#include <spaint/cameras/SimpleCamera.h>
using namespace spaint;

int main()
{
  MoveableCamera_Ptr primaryCamera(new SimpleCamera(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f)));
  CompositeCamera compositeCamera(primaryCamera);
  Camera_CPtr leftCamera(new DerivedCamera(
    primaryCamera,
    Eigen::AngleAxisf(-M_PI/4.0f, primaryCamera->v()).toRotationMatrix(),
    Eigen::Vector3f(1.0f, 0.0f, 0.0f)
  ));
  Camera_CPtr rightCamera(new DerivedCamera(
    primaryCamera,
    Eigen::AngleAxisf(M_PI/4.0f, primaryCamera->v()).toRotationMatrix(),
    Eigen::Vector3f(-1.0f, 0.0f, 0.0f)
  ));
  compositeCamera.add_secondary_camera("left", leftCamera);
  compositeCamera.add_secondary_camera("right", rightCamera);
  compositeCamera.rotate(primaryCamera->v(), M_PI/2.0f);
  std::cout << "LP:\n" << compositeCamera.get_secondary_camera("left")->p() << '\n';
  std::cout << "LU:\n" << compositeCamera.get_secondary_camera("left")->u() << '\n';
  std::cout << "LV:\n" << compositeCamera.get_secondary_camera("left")->v() << '\n';
  std::cout << "LN:\n" << compositeCamera.get_secondary_camera("left")->n() << '\n';
  std::cout << "RP:\n" << compositeCamera.get_secondary_camera("right")->p() << '\n';
  std::cout << "RU:\n" << compositeCamera.get_secondary_camera("right")->u() << '\n';
  std::cout << "RV:\n" << compositeCamera.get_secondary_camera("right")->v() << '\n';
  std::cout << "RN:\n" << compositeCamera.get_secondary_camera("right")->n() << '\n';
  return 0;
}
