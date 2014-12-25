#include <iostream>

#include <rigging/CompositeCamera.h>
#include <rigging/DerivedCamera.h>
#include <rigging/SimpleCamera.h>
using namespace rigging;

int main()
{
  MoveableCamera_Ptr primaryCamera(new SimpleCamera(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f)));
  CompositeCamera compositeCamera(primaryCamera);
  Camera_CPtr leftCamera(new DerivedCamera(
    primaryCamera,
    Eigen::AngleAxisf(-M_PI/2.0f, Eigen::Vector3f(0.0f, 1.0f, 0.0f)).toRotationMatrix(),  // the rotation is in *camera space*, i.e. about v
    Eigen::Vector3f(1.0f, 0.0f, 0.0f)                                                     // the translation is in *camera space*, i.e. along u
  ));
  Camera_CPtr rightCamera(new DerivedCamera(
    primaryCamera,
    Eigen::AngleAxisf(M_PI/2.0f, Eigen::Vector3f(0.0f, 1.0f, 0.0f)).toRotationMatrix(),   // the rotation is in *camera space*, i.e. about v
    Eigen::Vector3f(-1.0f, 0.0f, 0.0f)                                                    // the translation is in *camera space*, i.e. along -u
  ));
  compositeCamera.add_secondary_camera("left", leftCamera);
  compositeCamera.add_secondary_camera("right", rightCamera);
  compositeCamera.rotate(primaryCamera->u(), M_PI/2.0f);
  std::cout << "CN:\n" << compositeCamera.n() << '\n';
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
