#include <iostream>

#include <rigging/CompositeCamera.h>
#include <rigging/DerivedCamera.h>
#include <rigging/SimpleCamera.h>
using namespace rigging;

int main()
{
  CompositeCamera_Ptr rig(new CompositeCamera(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f)));
  Camera_CPtr leftCamera(new DerivedCamera(
    rig,
    Eigen::AngleAxisf(-M_PI/2.0f, Eigen::Vector3f(0.0f, 1.0f, 0.0f)).toRotationMatrix(),  // the rotation is in *camera space*, i.e. about v
    Eigen::Vector3f(1.0f, 0.0f, 0.0f)                                                     // the translation is in *camera space*, i.e. along u
  ));
  Camera_CPtr rightCamera(new DerivedCamera(
    rig,
    Eigen::AngleAxisf(M_PI/2.0f, Eigen::Vector3f(0.0f, 1.0f, 0.0f)).toRotationMatrix(),   // the rotation is in *camera space*, i.e. about v
    Eigen::Vector3f(-1.0f, 0.0f, 0.0f)                                                    // the translation is in *camera space*, i.e. along -u
  ));
  rig->add_secondary_camera("left", leftCamera);
  rig->add_secondary_camera("right", rightCamera);
  rig->rotate(rig->u(), M_PI/2.0f);
  std::cout << "CN:\n" << rig->n() << '\n';
  std::cout << "LP:\n" << rig->get_secondary_camera("left")->p() << '\n';
  std::cout << "LU:\n" << rig->get_secondary_camera("left")->u() << '\n';
  std::cout << "LV:\n" << rig->get_secondary_camera("left")->v() << '\n';
  std::cout << "LN:\n" << rig->get_secondary_camera("left")->n() << '\n';
  std::cout << "RP:\n" << rig->get_secondary_camera("right")->p() << '\n';
  std::cout << "RU:\n" << rig->get_secondary_camera("right")->u() << '\n';
  std::cout << "RV:\n" << rig->get_secondary_camera("right")->v() << '\n';
  std::cout << "RN:\n" << rig->get_secondary_camera("right")->n() << '\n';
  return 0;
}
