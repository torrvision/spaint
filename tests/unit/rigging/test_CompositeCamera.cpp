#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <rigging/CompositeCamera.h>
#include <rigging/DerivedCamera.h>
#include <rigging/SimpleCamera.h>
using namespace rigging;

#define CHECK_EQUIVALENT_VECTORS(l,r) BOOST_CHECK_SMALL((l - r).norm(), 1e-5f)

BOOST_AUTO_TEST_SUITE(test_CompositeCamera)

BOOST_AUTO_TEST_CASE(stereo_test)
{
  // Make the camera rig itself.
  CompositeCamera_Ptr rig(new CompositeCamera(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f)));

  // Add left eye and right eye cameras to the rig. These are respectively 1 unit to the left and right of the main camera
  // and point inwards at an angle of 45 degrees.
  Camera_CPtr leftCamera(new DerivedCamera(
    rig,
    Eigen::AngleAxisf(static_cast<float>(-M_PI/4.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f)).toRotationMatrix(),  // the rotation is in *camera space*, i.e. about v
    Eigen::Vector3f(1.0f, 0.0f, 0.0f)                                                     // the translation is in *camera space*, i.e. along u
  ));
  rig->add_secondary_camera("left", leftCamera);

  Camera_CPtr rightCamera(new DerivedCamera(
    rig,
    Eigen::AngleAxisf(static_cast<float>(M_PI/4.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f)).toRotationMatrix(),   // the rotation is in *camera space*, i.e. about v
    Eigen::Vector3f(-1.0f, 0.0f, 0.0f)                                                    // the translation is in *camera space*, i.e. along -u
  ));
  rig->add_secondary_camera("right", rightCamera);

  // Rotate the entire rig so that it faces downwards.
  rig->rotate(rig->u(), static_cast<float>(M_PI/2.0f));

  // Check the positions and orientations of the left and right eye cameras.
  const float ONE_OVER_ROOT_2 = 1.0f / sqrtf(2.0f);
  CHECK_EQUIVALENT_VECTORS(rig->get_secondary_camera("left")->p(), Eigen::Vector3f(-1.0f, 0.0f, 0.0f));
  CHECK_EQUIVALENT_VECTORS(rig->get_secondary_camera("left")->u(), Eigen::Vector3f(-ONE_OVER_ROOT_2, 0.0f, -ONE_OVER_ROOT_2));
  CHECK_EQUIVALENT_VECTORS(rig->get_secondary_camera("left")->v(), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
  CHECK_EQUIVALENT_VECTORS(rig->get_secondary_camera("left")->n(), Eigen::Vector3f(ONE_OVER_ROOT_2, 0.0f, -ONE_OVER_ROOT_2));
  CHECK_EQUIVALENT_VECTORS(rig->get_secondary_camera("right")->p(), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
  CHECK_EQUIVALENT_VECTORS(rig->get_secondary_camera("right")->u(), Eigen::Vector3f(-ONE_OVER_ROOT_2, 0.0f, ONE_OVER_ROOT_2));
  CHECK_EQUIVALENT_VECTORS(rig->get_secondary_camera("right")->v(), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
  CHECK_EQUIVALENT_VECTORS(rig->get_secondary_camera("right")->n(), Eigen::Vector3f(-ONE_OVER_ROOT_2, 0.0f, -ONE_OVER_ROOT_2));
}

BOOST_AUTO_TEST_SUITE_END()
