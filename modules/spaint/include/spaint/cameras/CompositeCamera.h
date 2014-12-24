/**
 * spaint: CompositeCamera.h
 */

#ifndef H_SPAINT_COMPOSITECAMERA
#define H_SPAINT_COMPOSITECAMERA

#include <map>
#include <string>

#include "DerivedCamera.h"
#include "MoveableCamera.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to represent a "composite" camera (i.e. a camera rig) consisting of several other cameras.
 *
 * A composite camera consists of a single primary camera that controls the position and orientation of the composite,
 * and a number of secondary cameras that are generally directly or indirectly based on that camera.
 */
class CompositeCamera : public MoveableCamera
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The primary camera, which controls the position and orientation of the composite. */
  MoveableCamera_Ptr m_primaryCamera;

  /** The secondary cameras. */
  std::map<std::string,DerivedCamera> m_secondaryCameras;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a composite camera.
   *
   * \param primaryCamera The primary camera, which controls the position and orientation of the composite.
   */
  explicit CompositeCamera(const MoveableCamera_Ptr& primaryCamera);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds a secondary camera to the composite.
   *
   * \param name    The name to give the secondary camera.
   * \param camera  The secondary camera.
   */
  void add_camera(const std::string& name, const DerivedCamera& camera);
};

}

#endif
