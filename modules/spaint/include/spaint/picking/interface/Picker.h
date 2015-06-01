/**
 * spaint: Picker.h
 */

#ifndef H_SPAINT_PICKER
#define H_SPAINT_PICKER

#include <ITMLib/Objects/ITMRenderState.h>

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to pick an individual point in the scene.
 */
class Picker
{
  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the picker.
   */
  virtual ~Picker() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Determines the nearest scene point (if any) that would be hit by a ray cast through (x,y) on the image plane
   *        when viewed from the camera pose with the specified render state.
   *
   * \param x           The x coordinate of the point on the image plane through which the ray is cast.
   * \param y           The y coordinate of the point on the image plane through which the ray is cast.
   * \param renderState A render state corresponding to the camera pose.
   * \param pickPointMB A memory block into which to write the voxel coordinates of the nearest scene point (if any) that is hit by the ray.
   * \return            true, if the ray hit the scene, or false otherwise.
   */
  virtual bool pick(int x, int y, const ITMLib::Objects::ITMRenderState *renderState, ORUtils::MemoryBlock<Vector3f>& pickPointMB) const = 0;

  /**
   * \brief Converts one or more pick points in Vector3f format into Vector3s format.
   *
   * \param pickPointsFloatMB A memory block containing the pick points in Vector3f format.
   * \param pickPointsShortMB A memory block into which to write the pick points in Vector3s format.
   */
  virtual void to_short(const ORUtils::MemoryBlock<Vector3f>& pickPointsFloatMB, ORUtils::MemoryBlock<Vector3s>& pickPointsShortMB) const = 0;
};

}

#endif
