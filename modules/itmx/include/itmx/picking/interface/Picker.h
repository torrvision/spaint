/**
 * itmx: Picker.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_ITMX_PICKER
#define H_ITMX_PICKER

#include <vector>

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>

namespace itmx {

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
   * \param x             The x coordinate of the point on the image plane through which the ray is cast.
   * \param y             The y coordinate of the point on the image plane through which the ray is cast.
   * \param renderState   A render state corresponding to the camera pose.
   * \param pickPointsMB  A memory block into which to write the voxel coordinates of the nearest scene point (if any) that is hit by the ray.
   * \param offset        The offset into the memory block at which to write.
   * \return              true, if the ray hit the scene, or false otherwise.
   */
  virtual bool pick(int x, int y, const ITMLib::ITMRenderState *renderState, ORUtils::MemoryBlock<Vector3f>& pickPointsMB, size_t offset = 0) const = 0;

  /**
   * \brief Converts one or more pick points in Vector3f format into Vector3s format.
   *
   * \param pickPointsFloatMB A memory block containing the pick points in Vector3f format.
   * \param pickPointsShortMB A memory block into which to write the pick points in Vector3s format.
   */
  virtual void to_short(const ORUtils::MemoryBlock<Vector3f>& pickPointsFloatMB, ORUtils::MemoryBlock<Vector3s>& pickPointsShortMB) const = 0;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Converts one or more pick points in voxel coordinates into scene coordinates.
   *
   * \param pickPointsMB  The voxel coordinates of the picked points.
   * \param voxelSize     The size of an InfiniTAM voxel (in metres).
   */
  template <typename Vec>
  static std::vector<Vec> get_positions(const ORUtils::MemoryBlock<Vector3f>& pickPointsMB, float voxelSize)
  {
    // If the pick points are on the GPU, copy them across to the CPU.
    pickPointsMB.UpdateHostFromDevice();

    // Convert the pick points from voxel coordinates into scene coordinates and return them.
    const Vector3f *pickPoints = pickPointsMB.GetData(MEMORYDEVICE_CPU);
    size_t pickPointCount = pickPointsMB.dataSize;
    std::vector<Vec> positions(pickPointCount);

    for(size_t i = 0; i < pickPointCount; ++i)
    {
      const Vector3f& pickPoint = pickPoints[i];
      positions[i] = Vec(pickPoint.x * voxelSize, pickPoint.y * voxelSize, pickPoint.z * voxelSize);
    }

    return positions;
  }
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<const Picker> Picker_CPtr;

}

#endif
