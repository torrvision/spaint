/**
 * spaint: VoxelMarker.h
 */

#ifndef H_SPAINT_VOXELMARKER
#define H_SPAINT_VOXELMARKER

#include <ITMLib/Objects/ITMScene.h>

#include "../../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to mark a set of voxels with a semantic label.
 */
class VoxelMarker
{
  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the voxel marker.
   */
  virtual ~VoxelMarker() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Marks a set of voxels in the scene with the specified semantic label.
   *
   * \param voxelLocationsMB  A memory block containing the locations of the voxels in the scene.
   * \param label             The semantic label with which to mark the voxels.
   * \param scene             The scene.
   */
  virtual void mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, unsigned char label, ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const = 0;
};

}

#endif
