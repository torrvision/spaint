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
   * \param oldVoxelLabelsMB  An optional memory block into which to store the old semantic labels of the voxels being marked.
   */
  virtual void mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, unsigned char label,
                           ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                           ORUtils::MemoryBlock<unsigned char> *oldVoxelLabelsMB = NULL) const = 0;

  /**
   * \brief Marks a set of voxels in the scene with the specified semantic labels.
   *
   * \param voxelLocationsMB  A memory block containing the locations of the voxels in the scene.
   * \param voxelLabelsMB     A memory block containing the semantic labels with which to mark the voxels (one per voxel).
   * \param scene             The scene.
   * \param oldVoxelLabelsMB  An optional memory block into which to store the old semantic labels of the voxels being marked.
   */
  virtual void mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                           const ORUtils::MemoryBlock<unsigned char>& voxelLabelsMB,
                           ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                           ORUtils::MemoryBlock<unsigned char> *oldVoxelLabelsMB = NULL) const = 0;
};

}

#endif
