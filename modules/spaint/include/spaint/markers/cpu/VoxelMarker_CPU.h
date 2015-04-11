/**
 * spaint: VoxelMarker_CPU.h
 */

#ifndef H_SPAINT_VOXELMARKER_CPU
#define H_SPAINT_VOXELMARKER_CPU

#include "../interface/VoxelMarker.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to mark a set of voxels with a semantic label using the CPU.
 */
class VoxelMarker_CPU : public VoxelMarker
{
  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, unsigned char label,
                           ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                           ORUtils::MemoryBlock<unsigned char> *oldVoxelLabelsMB) const;

  /** Override */
  virtual void mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                           const ORUtils::MemoryBlock<unsigned char>& voxelLabelsMB,
                           ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                           ORUtils::MemoryBlock<unsigned char> *oldVoxelLabelsMB) const;
};

}

#endif
