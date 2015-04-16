/**
 * spaint: VOPFeatureCalculator_CPU.h
 */

#ifndef H_SPAINT_VOPFEATURECALCULATOR_CPU
#define H_SPAINT_VOPFEATURECALCULATOR_CPU

#include "../interface/VOPFeatureCalculator.h"

namespace spaint {
/**
 * \brief An instance of a class deriving from this one can be used to calculate VOP feature descriptors for voxels sampled from a scene on the CPU.
 */
class VOPFeatureCalculator_CPU : public VOPFeatureCalculator
{
  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void calculate_surface_normals(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, const ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB,
                                         const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData) const;
};

}

#endif
