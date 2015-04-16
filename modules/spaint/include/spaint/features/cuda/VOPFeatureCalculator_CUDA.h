/**
 * spaint: VOPFeatureCalculator_CUDA.h
 */

#ifndef H_SPAINT_VOPFEATURECALCULATOR_CUDA
#define H_SPAINT_VOPFEATURECALCULATOR_CUDA

#include "../interface/VOPFeatureCalculator.h"

namespace spaint {
/**
 * \brief An instance of a class deriving from this one can be used to calculate VOP feature descriptors for voxels sampled from a scene using CUDA.
 */
class VOPFeatureCalculator_CUDA : public VOPFeatureCalculator
{
  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void calculate_surface_normals(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, const ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB,
                                         const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData) const;
};

}

#endif
