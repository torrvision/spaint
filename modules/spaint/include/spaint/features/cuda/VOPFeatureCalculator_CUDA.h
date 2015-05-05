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
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a CUDA-based VOP feature calculator.
   *
   * \param maxLabelCount     The maximum number of labels that can be in use.
   * \param maxVoxelsPerLabel The maximum number of voxels that will be sampled for each label.
   */
  VOPFeatureCalculator_CUDA(size_t maxLabelCount, size_t maxVoxelsPerLabel);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void calculate_surface_normals(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, const ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB,
                                         const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData) const;

  /** Override */
  virtual void generate_coordinate_systems(const ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const;
};

}

#endif
