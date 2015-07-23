/**
 * spaint: VOPFeatureCalculator_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
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
   * \param maxVoxelLocationCount The maximum number of voxel locations for which we will be calculating features at any one time.
   * \param patchSize             The side length of a VOP patch (must be odd).
   * \param patchSpacing          The spacing in the scene (in voxels) between individual pixels in a patch.
   * \param binCount              The number of bins into which to quantize orientations when aligning voxel patches.
   */
  VOPFeatureCalculator_CUDA(size_t maxVoxelLocationCount, size_t patchSize, float patchSpacing, size_t binCount);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void calculate_surface_normals(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                         ORUtils::MemoryBlock<float>& featuresMB) const;

  /** Override */
  virtual void convert_patches_to_lab(int voxelLocationCount, ORUtils::MemoryBlock<float>& featuresMB) const;

  /** Override */
  virtual void fill_in_heights(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, ORUtils::MemoryBlock<float>& featuresMB) const;

  /** Override */
  virtual void generate_coordinate_systems(int voxelLocationCount) const;

  /** Override */
  virtual void generate_rgb_patches(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                                    const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                    ORUtils::MemoryBlock<float>& featuresMB) const;

  /** Override */
  virtual void update_coordinate_systems(int voxelLocationCount, const ORUtils::MemoryBlock<float>& featuresMB) const;
};

}

#endif
