/**
 * spaint: VOPFeatureCalculator_CPU.h
 */

#ifndef H_SPAINT_VOPFEATURECALCULATOR_CPU
#define H_SPAINT_VOPFEATURECALCULATOR_CPU

#include "../interface/VOPFeatureCalculator.h"

namespace spaint {
/**
 * \brief An instance of a class deriving from this one can be used to calculate VOP feature descriptors for voxels sampled from a scene using the CPU.
 */
class VOPFeatureCalculator_CPU : public VOPFeatureCalculator
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a CPU-based VOP feature calculator.
   *
   * \param maxLabelCount     The maximum number of labels that can be in use.
   * \param maxVoxelsPerLabel The maximum number of voxels that will be sampled for each label.
   * \param patchSize         The side length of a VOP patch (must be odd).
   * \param patchSpacing      The spacing in the scene between individual pixels in a patch.
   */
  VOPFeatureCalculator_CPU(size_t maxLabelCount, size_t maxVoxelsPerLabel, size_t patchSize, float patchSpacing);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void calculate_surface_normals(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, const ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB,
                                         const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData) const;

  /** Override */
  virtual void convert_patches_to_lab(ORUtils::MemoryBlock<float>& featuresMB) const;

  /** Override */
  virtual void generate_coordinate_systems(const ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const;

  /** Override */
  virtual void generate_rgb_patches(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                                    const ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB,
                                    const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                    ORUtils::MemoryBlock<float>& featuresMB) const;
};

}

#endif
