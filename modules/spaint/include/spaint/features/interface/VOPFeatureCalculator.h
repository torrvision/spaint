/**
 * spaint: VOPFeatureCalculator.h
 */

#ifndef H_SPAINT_VOPFEATURECALCULATOR
#define H_SPAINT_VOPFEATURECALCULATOR

#include <ITMLib/Objects/ITMScene.h>

#include "../../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to calculate VOP feature descriptors for voxels sampled from a scene.
 */
class VOPFeatureCalculator
{
  //#################### PROTECTED VARIABLES ####################
protected:
  /** A memory block into which to store the initial patches used during feature calculation (packed sequentially). */
  //mutable ORUtils::MemoryBlock<float> m_initialPatchesMB;

  /** The maximum number of labels that can be allocated by the label manager. */
  int m_maxLabelCount;

  /** The maximum number of voxels that will be sampled for each label. */
  int m_maxVoxelsPerLabel;

  /** The surface normals at the voxel locations. */
  mutable ORUtils::MemoryBlock<Vector3f> m_surfaceNormalsMB;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a VOP feature calculator.
   *
   * \param maxLabelCount     The maximum number of labels that can be allocated by the label manager.
   * \param maxVoxelsPerLabel The maximum number of voxels that will be sampled for each label.
   */
  VOPFeatureCalculator(int maxLabelCount, int maxVoxelsPerLabel);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the feature calculator.
   */
  virtual ~VOPFeatureCalculator();

  //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Calculates the surface normals at the voxel locations.
   *
   * \param voxelLocationsMB        A memory block containing the locations of the voxels for which to calculate the surface normals (grouped by label).
   * \param voxelCountsForLabelsMB  A memory block containing the numbers of voxels for each label.
   * \param voxelData               The scene's voxel data.
   * \param indexData               The scene's index data.
   */
  virtual void calculate_surface_normals(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, const ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB,
                                         const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Calculates VOP feature descriptors for the specified voxels (grouped by label).
   *
   * \param voxelLocationsMB        A memory block containing the locations of the voxels for which to calculate feature descriptors (grouped by label).
   * \param voxelCountsForLabelsMB  A memory block containing the numbers of voxels for each label.
   * \param patchSize               The side length of a VOP patch (must be odd).
   * \param patchSpacing            The spacing in the scene between individual pixels in a patch.
   * \param scene                   The scene.
   * \param featuresMB              A memory block into which to store the calculated feature descriptors (packed sequentially).
   */
  void calculate_features(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                          const ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB,
                          int patchSize, float patchSpacing,
                          const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                          ORUtils::MemoryBlock<float>& featuresMB) const;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Computes the size of feature descriptor needed for a given patch size.
   *
   * \param patchSize The side length of a VOP patch (must be odd).
   * \return          The size of feature descriptor needed.
   */
  static int compute_feature_count(int patchSize);
};

}

#endif
