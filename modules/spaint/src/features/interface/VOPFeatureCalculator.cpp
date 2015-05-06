/**
 * spaint: VOPFeatureCalculator.cpp
 */

#include "features/interface/VOPFeatureCalculator.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

VOPFeatureCalculator::VOPFeatureCalculator(size_t maxLabelCount, size_t maxVoxelsPerLabel, size_t patchSize, float patchSpacing)
: m_initialPatchesMB(maxLabelCount * maxVoxelsPerLabel * patchSize * patchSize, true, true),
  m_maxLabelCount(maxLabelCount),
  m_maxVoxelsPerLabel(maxVoxelsPerLabel),
  m_patchSize(patchSize),
  m_patchSpacing(patchSpacing),
  m_surfaceNormalsMB(maxLabelCount * maxVoxelsPerLabel, true, true),
  m_xAxesMB(maxLabelCount * maxVoxelsPerLabel, true, true),
  m_yAxesMB(maxLabelCount * maxVoxelsPerLabel, true, true)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VOPFeatureCalculator::calculate_features(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                                              const ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB,
                                              const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                                              ORUtils::MemoryBlock<float>& featuresMB) const
{
  // Calculate the surface normals at the voxel locations.
  const SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const ITMVoxelIndex::IndexData *indexData = scene->index.getIndexData();
  calculate_surface_normals(voxelLocationsMB, voxelCountsForLabelsMB, voxelData, indexData);

  // Construct a coordinate system in the tangent plane to the surface at each voxel location.
  generate_coordinate_systems(voxelCountsForLabelsMB);

  // Read an RGB patch around each voxel location.
  // TODO

  // Convert the RGB patches to the CIELab colour space.
  // TODO

  // Compute a histogram of intensity gradients for each patch.
  // TODO

  // Determine the dominant orientation for each patch.
  // TODO

  // Read a new RGB patch around each voxel location that is oriented based on the dominant orientation.
  // TODO

  // Convert the new RGB patches to the CIELab colour space to form the feature vectors.
  // TODO

  // For each feature vector, fill in the signed distance to the dominant horizontal surface present in the scene as an extra feature.
  // TODO
}

size_t VOPFeatureCalculator::get_feature_count() const
{
  // TODO
  return 1;
}

}
