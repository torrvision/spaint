/**
 * spaint: VOPFeatureCalculator.cpp
 */

#include "features/interface/VOPFeatureCalculator.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

VOPFeatureCalculator::VOPFeatureCalculator(size_t maxLabelCount, size_t maxVoxelsPerLabel, size_t patchSize, float patchSpacing)
: m_maxLabelCount(maxLabelCount),
  m_maxVoxelsPerLabel(maxVoxelsPerLabel),
  m_patchSize(patchSize),
  m_patchSpacing(patchSpacing),
  m_surfaceNormalsMB(maxLabelCount * maxVoxelsPerLabel, true, true),
  m_xAxesMB(maxLabelCount * maxVoxelsPerLabel, true, true),
  m_yAxesMB(maxLabelCount * maxVoxelsPerLabel, true, true)
{}

//#################### DESTRUCTOR ####################

VOPFeatureCalculator::~VOPFeatureCalculator() {}

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

  // TODO
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

size_t VOPFeatureCalculator::get_feature_count()
{
  // TODO
  return 1;
}

}
