/**
 * spaint: LabelPropagator.cpp
 */

#include "propagation/interface/LabelPropagator.h"

#include "util/MemoryBlockFactory.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

LabelPropagator::LabelPropagator(size_t raycastResultSize)
: m_normals(MemoryBlockFactory::instance().make_block<Vector3f>(raycastResultSize))
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void LabelPropagator::propagate_label(SpaintVoxel::Label label, const ITMFloat4Image *raycastResult,
                                      ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const
{
  // Calculate the normals of the voxels in the raycast result.
  calculate_normals(raycastResult, scene);

  // Propagate the specified label across the scene, stopping at position or normal discontinuities.
  perform_propagation();
}

}
