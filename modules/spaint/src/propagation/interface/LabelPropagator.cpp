/**
 * spaint: LabelPropagator.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "propagation/interface/LabelPropagator.h"

#include <itmx/MemoryBlockFactory.h>
using itmx::MemoryBlockFactory;

namespace spaint {

//#################### CONSTRUCTORS ####################

LabelPropagator::LabelPropagator(size_t raycastResultSize, float maxAngleBetweenNormals, float maxSquaredDistanceBetweenColours, float maxSquaredDistanceBetweenVoxels)
: m_maxAngleBetweenNormals(maxAngleBetweenNormals),
  m_maxSquaredDistanceBetweenColours(maxSquaredDistanceBetweenColours),
  m_maxSquaredDistanceBetweenVoxels(maxSquaredDistanceBetweenVoxels),
  m_surfaceNormalsMB(MemoryBlockFactory::instance().make_block<Vector3f>(raycastResultSize))
{}

//#################### DESTRUCTOR ####################

LabelPropagator::~LabelPropagator() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void LabelPropagator::propagate_label(SpaintVoxel::Label label, const ITMFloat4Image *raycastResult, SpaintVoxelScene *scene) const
{
  // Calculate the normals of the voxels in the raycast result.
  calculate_normals(raycastResult, scene);

  // Propagate the specified label across the scene, stopping at position, normal or colour discontinuities.
  perform_propagation(label, raycastResult, scene);
}

}
