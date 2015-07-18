/**
 * spaint: LabelPropagator_CPU.cpp
 */

#include "propagation/cpu/LabelPropagator_CPU.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

LabelPropagator_CPU::LabelPropagator_CPU(size_t raycastResultSize)
: LabelPropagator(raycastResultSize)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void LabelPropagator_CPU::calculate_normals(const ITMFloat4Image *raycastResult, const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const
{
  // TODO
}

}
