/**
 * spaint: SemanticVisualiser_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "visualisers/cpu/SemanticVisualiser_CPU.h"

#include "visualisers/shared/SemanticVisualiser_Shared.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

SemanticVisualiser_CPU::SemanticVisualiser_CPU(size_t maxLabelCount)
: SemanticVisualiser(maxLabelCount)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void SemanticVisualiser_CPU::render_internal(const ITMLib::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene, const ORUtils::SE3Pose *pose,
                                             const ITMLib::ITMIntrinsics *intrinsics, const ITMLib::ITMRenderState *renderState,
                                             LightingType lightingType, float labelAlpha, ITMUChar4Image *outputImage) const
{
  // Calculate the light and viewer positions in voxel coordinates (the same coordinate space as the raycast results).
  const float voxelSize = scene->sceneParams->voxelSize;
  Vector3f lightPos = Vector3f(0.0f, -10.0f, -10.0f) / voxelSize;
  Vector3f viewerPos = Vector3f(pose->GetInvM().getColumn(3)) / voxelSize;

  // Shade all of the pixels in the image.
  int imgSize = outputImage->noDims.x * outputImage->noDims.y;
  Vector4u *outRendering = outputImage->GetData(MEMORYDEVICE_CPU);
  const Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
  const SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const ITMVoxelIndex::IndexData *voxelIndex = scene->index.getIndexData();
  const Vector3u *labelColours = m_labelColoursMB->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for (int locId = 0; locId < imgSize; ++locId)
  {
    Vector4f ptRay = pointsRay[locId];
    shade_pixel_semantic(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, labelColours, viewerPos, lightPos, lightingType, labelAlpha);
  }
}

}
