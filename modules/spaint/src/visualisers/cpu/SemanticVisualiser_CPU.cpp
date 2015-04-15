/**
 * spaint: SemanticVisualiser_CPU.cpp
 */

#include "visualisers/cpu/SemanticVisualiser_CPU.h"

#include "visualisers/shared/SemanticVisualiser_Shared.h"

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SemanticVisualiser_CPU::render(const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene, const ITMLib::Objects::ITMPose *pose,
                                    const ITMLib::Objects::ITMIntrinsics *intrinsics, const ITMLib::Objects::ITMRenderState *renderState,
                                    const LabelManager *labelManager, bool usePhong, ITMUChar4Image *outputImage) const
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
  const Vector3u *labelColours = &labelManager->get_label_colours()[0];

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for (int locId = 0; locId < imgSize; ++locId)
  {
    Vector4f ptRay = pointsRay[locId];
    shade_pixel_semantic(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, labelColours, viewerPos, lightPos, usePhong);
  }
}

}
