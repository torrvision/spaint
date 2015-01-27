/**
 * spaint: SemanticVisualiser_CPU.cpp
 */

#include "visualisers/multiplatform/cpu/SemanticVisualiser_CPU.h"

#include "visualisers/multiplatform/shared/SemanticVisualiser_Shared.h"

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SemanticVisualiser_CPU::render(const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene, const ITMLib::Objects::ITMPose *pose,
                                    const ITMLib::Objects::ITMIntrinsics *intrinsics, const ITMLib::Objects::ITMRenderState *renderState,
                                    ITMUChar4Image *outputImage) const
{
  // TODO: Get rid of this bit (after checking to make sure that's ok).
  const SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const ITMVoxelIndex::IndexData *voxelIndex = scene->index.getIndexData();

  Vector2i imgSize = outputImage->noDims;
  float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;

  Matrix4f invM = pose->GetInvM();
  Vector4f projParams = intrinsics->projectionParamsSimple.all;
  projParams.x = 1.0f / projParams.x;
  projParams.y = 1.0f / projParams.y;

  float mu = scene->sceneParams->mu;
  Vector3f lightSource = -Vector3f(invM.getColumn(2));

  Vector4u *outRendering = outputImage->GetData(MEMORYDEVICE_CPU);
  const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
  Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);

  for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
  {
    int locId = x + y * imgSize.x;
    int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

    castRay<SpaintVoxel,ITMVoxelIndex>(pointsRay[locId], x, y, voxelData, voxelIndex, invM, projParams, oneOverVoxelSize, mu, minmaximg[locId2]);
  }
  // END TODO

  Vector3u labelColours[] =
  {
    Vector3u(255, 255, 255),
    Vector3u(255, 0, 0),
    Vector3u(0, 255, 0),
    Vector3u(0, 0, 255)
  };

  for (int locId = 0; locId < imgSize.x * imgSize.y; ++locId)
  {
    Vector4f ptRay = pointsRay[locId];
    processPixelSemantic(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource, labelColours);
  }
}

}
