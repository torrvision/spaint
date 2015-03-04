/**
 * spaint: DepthCalculator_CPU.cpp
 */

#include "visualisers/cpu/DepthCalculator_CPU.h"

#include "visualisers/shared/DepthCalculator_Shared.h"

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS #################### 

void DepthCalculator_CPU::render_euclidean_distance(ITMFloatImage *outputImage, const ITMLib::Objects::ITMRenderState *renderState, const rigging::SimpleCamera *camera, float voxelSize) const
{
  // Shade all of the pixels in the image.
  int imgSize = outputImage->noDims.x * outputImage->noDims.y;
  float *outRendering = outputImage->GetData(MEMORYDEVICE_CPU);
  const Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
  const Eigen::Vector3f& cameraPositionEigen = camera->p();
  Vector3f cameraPosition;
  cameraPosition.x = cameraPositionEigen[0];
  cameraPosition.y = cameraPositionEigen[1];
  cameraPosition.z = cameraPositionEigen[2];

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < imgSize; ++locId)
  {
    Vector4f ptRay = pointsRay[locId];
    shade_pixel_euclidean_distance(outRendering[locId], cameraPosition, ptRay.toVector3(), voxelSize, ptRay.w > 0);
  }
}

void DepthCalculator_CPU::render_orthographic_distance(ITMFloatImage *outputImage, const ITMLib::Objects::ITMRenderState *renderState, const rigging::SimpleCamera *camera, float voxelSize) const
{
  // Shade all of the pixels in the image.
  int imgSize = outputImage->noDims.x * outputImage->noDims.y;
  float *outRendering = outputImage->GetData(MEMORYDEVICE_CPU);
  const Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
  const Eigen::Vector3f& cameraPositionEigen = camera->p();
  Vector3f cameraPosition;
  cameraPosition.x = cameraPositionEigen[0];
  cameraPosition.y = cameraPositionEigen[1];
  cameraPosition.z = cameraPositionEigen[2];
  
  const Eigen::Vector3f& cameraLookVectorEigen = camera->n();
  Vector3f cameraLookVector;
  cameraLookVector.x = cameraLookVectorEigen[0];
  cameraLookVector.y = cameraLookVectorEigen[1];
  cameraLookVector.z = cameraLookVectorEigen[2];

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < imgSize; ++locId)
  {
    Vector4f ptRay = pointsRay[locId];
    shade_pixel_orthographic_distance(outRendering[locId], cameraPosition, cameraLookVector, ptRay.toVector3(), voxelSize, ptRay.w > 0);
  }
}

}
