/**
 * spaint: Picker_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include <stdexcept>

#include "picking/cuda/Picker_CUDA.h"

#include "picking/shared/Picker_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_get_pick_point(float fracX, float fracY, int width, int height, const Vector4f *imageData, Vector3f *pickPoint, bool *result)
{
  *result = get_pick_point(fracX, fracY, width, height, imageData, *pickPoint);
}

__global__ void ck_to_short(const Vector3f *pickPointsFloat, Vector3s *pickPointsShort, int pointCount)
{
  int tid = threadIdx.x + blockDim.x * blockIdx.x;
  if(tid < pointCount) pickPointsShort[tid] = pickPointsFloat[tid].toShortRound();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool Picker_CUDA::pick(float fracX, float fracY, const ITMLib::ITMRenderState *renderState, ORUtils::MemoryBlock<Vector3f>& pickPointsMB, size_t offset) const
{
  if(offset >= pickPointsMB.dataSize)
  {
    throw std::runtime_error("Error: The offset at which to write the pick point is out of range");
  }

  static ORUtils::MemoryBlock<bool> result(1, true, true);
  ck_get_pick_point<<<1,1>>>(
    fracX, fracY,
    renderState->raycastResult->noDims.x,
    renderState->raycastResult->noDims.y,
    renderState->raycastResult->GetData(MEMORYDEVICE_CUDA),
    pickPointsMB.GetData(MEMORYDEVICE_CUDA) + offset,
    result.GetData(MEMORYDEVICE_CUDA)
  );
  result.UpdateHostFromDevice();
  return *result.GetData(MEMORYDEVICE_CPU);
}

void Picker_CUDA::to_short(const ORUtils::MemoryBlock<Vector3f>& pickPointsFloatMB, ORUtils::MemoryBlock<Vector3s>& pickPointsShortMB) const
{
  if(pickPointsFloatMB.dataSize != pickPointsShortMB.dataSize)
  {
    throw std::runtime_error("Error: The memory block into which to write the converted pick points must be of the right size");
  }

  const Vector3f *pickPointsFloat = pickPointsFloatMB.GetData(MEMORYDEVICE_CUDA);
  Vector3s *pickPointsShort = pickPointsShortMB.GetData(MEMORYDEVICE_CUDA);
  int pointCount = static_cast<int>(pickPointsFloatMB.dataSize);

  int threadsPerBlock = 256;
  int numBlocks = (pointCount + threadsPerBlock - 1) / threadsPerBlock;
  ck_to_short<<<numBlocks,threadsPerBlock>>>(pickPointsFloat, pickPointsShort, pointCount);
}

}
