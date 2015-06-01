/**
 * spaint: Picker_CUDA.cu
 */

#include <stdexcept>

#include "picking/cuda/Picker_CUDA.h"

#include "picking/shared/Picker_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_get_pick_point(int x, int y, int width, const Vector4f *imageData, Vector3f *pickPoint, bool *result)
{
  *result = get_pick_point(x, y, width, imageData, *pickPoint);
}

__global__ void ck_to_short(const Vector3f *pickPointsFloat, Vector3s *pickPointsShort, int pointCount)
{
  int tid = threadIdx.x + blockDim.x * blockIdx.x;
  if(tid < pointCount) pickPointsShort[tid] = pickPointsFloat[tid].toShortRound();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool Picker_CUDA::pick(int x, int y, const ITMLib::Objects::ITMRenderState *renderState, ORUtils::MemoryBlock<Vector3f>& pickPointMB) const
{
  static ORUtils::MemoryBlock<bool> result(1, true, true);
  ck_get_pick_point<<<1,1>>>(
    x, y,
    renderState->raycastResult->noDims.x,
    renderState->raycastResult->GetData(MEMORYDEVICE_CUDA),
    pickPointMB.GetData(MEMORYDEVICE_CUDA),
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
