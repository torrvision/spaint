/**
 * spaint: Picker_CUDA.cu
 */

#include "picking/cuda/Picker_CUDA.h"

#include "picking/shared/Picker_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_get_pick_point(int x, int y, int width, const Vector4f *imageData, Vector3f& pickPoint, bool& result)
{
  result = get_pick_point(x, y, width, imageData, pickPoint);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool Picker_CUDA::pick(int x, int y, const ITMLib::Objects::ITMRenderState *renderState, ORUtils::MemoryBlock<Vector3f>& pickPointMB) const
{
  static ORUtils::MemoryBlock<bool> result(1, true, true);
  ck_get_pick_point<<<1,1>>>(
    x, y,
    renderState->raycastResult->noDims.x,
    renderState->raycastResult->GetData(MEMORYDEVICE_CUDA),
    *pickPointMB.GetData(MEMORYDEVICE_CUDA),
    *result.GetData(MEMORYDEVICE_CUDA)
  );
  result.UpdateHostFromDevice();
  return result.GetData(MEMORYDEVICE_CPU)[0];
}

}
