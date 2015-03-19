/**
 * spaint: Picker_CPU.cpp
 */

#include <stdexcept>

#include "picking/cpu/Picker_CPU.h"

#include "picking/shared/Picker_Shared.h"

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool Picker_CPU::pick(int x, int y, const ITMLib::Objects::ITMRenderState *renderState, ORUtils::MemoryBlock<Vector3f>& pickPointMB) const
{
  return get_pick_point(
    x, y,
    renderState->raycastResult->noDims.x,
    renderState->raycastResult->GetData(MEMORYDEVICE_CPU),
    *pickPointMB.GetData(MEMORYDEVICE_CPU)
  );
}

/*bool Picker_CPU::pick(const std::vector<int>& x, const std::vector<int>& y, const ITMLib::Objects::ITMRenderState *renderState, ORUtils::MemoryBlock<Vector3f>& pickPointMB) const
{
}
*/

void Picker_CPU::to_short(const ORUtils::MemoryBlock<Vector3f>& pickPointFloatMB, ORUtils::MemoryBlock<Vector3s>& pickPointShortMB) const
{
  if(pickPointFloatMB.dataSize != pickPointShortMB.dataSize)
    throw std::runtime_error("The two memory blocks must have the same size.");

  const Vector3f *floatData = pickPointFloatMB.GetData(MEMORYDEVICE_CPU);
  Vector3s *shortData = pickPointShortMB.GetData(MEMORYDEVICE_CPU);

  int size = pickPointFloatMB.dataSize;
  for(int i = 0; i < size; ++i)
  {
    shortData[i] = floatData[i].toShortRound();
  }
  //*pickPointShortMB.GetData(MEMORYDEVICE_CPU) = pickPointFloatMB.GetData(MEMORYDEVICE_CPU)->toShortRound();
}

}
