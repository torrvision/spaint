/**
 * spaint: Picker_CPU.cpp
 */

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

void Picker_CPU::to_short(const ORUtils::MemoryBlock<Vector3f>& pickPointFloatMB, ORUtils::MemoryBlock<Vector3s>& pickPointShortMB) const
{
  *pickPointShortMB.GetData(MEMORYDEVICE_CPU) = pickPointFloatMB.GetData(MEMORYDEVICE_CPU)->toShortRound();
}

}
