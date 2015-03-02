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

}
