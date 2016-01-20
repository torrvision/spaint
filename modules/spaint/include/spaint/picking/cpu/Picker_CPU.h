/**
 * spaint: Picker_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_PICKER_CPU
#define H_SPAINT_PICKER_CPU

#include "../interface/Picker.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to pick an individual point in the scene using the CPU.
 */
class Picker_CPU : public Picker
{
  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual bool pick(float fracX, float fracY, const ITMLib::ITMRenderState *renderState, ORUtils::MemoryBlock<Vector3f>& pickPointsMB, size_t offset) const;

  /** Override */
  virtual void to_short(const ORUtils::MemoryBlock<Vector3f>& pickPointsFloatMB, ORUtils::MemoryBlock<Vector3s>& pickPointsShortMB) const;
};

}

#endif
