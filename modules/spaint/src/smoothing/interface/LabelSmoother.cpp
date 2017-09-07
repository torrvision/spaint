/**
 * spaint: LabelSmoother.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "smoothing/interface/LabelSmoother.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

LabelSmoother::LabelSmoother(size_t maxLabelCount, float maxSquaredDistanceBetweenVoxels)
: m_maxLabelCount(maxLabelCount),
  m_maxSquaredDistanceBetweenVoxels(maxSquaredDistanceBetweenVoxels)
{}

//#################### DESTRUCTOR ####################

LabelSmoother::~LabelSmoother() {}

}
