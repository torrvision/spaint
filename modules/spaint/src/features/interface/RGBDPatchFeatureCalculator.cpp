/**
 * spaint: RGBDPatchFeatureCalculator.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "features/interface/RGBDPatchFeatureCalculator.h"
#include "features/interface/RGBDPatchFeatureCalculator.tpp"

//#################### EXPLICIT INSTANTIATIONS ####################

namespace spaint {

template class RGBDPatchFeatureCalculator<Keypoint3DColour, RGBDPatchDescriptor>;

}
