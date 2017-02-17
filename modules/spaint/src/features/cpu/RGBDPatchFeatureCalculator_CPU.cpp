/**
 * spaint: RGBDPatchFeatureCalculator_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "features/cpu/RGBDPatchFeatureCalculator_CPU.h"
#include "features/cpu/RGBDPatchFeatureCalculator_CPU.tpp"

namespace spaint
{

//#################### EXPLICIT INSTANTIATIONS ####################

template class RGBDPatchFeatureCalculator_CPU<Keypoint2D, RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator_CPU<Keypoint3DColour, RGBDPatchDescriptor>;

}
