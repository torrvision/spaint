/**
 * grove: RGBDPatchFeatureCalculator.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "features/interface/RGBDPatchFeatureCalculator.h"
#include "features/interface/RGBDPatchFeatureCalculator.tpp"

//#################### EXPLICIT INSTANTIATIONS ####################

namespace grove {

template class RGBDPatchFeatureCalculator<Keypoint2D, RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator<Keypoint3DColour, RGBDPatchDescriptor>;

}
