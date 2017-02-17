/**
 * grove: RGBDPatchFeatureCalculator.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "features/interface/RGBDPatchFeatureCalculator.tpp"

namespace grove {

//#################### EXPLICIT INSTANTIATIONS ####################

template class RGBDPatchFeatureCalculator<Keypoint2D, RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator<Keypoint3DColour, RGBDPatchDescriptor>;

}
