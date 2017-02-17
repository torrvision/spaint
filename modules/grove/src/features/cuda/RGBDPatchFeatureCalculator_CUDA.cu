/**
 * grove: RGBDPatchFeatureCalculator_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "features/cuda/RGBDPatchFeatureCalculator_CUDA.h"
#include "features/cuda/RGBDPatchFeatureCalculator_CUDA.tcu"

namespace grove
{

//#################### EXPLICIT INSTANTIATIONS ####################

template class RGBDPatchFeatureCalculator_CUDA<Keypoint2D, RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator_CUDA<Keypoint3DColour, RGBDPatchDescriptor>;

}
