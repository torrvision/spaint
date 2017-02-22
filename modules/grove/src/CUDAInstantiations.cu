/**
 * grove: CUDAInstantiations.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "features/cuda/RGBDPatchFeatureCalculator_CUDA.tcu"

#include "reservoirs/cuda/ExampleReservoirs_CUDA.tcu"

namespace grove {

//#################### EXPLICIT INSTANTIATIONS ####################

template class RGBDPatchFeatureCalculator_CUDA<Keypoint2D,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator_CUDA<Keypoint3DColour,RGBDPatchDescriptor>;

template class ExampleReservoirs_CUDA<Keypoint3DColour, ORUtils::VectorX<int, 5> >;

}
