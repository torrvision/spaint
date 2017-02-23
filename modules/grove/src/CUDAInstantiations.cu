/**
 * grove: CUDAInstantiations.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "clustering/base/Prediction3DColour.h"
#include "clustering/cuda/ExampleClusterer_CUDA.tcu"

#include "features/cuda/RGBDPatchFeatureCalculator_CUDA.tcu"

#include "reservoirs/cuda/ExampleReservoirs_CUDA.tcu"

namespace grove {

//#################### EXPLICIT INSTANTIATIONS ####################

template class ExampleClusterer_CUDA<Keypoint3DColour, Prediction3DColour>;

template class RGBDPatchFeatureCalculator_CUDA<Keypoint2D,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator_CUDA<Keypoint3DColour,RGBDPatchDescriptor>;

template class ExampleReservoirs_CUDA<Keypoint3DColour, ORUtils::VectorX<int, 5> >;

}
