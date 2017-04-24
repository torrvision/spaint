/**
 * grove: CUDAInstantiations.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "clustering/base/Keypoint3DColourClusteringUtils.h"
#include "clustering/cuda/ExampleClusterer_CUDA.tcu"

#include "features/cuda/RGBDPatchFeatureCalculator_CUDA.tcu"

#include "forests/cuda/DecisionForest_CUDA.tcu"

#include "reservoirs/cuda/ExampleReservoirs_CUDA.tcu"

namespace grove {

namespace {
  static const int FOREST_TREES = 5; // TODO change
}

//#################### EXPLICIT INSTANTIATIONS ####################

template class ExampleClusterer_CUDA<Keypoint3DColour, Mode3DColour>;

template class RGBDPatchFeatureCalculator_CUDA<Keypoint2D,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator_CUDA<Keypoint3DColour,RGBDPatchDescriptor>;

template class DecisionForest_CUDA<RGBDPatchDescriptor, FOREST_TREES>;

template class ExampleReservoirs_CUDA<Keypoint2D>;
template class ExampleReservoirs_CUDA<Keypoint3DColour>;

}
