/**
 * grove: CUDAInstantiations.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "clustering/cuda/ExampleClusterer_CUDA.tcu"
#include "features/cuda/RGBDPatchFeatureCalculator_CUDA.tcu"
#include "forests/cuda/DecisionForest_CUDA.tcu"
#include "reservoirs/cuda/ExampleReservoirs_CUDA.tcu"
#include "scoreforests/Keypoint3DColourCluster.h"
#include "scoreforests/ScorePrediction.h"
using namespace boost;
using namespace ORUtils;

namespace grove {

namespace {
  static const int FOREST_TREES = 5; // TODO change
}

//#################### EXPLICIT INSTANTIATIONS ####################

template class DecisionForest_CUDA<RGBDPatchDescriptor,FOREST_TREES>;
template class ExampleClusterer_CUDA<Keypoint3DColour, Keypoint3DColourCluster, ScorePrediction::Capacity>;
template class ExampleReservoirs_CUDA<Keypoint2D>;
template class ExampleReservoirs_CUDA<Keypoint3DColour>;
template class RGBDPatchFeatureCalculator_CUDA<Keypoint2D,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator_CUDA<Keypoint3DColour,RGBDPatchDescriptor>;

template void ExampleReservoirs_CUDA<Keypoint2D>::add_examples_sub(const ExampleImage_CPtr&, const shared_ptr<const Image<VectorX<int,FOREST_TREES> > >&);
template void ExampleReservoirs_CUDA<Keypoint3DColour>::add_examples_sub(const ExampleImage_CPtr&, const shared_ptr<const Image<VectorX<int,FOREST_TREES> > >&);

}
