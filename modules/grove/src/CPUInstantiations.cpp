/**
 * grove: CPUInstantiations.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "clustering/ExampleClustererFactory.tpp"
#include "clustering/cpu/ExampleClusterer_CPU.tpp"
#include "clustering/interface/ExampleClusterer.tpp"
#include "features/FeatureCalculatorFactory.tpp"
#include "features/cpu/RGBDPatchFeatureCalculator_CPU.tpp"
#include "features/interface/RGBDPatchFeatureCalculator.tpp"
#include "forests/DecisionForestFactory.tpp"
#include "forests/cpu/DecisionForest_CPU.tpp"
#include "forests/interface/DecisionForest.tpp"
#include "reservoirs/ExampleReservoirsFactory.tpp"
#include "reservoirs/cpu/ExampleReservoirs_CPU.tpp"
#include "reservoirs/interface/ExampleReservoirs.tpp"
#include "scoreforests/Keypoint3DColourCluster.h"
#include "scoreforests/ScorePrediction.h"
using namespace boost;
using namespace ORUtils;

namespace grove {

//#################### CONSTANTS ####################

namespace {
  static const int FOREST_TREES = 5;
}

//#################### EXPLICIT INSTANTIATIONS ####################

template class ExampleClusterer<Keypoint3DColour, Keypoint3DColourCluster, ScorePrediction::Capacity>;
template class ExampleClusterer_CPU<Keypoint3DColour, Keypoint3DColourCluster, ScorePrediction::Capacity>;
template struct ExampleClustererFactory<Keypoint3DColour, Keypoint3DColourCluster, ScorePrediction::Capacity>;

template shared_ptr<RGBDPatchFeatureCalculator<Keypoint2D,RGBDPatchDescriptor> >
FeatureCalculatorFactory::make_custom_patch_feature_calculator<Keypoint2D,RGBDPatchDescriptor>(
  DeviceType deviceType,
  bool depthAdaptive,
  RGBDPatchFeatureDifferenceType depthDifferenceType,
  uint32_t depthFeatureCount,
  uint32_t depthFeatureOffset,
  uint32_t depthMinRadius,
  uint32_t depthMaxRadius,
  RGBDPatchFeatureDifferenceType rgbDifferenceType,
  uint32_t rgbFeatureCount,
  uint32_t rgbFeatureOffset,
  uint32_t rgbMinRadius,
  uint32_t rgbMaxRadius
);

template shared_ptr<RGBDPatchFeatureCalculator<Keypoint3DColour,RGBDPatchDescriptor> >
FeatureCalculatorFactory::make_custom_patch_feature_calculator<Keypoint3DColour,RGBDPatchDescriptor>(
  DeviceType deviceType,
  bool depthAdaptive,
  RGBDPatchFeatureDifferenceType depthDifferenceType,
  uint32_t depthFeatureCount,
  uint32_t depthFeatureOffset,
  uint32_t depthMinRadius,
  uint32_t depthMaxRadius,
  RGBDPatchFeatureDifferenceType rgbDifferenceType,
  uint32_t rgbFeatureCount,
  uint32_t rgbFeatureOffset,
  uint32_t rgbMinRadius,
  uint32_t rgbMaxRadius
);

template class RGBDPatchFeatureCalculator<Keypoint2D,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator<Keypoint3DColour,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator_CPU<Keypoint2D,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator_CPU<Keypoint3DColour,RGBDPatchDescriptor>;

template class DecisionForest<RGBDPatchDescriptor, FOREST_TREES>;
template class DecisionForest_CPU<RGBDPatchDescriptor, FOREST_TREES>;
template struct DecisionForestFactory<RGBDPatchDescriptor, FOREST_TREES>;

template class ExampleReservoirs<Keypoint2D>;
template class ExampleReservoirs<Keypoint3DColour>;
template class ExampleReservoirs_CPU<Keypoint2D>;
template class ExampleReservoirs_CPU<Keypoint3DColour>;
template struct ExampleReservoirsFactory<Keypoint2D>;
template struct ExampleReservoirsFactory<Keypoint3DColour>;

template void ExampleReservoirs<Keypoint2D>::add_examples(const ExampleImage_CPtr&, const shared_ptr<Image<VectorX<int,FOREST_TREES> > >&);
template void ExampleReservoirs<Keypoint2D>::add_examples(const ExampleImage_CPtr&, const shared_ptr<const Image<VectorX<int,FOREST_TREES> > >&);
template void ExampleReservoirs<Keypoint3DColour>::add_examples(const ExampleImage_CPtr&, const shared_ptr<Image<VectorX<int,FOREST_TREES> > >&);
template void ExampleReservoirs<Keypoint3DColour>::add_examples(const ExampleImage_CPtr&, const shared_ptr<const Image<VectorX<int,FOREST_TREES> > >&);

}
