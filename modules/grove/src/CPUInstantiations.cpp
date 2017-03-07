/**
 * grove: CPUInstantiations.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "clustering/ExampleClustererFactory.tpp"
#include "clustering/base/Prediction3DColour.h"
#include "clustering/interface/ExampleClusterer.tpp"

#include "features/cpu/RGBDPatchFeatureCalculator_CPU.tpp"
#include "features/interface/RGBDPatchFeatureCalculator.tpp"

#include "forests/DecisionForestFactory.tpp"
#include "forests/cpu/DecisionForest_CPU.tpp"
#include "forests/interface/DecisionForest.tpp"

#include "reservoirs/ExampleReservoirsFactory.tpp"
#include "reservoirs/cpu/ExampleReservoirs_CPU.tpp"
#include "reservoirs/interface/ExampleReservoirs.tpp"

namespace grove {

namespace {
  static const int FOREST_TREES = 5;
}

//#################### EXPLICIT INSTANTIATIONS ####################

template class ExampleClusterer<Keypoint3DColour, Prediction3DColour>;
template class ExampleClustererFactory<Keypoint3DColour, Prediction3DColour>;

template class RGBDPatchFeatureCalculator<Keypoint2D,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator<Keypoint3DColour,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator_CPU<Keypoint2D,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator_CPU<Keypoint3DColour,RGBDPatchDescriptor>;

template class DecisionForest<RGBDPatchDescriptor, FOREST_TREES>;
template class DecisionForest_CPU<RGBDPatchDescriptor, FOREST_TREES>;
template class DecisionForestFactory<RGBDPatchDescriptor, FOREST_TREES>;

template class ExampleReservoirs<Keypoint3DColour>;
template void ExampleReservoirs<Keypoint3DColour>::add_examples(const ExampleImage_CPtr&,
    const boost::shared_ptr<ORUtils::Image<ORUtils::VectorX<int, FOREST_TREES> > >&);
template void ExampleReservoirs<Keypoint3DColour>::add_examples(const ExampleImage_CPtr&,
    const boost::shared_ptr<const ORUtils::Image<ORUtils::VectorX<int, FOREST_TREES> > >&);
template class ExampleReservoirs_CPU<Keypoint3DColour>;
template class ExampleReservoirsFactory<Keypoint3DColour>;

template class ExampleReservoirs<Keypoint2D>;
template void ExampleReservoirs<Keypoint2D>::add_examples(const ExampleImage_CPtr&,
    const boost::shared_ptr<ORUtils::Image<ORUtils::VectorX<int, FOREST_TREES> > >&);
template void ExampleReservoirs<Keypoint2D>::add_examples(const ExampleImage_CPtr&,
    const boost::shared_ptr<const ORUtils::Image<ORUtils::VectorX<int, FOREST_TREES> > >&);
template class ExampleReservoirs_CPU<Keypoint2D>;
template class ExampleReservoirsFactory<Keypoint2D>;

}
