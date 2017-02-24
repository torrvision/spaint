/**
 * grove: CPUInstantiations.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "clustering/ExampleClustererFactory.tpp"
#include "clustering/base/Prediction3DColour.h"
#include "clustering/interface/ExampleClusterer.tpp"

#include "features/cpu/RGBDPatchFeatureCalculator_CPU.tpp"
#include "features/interface/RGBDPatchFeatureCalculator.tpp"

#include "reservoirs/ExampleReservoirsFactory.tpp"
#include "reservoirs/cpu/ExampleReservoirs_CPU.tpp"
#include "reservoirs/interface/ExampleReservoirs.tpp"

namespace grove {

//#################### EXPLICIT INSTANTIATIONS ####################

template class ExampleClusterer<Keypoint3DColour, Prediction3DColour>;
template class ExampleClustererFactory<Keypoint3DColour, Prediction3DColour>;

template class RGBDPatchFeatureCalculator<Keypoint2D,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator<Keypoint3DColour,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator_CPU<Keypoint2D,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator_CPU<Keypoint3DColour,RGBDPatchDescriptor>;

template class ExampleReservoirs<Keypoint3DColour>;
template void ExampleReservoirs<Keypoint3DColour>::add_examples(const ExampleImage_CPtr&,
    const boost::shared_ptr<ORUtils::Image<ORUtils::VectorX<int, 5> > >&);
template void ExampleReservoirs<Keypoint3DColour>::add_examples(const ExampleImage_CPtr&,
    const boost::shared_ptr<const ORUtils::Image<ORUtils::VectorX<int, 5> > >&);
template class ExampleReservoirs_CPU<Keypoint3DColour>;
template class ExampleReservoirsFactory<Keypoint3DColour>;

}
