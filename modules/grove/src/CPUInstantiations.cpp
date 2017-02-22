/**
 * grove: CPUInstantiations.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "features/cpu/RGBDPatchFeatureCalculator_CPU.tpp"
#include "features/interface/RGBDPatchFeatureCalculator.tpp"

#include "reservoirs/cpu/ExampleReservoirs_CPU.tpp"
#include "reservoirs/interface/ExampleReservoirs.tpp"
#include "reservoirs/ExampleReservoirsFactory.tpp"

namespace grove {

//#################### EXPLICIT INSTANTIATIONS ####################

template class RGBDPatchFeatureCalculator<Keypoint2D,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator<Keypoint3DColour,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator_CPU<Keypoint2D,RGBDPatchDescriptor>;
template class RGBDPatchFeatureCalculator_CPU<Keypoint3DColour,RGBDPatchDescriptor>;

template class ExampleReservoirs<Keypoint3DColour, ORUtils::VectorX<int, 5> >;
template class ExampleReservoirs_CPU<Keypoint3DColour, ORUtils::VectorX<int, 5> >;
template class ExampleReservoirsFactory<Keypoint3DColour, ORUtils::VectorX<int, 5> >;

}
