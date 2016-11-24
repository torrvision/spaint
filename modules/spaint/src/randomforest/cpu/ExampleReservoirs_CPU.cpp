/**
 * spaint: ExampleReservoirs_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/cpu/ExampleReservoirs_CPU.h"
#include "randomforest/cpu/ExampleReservoirs_CPU.tpp"

namespace spaint
{
// TODO: Duplicated with the cuda file, move somewhere else
template<>
_CPU_AND_GPU_CODE_
inline Keypoint3DColour make_example_from_feature<Keypoint3DColour,
Keypoint3DColour>(const Keypoint3DColour &feature)
{
  return feature;
}

template class ExampleReservoirs_CPU<Keypoint3DColour, Keypoint3DColour,
    LeafIndices> ;
}
