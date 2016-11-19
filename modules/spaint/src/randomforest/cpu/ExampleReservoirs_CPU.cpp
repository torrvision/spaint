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
inline PositionColourExample make_example_from_feature<PositionColourExample,
    RGBDPatchFeature>(const RGBDPatchFeature &feature)
{
  PositionColourExample res;
  res.position = feature.position.toVector3();
  res.colour = feature.colour;

  return res;
}

template class ExampleReservoirs_CPU<PositionColourExample, RGBDPatchFeature,
    LeafIndices> ;
}
