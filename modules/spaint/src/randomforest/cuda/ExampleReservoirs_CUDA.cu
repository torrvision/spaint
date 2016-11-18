/**
 * spaint: ExampleReservoirs_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/cuda/ExampleReservoirs_CUDA.h"
#include "randomforest/cuda/ExampleReservoirs_CUDA.tcu"

namespace spaint
{

__global__ void ck_init_random_states(CUDARNG *randomStates, uint32_t nbStates,
    uint32_t seed)
{
  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if (idx >= nbStates)
    return;

  randomStates[idx].reset(seed, idx);
}

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

template class ExampleReservoirs_CUDA<PositionColourExample, RGBDPatchFeature,
    LeafIndices> ;
}
