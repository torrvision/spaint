/**
 * spaint: ExampleReservoirs_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "reservoirs/cuda/ExampleReservoirs_CUDA.h"
#include "reservoirs/cuda/ExampleReservoirs_CUDA.tcu"

namespace grove
{

//#################### CUDA KERNELS ####################

__global__ void ck_init_random_states(CUDARNG *randomStates, uint32_t nbStates, uint32_t seed)
{
  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if (idx >= nbStates)
    return;

  randomStates[idx].reset(seed, idx);
}

}
