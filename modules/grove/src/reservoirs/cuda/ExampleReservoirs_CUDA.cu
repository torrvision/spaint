/**
 * spaint: ExampleReservoirs_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "reservoirs/cuda/ExampleReservoirs_CUDA.h"
#include "reservoirs/cuda/ExampleReservoirs_CUDA.tcu"

namespace grove {

//#################### CUDA KERNELS ####################

__global__ void ck_init_rngs(CUDARNG *rngs, uint32_t rngCount, uint32_t seed)
{
  int tid = threadIdx.x + blockIdx.x * blockDim.x;
  if(tid < rngCount)
  {
    rngs[tid].reset(seed, tid);
  }
}

}
