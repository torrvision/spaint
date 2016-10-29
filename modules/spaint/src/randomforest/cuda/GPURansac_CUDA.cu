/**
 * spaint: GPURansac_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/cuda/GPURansac_CUDA.h"

#include "util/MemoryBlockFactory.h"

namespace spaint
{

namespace
{
__global__ void ck_init_random_states(GPURansac_CUDA::RandomState *randomStates,
    uint32_t nbStates, uint32_t seed)
{
  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if (idx >= nbStates)
    return;

  curand_init(seed, idx, 0, &randomStates[idx]);
}
}

GPURansac_CUDA::GPURansac_CUDA()
{
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();
  m_randomStates = mbf.make_block<RandomState>(m_kInitRansac);
  m_rngSeed = 42;

  init_random();
}

void GPURansac_CUDA::init_random()
{
  RandomState *randomStates = m_randomStates->GetData(MEMORYDEVICE_CUDA);

  // Initialize random states
  dim3 blockSize(256);
  dim3 gridSize((m_kInitRansac + blockSize.x - 1) / blockSize.x);
  ck_init_random_states<<<gridSize, blockSize>>>(randomStates, m_kInitRansac, m_rngSeed);
}

}
