/**
 * spaint: GPUReservoir_CUDA.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/cuda/GPUReservoir_CUDA.h"
#include "util/MemoryBlockFactory.h"

#include <device_atomic_functions.h>

namespace spaint
{

__global__ void ck_init_random_states(
    GPUReservoir_CUDA::RandomState *randomStates, uint32_t nbStates,
    uint32_t seed)
{
  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if (idx >= nbStates)
    return;

  curand_init(seed, idx, 0, &randomStates[idx]);
}

__global__ void ck_add_examples(const RGBDPatchFeature *features,
    const LeafIndices *leafIndices, Vector2i imgSize,
    GPUReservoir_CUDA::RandomState *randomStates,
    GPUReservoir_CUDA::ExampleType *reservoirs, int *reservoirSize,
    int *reservoirAddCalls, uint32_t reservoirCapacity)
{
  const int x = threadIdx.x + blockIdx.x * blockDim.x;
  const int y = threadIdx.y + blockIdx.y * blockDim.y;

  if (x >= imgSize.x || y >= imgSize.y)
    return;

  const int linearIdx = y * imgSize.x + x;

  const RGBDPatchFeature &feature = features[linearIdx];
  const LeafIndices leaves = leafIndices[linearIdx];

  if (!feature.valid())
    return;

  GPUReservoir_CUDA::ExampleType example;
  example.position = feature.position.toVector3();
  example.colour = feature.colour;

  for (int treeIdx = 0; treeIdx < LeafIndices::value_size; ++treeIdx)
  {
    const int leafIdx = leaves[treeIdx]; // The row in the reservoirs array
    const int reservoirStartIdx = leafIdx * reservoirCapacity;

    // Get the number of total add calls
    const int addCallsCount = atomicAdd(&reservoirAddCalls[leafIdx], 1);

    // if addCallsCount is less than capacity we can add the example straightaway
    if (addCallsCount < reservoirCapacity)
    {
      reservoirs[reservoirStartIdx + addCallsCount] = example;
      // Also increment the size of the current reservoir
      // might not be needed, reservoirAddCalls also has the same info but care needs
      // to be taken when reading values since they will be greater than the reservoir capacity
      atomicAdd(&reservoirSize[leafIdx], 1);
    }
    else
    {
      // Generate a random index and check if we have to evict an example
      // curand_uniform generates a number in ]0,1]
      const int randomIdx = static_cast<int>(truncf(
          curand_uniform(&randomStates[linearIdx])
              * (reservoirCapacity + 0.999999f)));

      if (randomIdx < reservoirCapacity)
      {
        reservoirs[reservoirStartIdx + randomIdx] = example;
      }
    }
  }
}

GPUReservoir_CUDA::GPUReservoir_CUDA(size_t capacity, size_t nbLeaves,
    uint32_t rngSeed) :
    GPUReservoir(capacity, nbLeaves, rngSeed)
{
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();
  m_randomStates = mbf.make_block<RandomState>();

  // Update device for the other variables (not m_data since its content are meaningless now)
  m_reservoirsAddCalls->UpdateDeviceFromHost();
  m_reservoirsSize->UpdateDeviceFromHost();

  init_random();
}

void GPUReservoir_CUDA::add_examples(const RGBDPatchFeatureImage_CPtr &features,
    const LeafIndicesImage_CPtr &leafIndices)
{
  const Vector2i imgSize = features->noDims;
  const int nbExamples = imgSize.width * imgSize.height;

  // Check that we have enough random states and if not reallocate them
  if (nbExamples > m_randomStates->dataSize)
  {
    m_randomStates->ChangeDims(nbExamples);
    init_random();
  }

  const RGBDPatchFeature *featureData = features->GetData(MEMORYDEVICE_CUDA);
  const LeafIndices *leafIndicesData = leafIndices->GetData(MEMORYDEVICE_CUDA);

  RandomState *randomStates = m_randomStates->GetData(MEMORYDEVICE_CUDA);
  ExampleType *reservoirData = m_data->GetData(MEMORYDEVICE_CUDA);
  int *reservoirSize = m_reservoirsSize->GetData(MEMORYDEVICE_CUDA);
  int *reservoirAddCalls = m_reservoirsAddCalls->GetData(MEMORYDEVICE_CUDA);

  dim3 blockSize(32, 32);
  dim3 gridSize((imgSize.width + blockSize.x - 1) / blockSize.x,
      (imgSize.height + blockSize.y - 1) / blockSize.y);
  ck_add_examples<<<gridSize, blockSize>>>(featureData, leafIndicesData, imgSize, randomStates,
      reservoirData, reservoirSize, reservoirAddCalls, m_reservoirCapacity);
  ORcudaKernelCheck;
}

void GPUReservoir_CUDA::clear()
{
  m_reservoirsSize->Clear();
  m_reservoirsAddCalls->Clear();
  init_random();
}

void GPUReservoir_CUDA::init_random()
{
  const int nbStates = m_randomStates->dataSize;
  RandomState *randomStates = m_randomStates->GetData(MEMORYDEVICE_CUDA);

  // Initialize random states
  dim3 blockSize(256);
  dim3 gridSize((nbStates + blockSize.x - 1) / blockSize.x);
  ck_init_random_states<<<gridSize, blockSize>>>(randomStates, nbStates, m_rngSeed);
  ORcudaKernelCheck;
}
}
