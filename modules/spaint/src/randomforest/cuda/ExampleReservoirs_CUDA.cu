/**
 * spaint: ExampleReservoirs_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "ORUtils/PlatformIndependence.h"
#include "randomforest/cuda/ExampleReservoirs_CUDA.h"
#include "util/MemoryBlockFactory.h"

#include <device_atomic_functions.h>

using namespace tvgutil;

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

template<typename ExampleType, typename FeatureType>
_CPU_AND_GPU_CODE_
inline ExampleType make_example_from_feature(const FeatureType &feature);

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

template<typename ExampleType, typename FeatureType, typename LeafType>
__global__ void ck_add_examples(const FeatureType *features,
    const LeafType *leafIndices, Vector2i imgSize, CUDARNG *randomStates,
    ExampleType *reservoirs, int *reservoirSize, int *reservoirAddCalls,
    uint32_t reservoirCapacity)
{
  const int x = threadIdx.x + blockIdx.x * blockDim.x;
  const int y = threadIdx.y + blockIdx.y * blockDim.y;

  if (x >= imgSize.x || y >= imgSize.y)
    return;

  const int linearIdx = y * imgSize.x + x;

  const FeatureType &feature = features[linearIdx];
  const LeafType leaves = leafIndices[linearIdx];

  if (!feature.valid())
    return;

  ExampleType example = make_example_from_feature<ExampleType>(feature);

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
      const int randomIdx = randomStates[linearIdx].generate_int_from_uniform(0,
          addCallsCount);
//      const int randomIdx = static_cast<int>(truncf(
//          curand_uniform(&randomStates[linearIdx])
//              * (reservoirCapacity + 0.999999f)));

      if (randomIdx < reservoirCapacity)
      {
        reservoirs[reservoirStartIdx + randomIdx] = example;
      }
    }
  }
}

template<typename ExampleType, typename FeatureType, typename LeafType>
ExampleReservoirs_CUDA<ExampleType, FeatureType, LeafType>::ExampleReservoirs_CUDA(
    size_t capacity, size_t nbLeaves, uint32_t rngSeed) :
    ExampleReservoirs<ExampleType, FeatureType, LeafType>(capacity, nbLeaves,
        rngSeed)
{
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();
  m_randomStates = mbf.make_block<CUDARNG>();

  // Update device for the other variables (not m_data since its content are meaningless now)
  m_reservoirsAddCalls->UpdateDeviceFromHost();
  m_reservoirsSize->UpdateDeviceFromHost();

  init_random();
}

template<typename ExampleType, typename FeatureType, typename LeafType>
void ExampleReservoirs_CUDA<ExampleType, FeatureType, LeafType>::add_examples(
    const FeatureImage_CPtr &features, const LeafImage_CPtr &leafIndices)
{
  const Vector2i imgSize = features->noDims;
  const int nbExamples = imgSize.width * imgSize.height;

  // Check that we have enough random states and if not reallocate them
  if (nbExamples > m_randomStates->dataSize)
  {
    m_randomStates->ChangeDims(nbExamples);
    init_random();
  }

  const FeatureType *featureData = features->GetData(MEMORYDEVICE_CUDA);
  const LeafIndices *leafIndicesData = leafIndices->GetData(MEMORYDEVICE_CUDA);

  CUDARNG *randomStates = m_randomStates->GetData(MEMORYDEVICE_CUDA);
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

template<typename ExampleType, typename FeatureType, typename LeafType>
void ExampleReservoirs_CUDA<ExampleType, FeatureType, LeafType>::clear()
{
  m_reservoirsSize->Clear();
  m_reservoirsAddCalls->Clear();
  init_random();
}

template<typename ExampleType, typename FeatureType, typename LeafType>
void ExampleReservoirs_CUDA<ExampleType, FeatureType, LeafType>::init_random()
{
  const size_t nbStates = m_randomStates->dataSize;

  if (nbStates == 0)
    return;

  CUDARNG *randomStates = m_randomStates->GetData(MEMORYDEVICE_CUDA);

  // Initialize random states
  dim3 blockSize(256);
  dim3 gridSize((nbStates + blockSize.x - 1) / blockSize.x);
  ck_init_random_states<<<gridSize, blockSize>>>(randomStates, nbStates, m_rngSeed);
  ORcudaKernelCheck;
}

template class ExampleReservoirs_CUDA<PositionColourExample, RGBDPatchFeature,
    LeafIndices> ;
}
