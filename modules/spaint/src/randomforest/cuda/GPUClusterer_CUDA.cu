/**
 * spaint: GPUClusterer_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/cuda/GPUClusterer_CUDA.h"

#include "util/MemoryBlockFactory.h"

#include <iostream>

namespace spaint
{
__global__ void ck_compute_density(const PositionColourExample *examples,
    const int *reservoirSizes, float *densities, int reservoirCapacity,
    int startReservoirIdx, float sigma)
{
  // The assumption is that the kernel indices are always valid.
  const int reservoirIdx = blockIdx.x + startReservoirIdx;
  const int reservoirOffset = reservoirIdx * reservoirCapacity;
  const int reservoirSize = reservoirSizes[reservoirIdx];
  const int elementIdx = threadIdx.x;
  const int elementOffset = reservoirOffset + elementIdx;

  const float three_sigma_sq = (3.f * sigma) * (3.f * sigma); // Points farther away have small contribution to the density
  const float minus_one_over_two_sigma_sq = -1.f / (2.f * sigma * sigma);

  float density = 0.f;

  if (elementIdx < reservoirSize)
  {
    const Vector3f centerPosition = examples[elementOffset].position;

    for (int i = 0; i < reservoirSize; ++i)
    {
      const Vector3f examplePosition = examples[reservoirOffset + i].position;
      const Vector3f diff = examplePosition - centerPosition;
      const float normSq = dot(diff, diff);

      if (normSq < three_sigma_sq)
      {
        density += expf(normSq * minus_one_over_two_sigma_sq);
      }
    }
  }

  densities[elementOffset] = density;
}

__global__ void ck_link_neighbors(const PositionColourExample *examples,
    const int *reservoirSizes, const float *densities, int *parents,
    int *clusterIndices, int *nbClustersPerReservoir, int reservoirCapacity,
    int startReservoirIdx, float tauSq)
{
  // The assumption is that the kernel indices are always valid.
  const int reservoirIdx = blockIdx.x + startReservoirIdx;
  const int reservoirOffset = reservoirIdx * reservoirCapacity;
  const int reservoirSize = reservoirSizes[reservoirIdx];
  const int elementIdx = threadIdx.x;
  const int elementOffset = reservoirOffset + elementIdx;

  int parentIdx = elementIdx;
  int clusterIdx = -1;

  if (elementIdx < reservoirSize)
  {
    const Vector3f centerPosition = examples[elementOffset].position;
    const float centerDensity = densities[elementOffset];
    float minDistance = tauSq;

    for (int i = 0; i < reservoirSize; ++i)
    {
      if (i == elementIdx)
        continue;

      const Vector3f examplePosition = examples[reservoirOffset + i].position;
      const float exampleDensity = densities[reservoirOffset + i];

      const Vector3f diff = examplePosition - centerPosition;
      const float normSq = dot(diff, diff);

      if (normSq < minDistance && centerDensity < exampleDensity)
      {
        minDistance = normSq;
        parentIdx = i;
      }
    }

    // current element is the root of a subtree, get a unique cluster index
    if (parentIdx == elementIdx)
    {
      clusterIdx = atomicAdd(&nbClustersPerReservoir[reservoirIdx], 1);
    }
  }

  parents[elementOffset] = parentIdx;
  clusterIndices[elementOffset] = clusterIdx;
}

__global__ void ck_identify_cluster(const int *reservoirSizes,
    const int *parents, int *clusterIndices, int reservoirCapacity,
    int startReservoirIdx)
{
  // The assumption is that the kernel indices are always valid.
  const int reservoirIdx = blockIdx.x + startReservoirIdx;
  const int reservoirOffset = reservoirIdx * reservoirCapacity;
  const int reservoirSize = reservoirSizes[reservoirIdx];
  const int elementIdx = threadIdx.x;
  const int elementOffset = reservoirOffset + elementIdx;

  // No need to check if the current element is valid
  // ck_link_neighbors sets the parent for invalid elements to themselves
  int parentIdx = parents[elementOffset];
  int currentIdx = elementIdx;
  while (parentIdx != currentIdx)
  {
    currentIdx = parentIdx;
    parentIdx = parents[reservoirOffset + parentIdx];
  }

  // found the root of the subtree, get its cluster idx
  const int clusterIdx = clusterIndices[reservoirOffset + parentIdx];
  clusterIndices[elementOffset] = clusterIdx;
}

GPUClusterer_CUDA::GPUClusterer_CUDA(float sigma, float tau) :
    GPUClusterer(sigma, tau)
{
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();
  m_densities = mbf.make_image<float>(Vector2i(0, 0));
  m_parents = mbf.make_image<int>(Vector2i(0, 0));
  m_clusterIdx = mbf.make_image<int>(Vector2i(0, 0));
  m_nbClustersPerReservoir = mbf.make_image<int>(Vector2i(0, 0));
}

void GPUClusterer_CUDA::find_modes(const PositionReservoir_CPtr &reservoirs,
    GPUForestPredictionsBlock_Ptr &predictions, size_t startIdx, size_t count)
{
  const int nbReservoirs = reservoirs->get_reservoirs_count();
  const int reservoirCapacity = reservoirs->get_capacity();

  if (startIdx + count > nbReservoirs)
    throw std::runtime_error("startIdx + count > nbReservoirs");

  {
    // Happens only once
    const Vector2i temporariesSize(reservoirCapacity, nbReservoirs);
    m_densities->ChangeDims(temporariesSize);
    m_parents->ChangeDims(temporariesSize);
    m_clusterIdx->ChangeDims(temporariesSize);
    m_nbClustersPerReservoir->ChangeDims(Vector2i(1, nbReservoirs));
  }

  m_nbClustersPerReservoir->Clear();

  const PositionColourExample *examples = reservoirs->get_reservoirs()->GetData(
      MEMORYDEVICE_CUDA);
  const int *reservoirSizes = reservoirs->get_reservoirs_size()->GetData(
      MEMORYDEVICE_CUDA);
  float *densities = m_densities->GetData(MEMORYDEVICE_CUDA);

  dim3 blockSize(reservoirCapacity); // One thread per item in each reservoir
  dim3 gridSize(count); // One block per reservoir to process
  ck_compute_density<<<gridSize, blockSize>>>(examples, reservoirSizes, densities, reservoirCapacity,
      startIdx, m_sigma);
//  cudaDeviceSynchronize();

  int *parents = m_parents->GetData(MEMORYDEVICE_CUDA);
  int *clusterIndices = m_clusterIdx->GetData(MEMORYDEVICE_CUDA);
  int *nbClustersPerReservoir = m_nbClustersPerReservoir->GetData(
      MEMORYDEVICE_CUDA);

  ck_link_neighbors<<<gridSize, blockSize>>>(examples, reservoirSizes, densities, parents, clusterIndices,
      nbClustersPerReservoir, reservoirCapacity, startIdx, m_tau * m_tau);
//  cudaDeviceSynchronize();

  ck_identify_cluster<<<gridSize, blockSize>>>(reservoirSizes, parents, clusterIndices,
      reservoirCapacity, startIdx);
  cudaDeviceSynchronize();

//  m_nbClustersPerReservoir->UpdateHostFromDevice();
//  reservoirs->get_reservoirs_size()->UpdateHostFromDevice();
//
//  for (int i = 0; i < count; ++i)
//  {
//    std::cout << "Reservoir " << i + startIdx << " has "
//        << m_nbClustersPerReservoir->GetData(MEMORYDEVICE_CPU)[i + startIdx]
//        << " clusters and "
//        << reservoirs->get_reservoirs_size()->GetData(MEMORYDEVICE_CPU)[i
//            + startIdx] << " elements." << std::endl;
//  }
}

}
