/**
 * spaint: PreemptiveRansac_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/cuda/PreemptiveRansac_CUDA.h"
#include "randomforest/shared/PreemptiveRansac_Shared.h"

#include "util/MemoryBlockFactory.h"

#include <Eigen/Dense>

#include <thrust/device_ptr.h>
#include <thrust/execution_policy.h>
#include <thrust/sort.h>

using namespace tvgutil;

namespace spaint
{

namespace
{
__global__ void ck_init_random_generators(CUDARNG *randomGenerators,
    uint32_t nbStates, uint32_t seed)
{
  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if (idx >= nbStates)
    return;

  randomGenerators[idx].reset(seed, idx);
}

__global__ void ck_reset_pose_candidates(PoseCandidates *poseCandidates)
{
  poseCandidates->nbCandidates = 0;
}

template<typename RNG>
__global__ void ck_generate_pose_candidates(const RGBDPatchFeature *features,
    const GPUForestPrediction *predictions, const Vector2i imgSize,
    RNG *randomGenerators, PoseCandidate *poseCandidates, int *nbPoseCandidates,
    int maxNbPoseCandidates,
    bool m_useAllModesPerLeafInPoseHypothesisGeneration,
    bool m_checkMinDistanceBetweenSampledModes,
    float m_minDistanceBetweenSampledModes,
    bool m_checkRigidTransformationConstraint,
    float m_translationErrorMaxForCorrectPose)
{
  const int candidateIdx = blockIdx.x * blockDim.x + threadIdx.x;

  if (candidateIdx >= maxNbPoseCandidates)
    return;

  PoseCandidate candidate;
  candidate.cameraId = candidateIdx;

  bool valid = preemptive_ransac_generate_candidate(features, predictions,
      imgSize, randomGenerators[candidateIdx], candidate,
      m_useAllModesPerLeafInPoseHypothesisGeneration,
      m_checkMinDistanceBetweenSampledModes, m_minDistanceBetweenSampledModes,
      m_checkRigidTransformationConstraint,
      m_translationErrorMaxForCorrectPose);

  if (valid)
  {
    const int candidateIdx = atomicAdd(nbPoseCandidates, 1);
    poseCandidates[candidateIdx] = candidate;
  }
}

__global__ void ck_compute_energies(const RGBDPatchFeature *features,
    const GPUForestPrediction *predictions, PoseCandidate *poseCandidates,
    int nbCandidates)
{
  const int tId = threadIdx.x;
  const int threadsPerBlock = blockDim.x;
  const int candidateIdx = blockIdx.x;

  if (candidateIdx >= nbCandidates)
  {
    // Candidate has been trimmed, entire block returns,
    // does not cause troubles with the following __syncthreads()
    return;
  }

  PoseCandidate &currentCandidate = poseCandidates[candidateIdx];

  float localEnergy = 0.f;

  const int nbInliers = currentCandidate.nbInliers;
  for (int inlierIdx = tId; inlierIdx < nbInliers; inlierIdx += threadsPerBlock)
  {
    const int linearIdx = currentCandidate.inliers[inlierIdx].linearIdx;
    const Vector3f localPixel = features[linearIdx].position.toVector3();
    const Vector3f projectedPixel = currentCandidate.cameraPose * localPixel;

    const GPUForestPrediction &pred = predictions[linearIdx];

    // eval individual energy
    float energy;
    int argmax = pred.get_best_mode_and_energy(projectedPixel, energy);

    // Has at least a valid mode
    if (argmax < 0)
    {
      // should have not been inserted in the inlier set
      printf("prediction has no valid modes\n");
      continue;
    }

    if (pred.modes[argmax].nbInliers == 0)
    {
      // the original implementation had a simple continue
      printf("mode has no inliers\n");
      continue;
    }

    energy /= static_cast<float>(pred.nbModes);
    energy /= static_cast<float>(pred.modes[argmax].nbInliers);

    if (energy < 1e-6f)
      energy = 1e-6f;
    energy = -log10f(energy);

    currentCandidate.inliers[inlierIdx].energy = energy;
    currentCandidate.inliers[inlierIdx].modeIdx = argmax;
    localEnergy += energy;
  }

  // Now reduce by shuffling down the local energies
  //(localEnergy for thread 0 in the warp contains the sum for the warp)
  for (int offset = warpSize / 2; offset > 0; offset /= 2)
    localEnergy += __shfl_down(localEnergy, offset);

  // Thread 0 of each warp updates the final energy
  if ((threadIdx.x & (warpSize - 1)) == 0)
    atomicAdd(&currentCandidate.energy, localEnergy);

  __syncthreads(); // Wait for all threads in the block

  // tId 0 computes the final energy
  if (tId == 0)
    currentCandidate.energy = currentCandidate.energy
        / static_cast<float>(currentCandidate.nbInliers);
}

__global__ void ck_reset_candidate_energies(PoseCandidate *poseCandidates,
    int nbPoseCandidates)
{
  const int candidateIdx = threadIdx.x;

  if (candidateIdx >= nbPoseCandidates)
  {
    return;
  }

  poseCandidates[candidateIdx].energy = 0.f;
}
}

PreemptiveRansac_CUDA::PreemptiveRansac_CUDA() :
    PreemptiveRansac()
{
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();
  m_randomGenerators = mbf.make_block<CUDARNG>(PoseCandidates::MAX_CANDIDATES);
  m_rngSeed = 42;
  m_nbPoseCandidates_device = mbf.make_image<int>(Vector2i(1, 1));

  init_random();
}

void PreemptiveRansac_CUDA::init_random()
{
  CUDARNG *randomGenerators = m_randomGenerators->GetData(MEMORYDEVICE_CUDA);

  // Initialize random states
  dim3 blockSize(256);
  dim3 gridSize(
      (PoseCandidates::MAX_CANDIDATES + blockSize.x - 1) / blockSize.x);

  ck_init_random_generators<<<gridSize, blockSize>>>(randomGenerators, PoseCandidates::MAX_CANDIDATES, m_rngSeed);
}

void PreemptiveRansac_CUDA::generate_pose_candidates()
{
  const Vector2i imgSize = m_featureImage->noDims;
  const RGBDPatchFeature *features = m_featureImage->GetData(MEMORYDEVICE_CUDA);
  const GPUForestPrediction *predictions = m_predictionsImage->GetData(
      MEMORYDEVICE_CUDA);

  CUDARNG *randomGenerators = m_randomGenerators->GetData(MEMORYDEVICE_CUDA);
  PoseCandidate *poseCandidates = m_poseCandidates->GetData(MEMORYDEVICE_CUDA);
  int *nbPoseCandidates = m_nbPoseCandidates_device->GetData(MEMORYDEVICE_CUDA);

  dim3 blockSize(32);
  dim3 gridSize(
      (PoseCandidates::MAX_CANDIDATES + blockSize.x - 1) / blockSize.x);

  // Reset number of candidates (only on device, the host number will be updated later)
  ORcudaSafeCall(cudaMemsetAsync(nbPoseCandidates, 0, sizeof(int)));

  ck_generate_pose_candidates<<<gridSize, blockSize>>>(features, predictions, imgSize, randomGenerators,
      poseCandidates, nbPoseCandidates, PoseCandidates::MAX_CANDIDATES,
      m_useAllModesPerLeafInPoseHypothesisGeneration,
      m_checkMinDistanceBetweenSampledModes, m_minDistanceBetweenSampledModes,
      m_checkRigidTransformationConstraint,
      m_translationErrorMaxForCorrectPose);
  ORcudaKernelCheck;

  // Need to make the data available to the host
  m_poseCandidates->UpdateHostFromDevice();
  m_nbPoseCandidates = m_nbPoseCandidates_device->GetElement(0,
      MEMORYDEVICE_CUDA);

  // Now perform kabsch on the candidates
  //#ifdef ENABLE_TIMERS
  //    boost::timer::auto_cpu_timer t(6,
  //        "kabsch: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
  //#endif
  compute_candidate_pose_kabsch();
}

void PreemptiveRansac_CUDA::compute_and_sort_energies()
{
  PreemptiveRansac::compute_and_sort_energies();
  return;

//  // Need to make the data available to the device
//  m_poseCandidates->UpdateDeviceFromHost();
//  m_nbPoseCandidates->UpdateDeviceFromHost();
//
//  const RGBDPatchFeature *features = m_featureImage->GetData(MEMORYDEVICE_CUDA);
//  const GPUForestPrediction *predictions = m_predictionsImage->GetData(
//      MEMORYDEVICE_CUDA);
//  PoseCandidate *poseCandidates = m_poseCandidates->GetData(MEMORYDEVICE_CUDA);
//  const int nbPoseCandidates = *m_nbPoseCandidates->GetData(MEMORYDEVICE_CPU);
//
//  ck_reset_candidate_energies<<<1, nbPoseCandidates>>>(poseCandidates, nbPoseCandidates);
//  ORcudaKernelCheck;
//
//  dim3 blockSize(128); // threads to compute the energy for each candidate
//  dim3 gridSize(nbPoseCandidates); // Launch one block per candidate (many blocks will exit immediately in the later stages of ransac)
//  ck_compute_energies<<<gridSize, blockSize>>>(features, predictions, poseCandidates, nbPoseCandidates);
//  ORcudaKernelCheck;
//
//  // Sort by ascending energy
//  thrust::device_ptr<PoseCandidate> candidatesStart(poseCandidates);
//  thrust::device_ptr<PoseCandidate> candidatesEnd(
//      poseCandidates + nbPoseCandidates);
//  thrust::sort(candidatesStart, candidatesEnd);
//
//  // Need to make the data available to the host once again
//  m_poseCandidates->UpdateHostFromDevice();
}

void PreemptiveRansac_CUDA::compute_candidate_pose_kabsch()
{
  const RGBDPatchFeature *features = m_featureImage->GetData(MEMORYDEVICE_CPU);
  const GPUForestPrediction *predictions = m_predictionsImage->GetData(
      MEMORYDEVICE_CPU);
  PoseCandidate *poseCandidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);

//  std::cout << "Generated " << nbPoseCandidates << " candidates." << std::endl;

#pragma omp parallel for
  for (int candidateIdx = 0; candidateIdx < m_nbPoseCandidates; ++candidateIdx)
  {
    PoseCandidate &candidate = poseCandidates[candidateIdx];

    Eigen::MatrixXf localPoints(3, candidate.nbInliers);
    Eigen::MatrixXf worldPoints(3, candidate.nbInliers);
    for (int s = 0; s < candidate.nbInliers; ++s)
    {
      const int linearIdx = candidate.inliers[s].linearIdx;
      const int modeIdx = candidate.inliers[s].modeIdx;
      const GPUForestPrediction &pred = predictions[linearIdx];

      localPoints.col(s) = Eigen::Map<const Eigen::Vector3f>(
          features[linearIdx].position.v);
      worldPoints.col(s) = Eigen::Map<const Eigen::Vector3f>(
          pred.modes[modeIdx].position.v);
    }

    Eigen::Map<Eigen::Matrix4f>(candidate.cameraPose.m) = Kabsch(localPoints,
        worldPoints);
  }
}

}
