/**
 * grove: PreemptiveRansac_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ransac/cuda/PreemptiveRansac_CUDA.h"
using namespace tvgutil;

#ifdef _MSC_VER
  // Suppress some VC++ warnings that are produced when including the Thrust headers.
  #pragma warning(disable:4244 4267)
#endif

#include <thrust/device_ptr.h>
#include <thrust/sort.h>

#ifdef _MSC_VER
  // Reenable the suppressed warnings for the rest of the translation unit.
  #pragma warning(default:4244 4267)
#endif

#include <itmx/base/MemoryBlockFactory.h>
using namespace itmx;

#include "ransac/shared/PreemptiveRansac_Shared.h"

namespace grove {

//#################### CUDA KERNELS ####################

__global__ void ck_compute_energies(const Keypoint3DColour *keypoints, const ScorePrediction *predictions, const int *inlierRasterIndices,
                                    uint32_t nbInliers, PoseCandidate *poseCandidates, int nbCandidates)
{
  const int tid = threadIdx.x;
  const int threadsPerBlock = blockDim.x;
  const int candidateIdx = blockIdx.x;

  if(candidateIdx >= nbCandidates)
  {
    // The candidate has been culled, so early out. Note that since we are using each thread block to
    // compute the energy for a single candidate, the entire block will return in this case, so the
    // __syncthreads() call later in the kernel is safe.
    return;
  }

  PoseCandidate& currentCandidate = poseCandidates[candidateIdx];

  // For each thread in the block, first compute the sum of the energies for a strided subset of the inliers.
  // In particular, thread tid in the block computes the sum of the energies for the inliers with array indices
  // tid + k * threadsPerBlock.
  float energySum = compute_energy_sum_for_inlier_subset(
    currentCandidate.cameraPose, keypoints, predictions, inlierRasterIndices, nbInliers, tid, threadsPerBlock
  );

  // Then, add up the sums computed by the individual threads to compute the overall energy for the candidate.
  // To do this, we perform an efficient, shuffle-based reduction as described in the following blog post:
  // https://devblogs.nvidia.com/parallelforall/faster-parallel-reductions-kepler

  // Step 1: Sum the energies in each warp using downward shuffling, storing the result in the energySum variable of the first thread in the warp.
  for(int offset = warpSize / 2; offset > 0; offset /= 2) energySum += __shfl_down(energySum, offset);

  // Step 2: If this is the first thread in the warp, add the energy sum for the warp to the candidate's energy.
  if((threadIdx.x & (warpSize - 1)) == 0) atomicAdd(&currentCandidate.energy, energySum);

  // Step 3: Wait for all of the atomic adds to finish.
  __syncthreads();

  // Step 4: If this is the first thread in the entire block, compute the final energy for the candidate by dividing by the number of inliers.
  if(tid == 0) currentCandidate.energy = currentCandidate.energy / static_cast<float>(nbInliers);
}

template <typename RNG>
__global__ void ck_generate_pose_candidates(const Keypoint3DColour *keypoints, const ScorePrediction *predictions,
                                            const Vector2i imgSize, RNG *rngs, PoseCandidate *poseCandidates, int *nbPoseCandidates,
                                            uint32_t maxCandidateGenerationIterations, uint32_t maxPoseCandidates,
                                            bool useAllModesPerLeafInPoseHypothesisGeneration, bool checkMinDistanceBetweenSampledModes,
                                            float minDistanceBetweenSampledModes, bool checkRigidTransformationConstraint,
                                            float translationErrorMaxForCorrectPose)
{
  const int candidateIdx = blockIdx.x * blockDim.x + threadIdx.x;
  if(candidateIdx >= maxPoseCandidates) return;

  // Try to generate a valid pose candidate.
  PoseCandidate candidate;
  bool valid = generate_pose_candidate(
    keypoints, predictions, imgSize, rngs[candidateIdx], candidate, maxCandidateGenerationIterations, useAllModesPerLeafInPoseHypothesisGeneration,
    checkMinDistanceBetweenSampledModes, minDistanceBetweenSampledModes, checkRigidTransformationConstraint, translationErrorMaxForCorrectPose
  );

  // If we succeed, grab a unique index in the output array and store the candidate into the corresponding array element.
  if(valid)
  {
    const int finalCandidateIdx = atomicAdd(nbPoseCandidates, 1);
    poseCandidates[finalCandidateIdx] = candidate;
  }
}

__global__ void ck_prepare_inliers_for_optimisation(const Keypoint3DColour *keypoints, const ScorePrediction *predictions, const int *inlierIndices,
                                                    int nbInliers, const PoseCandidate *poseCandidates, int nbPoseCandidates, Vector4f *inlierCameraPoints,
                                                    Keypoint3DColourCluster *inlierModes, float inlierThreshold)
{
  const int candidateIdx = blockIdx.y;
  const int inlierIdx = blockIdx.x * blockDim.x + threadIdx.x;

  if(candidateIdx >= nbPoseCandidates || inlierIdx >= nbInliers) return;

  preemptive_ransac_prepare_inliers_for_optimisation(
    keypoints, predictions, inlierIndices, nbInliers, poseCandidates, inlierCameraPoints, inlierModes, inlierThreshold, candidateIdx, inlierIdx
  );
}

__global__ void ck_reset_candidate_energies(PoseCandidate *poseCandidates, int nbPoseCandidates)
{
  const int candidateIdx = blockIdx.x * blockDim.x + threadIdx.x;
  if(candidateIdx < nbPoseCandidates)
  {
    poseCandidates[candidateIdx].energy = 0.0f;
  }
}

template <bool useMask, typename RNG>
__global__ void ck_sample_inliers(const Keypoint3DColour *keypoints, const ScorePrediction *predictions, const Vector2i imgSize, RNG *rngs,
                                  int *inlierRasterIndices, int *nbInliers, uint32_t ransacInliersPerIteration, int *inliersMask = NULL)
{
  const uint32_t sampleIdx = blockIdx.x * blockDim.x + threadIdx.x;
  if(sampleIdx < ransacInliersPerIteration)
  {
    // Try to sample the raster index of a valid keypoint which prediction has at least one modal cluster, using the mask if necessary.
    const int rasterIdx = sample_inlier<useMask>(keypoints, predictions, imgSize, rngs[sampleIdx], inliersMask);

    // If we succeed, grab a unique index in the output array and store the inlier raster index into the corresponding array element.
    if(rasterIdx >= 0)
    {
      const int arrayIdx = atomicAdd(nbInliers, 1);
      inlierRasterIndices[arrayIdx] = rasterIdx;
    }
  }
}

//#################### CONSTRUCTORS ####################

PreemptiveRansac_CUDA::PreemptiveRansac_CUDA(const SettingsContainer_CPtr& settings)
: PreemptiveRansac(settings)
{
  MemoryBlockFactory& mbf = MemoryBlockFactory::instance();

  // Allocate memory blocks.
  m_nbInliers_device = mbf.make_block<int>(1);        // Size 1, just to store a value that can be accessed from the GPU.
  m_nbPoseCandidates_device = mbf.make_block<int>(1); // As above.
  m_rngs = mbf.make_block<CUDARNG>(m_maxPoseCandidates);

  // Default random seed.
  m_rngSeed = 42;

  // Reset RNGs.
  init_random();
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

void PreemptiveRansac_CUDA::compute_energies_and_sort()
{
  const int *inlierRasterIndices = m_inlierRasterIndicesBlock->GetData(MEMORYDEVICE_CUDA);
  const Keypoint3DColour *keypoints = m_keypointsImage->GetData(MEMORYDEVICE_CUDA);
  const uint32_t nbInliers = static_cast<uint32_t>(m_inlierRasterIndicesBlock->dataSize); // The number of currently sampled inlier points (used to compute the energy).
  const int nbPoseCandidates = static_cast<int>(m_poseCandidates->dataSize);              // The number of currently "valid" pose candidates.
  PoseCandidate *poseCandidates = m_poseCandidates->GetData(MEMORYDEVICE_CUDA);           // The raster indices of the current sampled inlier points.
  const ScorePrediction *predictions = m_predictionsImage->GetData(MEMORYDEVICE_CUDA);

  // Reset the energies for all pose candidates.
  {
    dim3 blockSize(256);
    dim3 gridSize((nbPoseCandidates + blockSize.x - 1) / blockSize.x);
    ck_reset_candidate_energies<<<gridSize,blockSize>>>(poseCandidates, nbPoseCandidates);
    ORcudaKernelCheck;
  }

  // Compute the energies for all pose candidates.
  {
    // Launch one block per candidate (in this way, many blocks will exit immediately in the later stages of P-RANSAC).
    dim3 blockSize(128); // Threads to compute the energy for each candidate.
    dim3 gridSize(nbPoseCandidates);
    ck_compute_energies<<<gridSize,blockSize>>>(keypoints, predictions, inlierRasterIndices, nbInliers, poseCandidates, nbPoseCandidates);
    ORcudaKernelCheck;
  }

  // Sort the candidates into non-increasing order of energy.
  thrust::device_ptr<PoseCandidate> candidatesStart(poseCandidates);
  thrust::device_ptr<PoseCandidate> candidatesEnd(poseCandidates + nbPoseCandidates);
  thrust::sort(candidatesStart, candidatesEnd);
}

void PreemptiveRansac_CUDA::generate_pose_candidates()
{
  const Vector2i imgSize = m_keypointsImage->noDims;
  const Keypoint3DColour *keypoints = m_keypointsImage->GetData(MEMORYDEVICE_CUDA);
  PoseCandidate *poseCandidates = m_poseCandidates->GetData(MEMORYDEVICE_CUDA);
  const ScorePrediction *predictions = m_predictionsImage->GetData(MEMORYDEVICE_CUDA);
  CUDARNG *rngs = m_rngs->GetData(MEMORYDEVICE_CUDA);

  // Reset the number of pose candidates (we do this on the device only at this stage, and update the corresponding host value once we are done generating).
  int *nbPoseCandidates_device = m_nbPoseCandidates_device->GetData(MEMORYDEVICE_CUDA);
  ORcudaSafeCall(cudaMemsetAsync(nbPoseCandidates_device, 0, sizeof(int)));

  // Generate at most m_maxPoseCandidates new pose candidates.
  dim3 blockSize(32);
  dim3 gridSize((m_maxPoseCandidates + blockSize.x - 1) / blockSize.x);

  ck_generate_pose_candidates<<<gridSize,blockSize>>>(
    keypoints, predictions, imgSize, rngs, poseCandidates, nbPoseCandidates_device, m_maxCandidateGenerationIterations,
    m_maxPoseCandidates, m_useAllModesPerLeafInPoseHypothesisGeneration, m_checkMinDistanceBetweenSampledModes,
    m_minSquaredDistanceBetweenSampledModes, m_checkRigidTransformationConstraint, m_maxTranslationErrorForCorrectPose
  );
  ORcudaKernelCheck;

  // Copy all relevant data back across to the host for use by the Kabsch algorithm.
  m_poseCandidates->dataSize = m_nbPoseCandidates_device->GetElement(0, MEMORYDEVICE_CUDA);
  m_poseCandidates->UpdateHostFromDevice();

  // Run Kabsch on all the generated candidates to estimate the rigid transformations.
  compute_candidate_poses_kabsch();

  // Copy the computed rigid transformations back across to the device.
  m_poseCandidates->UpdateDeviceFromHost();
}

void PreemptiveRansac_CUDA::prepare_inliers_for_optimisation()
{
  const Keypoint3DColour *keypointsData = m_keypointsImage->GetData(MEMORYDEVICE_CUDA);
  const ScorePrediction *predictionsData = m_predictionsImage->GetData(MEMORYDEVICE_CUDA);

  const uint32_t nbInliers = static_cast<uint32_t>(m_inlierRasterIndicesBlock->dataSize);
  const int *inlierLinearisedIndicesData = m_inlierRasterIndicesBlock->GetData(MEMORYDEVICE_CUDA);

  const int nbPoseCandidates = static_cast<int>(m_poseCandidates->dataSize);
  const PoseCandidate *poseCandidatesData = m_poseCandidates->GetData(MEMORYDEVICE_CUDA);

  // Grap pointers to the output storage.
  Vector4f *candidateCameraPoints = m_poseOptimisationCameraPoints->GetData(MEMORYDEVICE_CUDA);
  Keypoint3DColourCluster *candidateModes = m_poseOptimisationPredictedModes->GetData(MEMORYDEVICE_CUDA);

  dim3 blockSize(256);
  dim3 gridSize((nbInliers + blockSize.x - 1) / blockSize.x, nbPoseCandidates);

  ck_prepare_inliers_for_optimisation<<<gridSize, blockSize>>>(
    keypointsData, predictionsData, inlierLinearisedIndicesData, nbInliers, poseCandidatesData,
    nbPoseCandidates, candidateCameraPoints, candidateModes, m_poseOptimisationInlierThreshold
  );
  ORcudaKernelCheck;

  // Compute the actual size of the buffers to avoid unnecessary copies.
  const uint32_t poseOptimisationBufferSize = nbInliers * nbPoseCandidates;
  m_poseOptimisationCameraPoints->dataSize = poseOptimisationBufferSize;
  m_poseOptimisationPredictedModes->dataSize = poseOptimisationBufferSize;

  // Make the inlier data available to the optimiser which is running on the CPU.
  m_poseOptimisationCameraPoints->UpdateHostFromDevice();
  m_poseOptimisationPredictedModes->UpdateHostFromDevice();
}

void PreemptiveRansac_CUDA::reset_inliers(bool resetMask)
{
  PreemptiveRansac::reset_inliers(resetMask);
  ORcudaSafeCall(cudaMemsetAsync(m_nbInliers_device->GetData(MEMORYDEVICE_CUDA), 0, sizeof(int)));
}

void PreemptiveRansac_CUDA::sample_inliers(bool useMask)
{
  const Vector2i imgSize = m_keypointsImage->noDims;
  int *inlierRasterIndices = m_inlierRasterIndicesBlock->GetData(MEMORYDEVICE_CUDA);
  int *inliersMask = m_inliersMaskImage->GetData(MEMORYDEVICE_CUDA);
  const Keypoint3DColour *keypoints = m_keypointsImage->GetData(MEMORYDEVICE_CUDA);
  int *nbInliers_device = m_nbInliers_device->GetData(MEMORYDEVICE_CUDA);
  const ScorePrediction *predictions = m_predictionsImage->GetData(MEMORYDEVICE_CUDA);
  CUDARNG *rngs = m_rngs->GetData(MEMORYDEVICE_CUDA);

  dim3 blockSize(128);
  dim3 gridSize((m_ransacInliersPerIteration + blockSize.x - 1) / blockSize.x);

  if(useMask)
  {
    ck_sample_inliers<true><<<gridSize,blockSize>>>(
      keypoints, predictions, imgSize, rngs, inlierRasterIndices, nbInliers_device, m_ransacInliersPerIteration, inliersMask
    );
    ORcudaKernelCheck;
  }
  else
  {
    ck_sample_inliers<false><<<gridSize,blockSize>>>(
      keypoints, predictions, imgSize, rngs, inlierRasterIndices, nbInliers_device, m_ransacInliersPerIteration
    );
    ORcudaKernelCheck;
  }

  // Update the host's record of the number of inliers.
  m_inlierRasterIndicesBlock->dataSize = static_cast<size_t>(m_nbInliers_device->GetElement(0, MEMORYDEVICE_CUDA));
}

void PreemptiveRansac_CUDA::update_candidate_poses()
{
  // The pose update is currently implemented by the base class, need to copy the relevant data to host memory.
  m_poseCandidates->UpdateHostFromDevice();

  PreemptiveRansac::update_candidate_poses();

  // The copy the updated poses back to the device.
  m_poseCandidates->UpdateDeviceFromHost();
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void PreemptiveRansac_CUDA::init_random()
{
  CUDARNG *rngs = m_rngs->GetData(MEMORYDEVICE_CUDA);

  // Initialize random states
  dim3 blockSize(256);
  dim3 gridSize((m_maxPoseCandidates + blockSize.x - 1) / blockSize.x);

  ck_reinit_rngs<<<gridSize, blockSize>>>(rngs, m_maxPoseCandidates, m_rngSeed);
  ORcudaKernelCheck;
}

void PreemptiveRansac_CUDA::update_host_pose_candidates() const
{
  m_poseCandidates->UpdateHostFromDevice();
}

}
