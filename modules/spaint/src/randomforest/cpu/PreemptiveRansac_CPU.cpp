/**
 * spaint: PreemptiveRansac_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/cpu/PreemptiveRansac_CPU.h"
#include "randomforest/shared/PreemptiveRansac_Shared.h"

#include "util/MemoryBlockFactory.h"

#include <Eigen/Dense>

using namespace tvgutil;

namespace spaint
{
PreemptiveRansac_CPU::PreemptiveRansac_CPU() :
    PreemptiveRansac()
{
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();
  m_randomGenerators = mbf.make_block<CPURNG>(PoseCandidates::MAX_CANDIDATES);
  m_rngSeed = 42;

  init_random();
}

void PreemptiveRansac_CPU::init_random()
{
  CPURNG *randomGenerators = m_randomGenerators->GetData(MEMORYDEVICE_CPU);

  // Initialize random states
  for (int i = 0; i < PoseCandidates::MAX_CANDIDATES; ++i)
  {
    randomGenerators[i].reset(m_rngSeed + i);
  }
}

void PreemptiveRansac_CPU::generate_pose_candidates()
{
  const Vector2i imgSize = m_featureImage->noDims;
  const RGBDPatchFeature *features = m_featureImage->GetData(MEMORYDEVICE_CPU);
  const GPUForestPrediction *predictions = m_predictionsImage->GetData(
      MEMORYDEVICE_CPU);

  CPURNG *randomGenerators = m_randomGenerators->GetData(MEMORYDEVICE_CPU);
  PoseCandidate *poseCandidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);

  m_nbPoseCandidates = 0;

#ifdef WITH_OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
  for (int candidateIdx = 0; candidateIdx < PoseCandidates::MAX_CANDIDATES;
      ++candidateIdx)
  {
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
      int candidateIdx;

#ifdef WITH_OPENMP
#pragma omp atomic capture
#endif
      candidateIdx = m_nbPoseCandidates++;
      poseCandidates[candidateIdx] = candidate;
    }
  }

  // TODO: think about this

  // Now perform kabsch on the candidates
  //#ifdef ENABLE_TIMERS
  //    boost::timer::auto_cpu_timer t(6,
  //        "kabsch: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
  //#endif
  compute_candidate_pose_kabsch();
}

void PreemptiveRansac_CPU::compute_and_sort_energies()
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

void PreemptiveRansac_CPU::compute_candidate_pose_kabsch()
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
