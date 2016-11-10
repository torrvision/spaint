/**
 * spaint: PreemptiveRansac.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_PREEMPTIVERANSAC
#define H_SPAINT_PREEMPTIVERANSAC

#include <boost/optional.hpp>

#include "ORUtils/PlatformIndependence.h"

#include "ITMLib/Utils/ITMMath.h"
#include "../../features/interface/RGBDPatchFeature.h"
#include "GPUForestTypes.h"
#include <Eigen/Dense>

namespace spaint
{

struct PoseCandidate
{
  enum
  {
    MAX_INLIERS = 3003 // 3 for Kabsch and 500 * RANSAC iteration (6 iterations, when running with 64 candidates)
//    MAX_INLIERS = 5503 // 3 for Kabsch and 500 * RANSAC iteration (10 iterations, when running with 1024 candidates + 500 initial candidates)
  };

  struct Inlier
  {
    int linearIdx;
    int modeIdx;
    float energy;
  };

  Matrix4f cameraPose;
  Inlier inliers[MAX_INLIERS];
  int nbInliers;
  float energy;
  int cameraId;
};

_CPU_AND_GPU_CODE_ inline bool operator <(const PoseCandidate &a,
    const PoseCandidate &b)
{
  return a.energy < b.energy;
}

typedef ORUtils::MemoryBlock<PoseCandidate> PoseCandidateMemoryBlock;
typedef boost::shared_ptr<PoseCandidateMemoryBlock> PoseCandidateMemoryBlock_Ptr;
typedef boost::shared_ptr<const PoseCandidateMemoryBlock> PoseCandidateMemoryBlock_CPtr;

struct PoseCandidates
{
  enum
  {
    MAX_CANDIDATES = 1024
  };

  PoseCandidate candidates[MAX_CANDIDATES];
  int nbCandidates;
};

typedef ORUtils::MemoryBlock<PoseCandidates> PoseCandidatesMemoryBlock;
typedef boost::shared_ptr<PoseCandidatesMemoryBlock> PoseCandidatesMemoryBlock_Ptr;
typedef boost::shared_ptr<const PoseCandidatesMemoryBlock> PoseCandidatesMemoryBlock_CPtr;

class PreemptiveRansac
{
public:
  PreemptiveRansac();
  virtual ~PreemptiveRansac();

  int get_min_nb_required_points() const;
  boost::optional<PoseCandidate> estimate_pose(
      const RGBDPatchFeatureImage_CPtr &features,
      const GPUForestPredictionsImage_CPtr &forestPredictions);

protected:
  // Member variables from scoreforests
  size_t m_nbPointsForKabschBoostrap;
  bool m_useAllModesPerLeafInPoseHypothesisGeneration;
  bool m_checkMinDistanceBetweenSampledModes;
  float m_minDistanceBetweenSampledModes;
  bool m_checkRigidTransformationConstraint;
  float m_translationErrorMaxForCorrectPose;
  size_t m_batchSizeRansac;
  int m_trimKinitAfterFirstEnergyComputation;
  bool m_poseUpdate;
  bool m_usePredictionCovarianceForPoseOptimization;
  float m_poseOptimizationInlierThreshold;

  RGBDPatchFeatureImage_CPtr m_featureImage;
  GPUForestPredictionsImage_CPtr m_predictionsImage;

  PoseCandidateMemoryBlock_Ptr m_poseCandidates;
  int m_nbPoseCandidates;

  virtual void generate_pose_candidates();
  void sample_pixels_for_ransac(std::vector<bool> &maskSampledPixels,
      std::vector<Vector2i> &sampledPixelIdx, std::mt19937 &eng, int batchSize);
  void update_inliers_for_optimization(
      const std::vector<Vector2i> &sampledPixelIdx);
  virtual void compute_and_sort_energies();
  void update_candidate_poses();
  bool update_candidate_pose(PoseCandidate &poseCandidate) const;
  Eigen::Matrix4f Kabsch(Eigen::MatrixXf &P, Eigen::MatrixXf &Q) const;

private:
  bool hypothesize_pose(PoseCandidate &res, std::mt19937 &eng);
  void compute_pose_energy(PoseCandidate &candidate) const;
};

typedef boost::shared_ptr<PreemptiveRansac> PreemptiveRansac_Ptr;
typedef boost::shared_ptr<const PreemptiveRansac> PreemptiveRansac_CPtr;
}
#endif
