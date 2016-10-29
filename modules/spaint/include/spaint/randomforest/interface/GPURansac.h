/**
 * spaint: GPURansac.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_GPURANSAC
#define H_SPAINT_GPURANSAC

#include <boost/optional.hpp>

#include "ITMLib/Utils/ITMMath.h"
#include "../../features/interface/RGBDPatchFeature.h"
#include "GPUForestTypes.h"

namespace spaint
{

struct PoseCandidate
{
  static const int MAX_INLIERS = 3003; // 3 for Kabsch and 500 * RANSAC iteration (6 iterations)

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

typedef ORUtils::MemoryBlock<PoseCandidate> PoseCandidateMemoryBlock;
typedef boost::shared_ptr<PoseCandidateMemoryBlock> PoseCandidateMemoryBlock_Ptr;
typedef boost::shared_ptr<const PoseCandidateMemoryBlock> PoseCandidateMemoryBlock_CPtr;

class GPURansac
{
public:
  GPURansac();
  virtual ~GPURansac();

  int get_min_nb_required_points() const;
  boost::optional<PoseCandidate> estimate_pose(
      const RGBDPatchFeatureImage_CPtr &features,
      const GPUForestPredictionsImage_CPtr &forestPredictions);

protected:
  // Member variables from scoreforests
  size_t m_kInitRansac;
  size_t m_nbPointsForKabschBoostrap;
  bool m_useAllModesPerLeafInPoseHypothesisGeneration;
  bool m_checkMinDistanceBetweenSampledModes;
  float m_minDistanceBetweenSampledModes;
  bool m_checkRigidTransformationConstraint;
  float m_translationErrorMaxForCorrectPose;
  size_t m_batchSizeRansac;
  size_t m_trimKinitAfterFirstEnergyComputation;
  bool m_poseUpdate;
  bool m_usePredictionCovarianceForPoseOptimization;
  float m_poseOptimizationInlierThreshold;

  RGBDPatchFeatureImage_CPtr m_featureImage;
  GPUForestPredictionsImage_CPtr m_predictionsImage;

  PoseCandidateMemoryBlock_Ptr m_poseCandidates;
  size_t m_nbPoseCandidates;

  void generate_pose_candidates();
  bool hypothesize_pose(PoseCandidate &res, std::mt19937 &eng);
  void sample_pixels_for_ransac(std::vector<bool> &maskSampledPixels,
      std::vector<Vector2i> &sampledPixelIdx, std::mt19937 &eng, int batchSize);
  void update_inliers_for_optimization(
      const std::vector<Vector2i> &sampledPixelIdx);
  void compute_and_sort_energies();
  void compute_pose_energy(PoseCandidate &candidate) const;
  void update_candidate_poses();
  bool update_candidate_pose(PoseCandidate &poseCandidate) const;

private:
};

typedef boost::shared_ptr<GPURansac> GPURansac_Ptr;
typedef boost::shared_ptr<const GPURansac> GPURansac_CPtr;
}
#endif
