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
    KABSCH_POINTS = 3
  };

  Matrix4f cameraPose;
  Vector3f cameraPoints[KABSCH_POINTS];
  Vector3f worldPoints[KABSCH_POINTS];
  float energy;
};

_CPU_AND_GPU_CODE_ inline bool operator <(const PoseCandidate &a,
    const PoseCandidate &b)
{
  return a.energy < b.energy;
}

typedef ORUtils::MemoryBlock<PoseCandidate> PoseCandidateMemoryBlock;
typedef boost::shared_ptr<PoseCandidateMemoryBlock> PoseCandidateMemoryBlock_Ptr;
typedef boost::shared_ptr<const PoseCandidateMemoryBlock> PoseCandidateMemoryBlock_CPtr;

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
  float m_minSquaredDistanceBetweenSampledModes;
  bool m_checkRigidTransformationConstraint;
  float m_translationErrorMaxForCorrectPose;
  size_t m_batchSizeRansac;
  int m_trimKinitAfterFirstEnergyComputation;
  bool m_poseUpdate;
  bool m_usePredictionCovarianceForPoseOptimization;
  float m_poseOptimizationInlierThreshold;

  RGBDPatchFeatureImage_CPtr m_featureImage;
  GPUForestPredictionsImage_CPtr m_predictionsImage;

  size_t m_nbMaxPoseCandidates;
  PoseCandidateMemoryBlock_Ptr m_poseCandidates;

  size_t m_nbMaxInliers;
  ITMIntImage_Ptr m_inliersMaskImage;
  ITMIntImage_Ptr m_inliersIndicesImage;

  virtual void generate_pose_candidates() = 0;
  virtual void sample_inlier_candidates(bool useMask = false) = 0;
  virtual void compute_and_sort_energies() = 0;
  virtual void update_candidate_poses() = 0;

  void compute_candidate_poses_kabsch();
  bool update_candidate_pose(PoseCandidate &poseCandidate) const;
};

typedef boost::shared_ptr<PreemptiveRansac> PreemptiveRansac_Ptr;
typedef boost::shared_ptr<const PreemptiveRansac> PreemptiveRansac_CPtr;
}
#endif
