/**
 * spaint: SLAMComponentWithScoreForest.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SLAMCOMPONENTWITHSCOREFOREST
#define H_SPAINT_SLAMCOMPONENTWITHSCOREFOREST

#include "SLAMComponent.h"

#include <tuple>
#include <random>

#include <boost/shared_ptr.hpp>

#include <DatasetRGBD7Scenes.hpp>
#include <DFBP.hpp>

#include "../features/FeatureCalculatorFactory.h"
#include "../randomforest/interface/GPUForest.h"

namespace spaint
{

/**
 * \brief An instance of this pipeline component can be used to perform simultaneous localisation and mapping (SLAM).
 */
class SLAMComponentWithScoreForest: public SLAMComponent
{
  typedef std::tuple<Eigen::Matrix4f, std::vector<std::pair<int, int>>, float,
      int> PoseCandidate;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a SLAM component performing relocalization using a Score Forest.
   *
   * \param context           The shared context needed for SLAM.
   * \param sceneID           The ID of the scene to reconstruct.
   * \param imageSourceEngine The engine used to provide input images to the fusion process.
   * \param trackerType       The type of tracker to use.
   * \param trackerParams     The parameters for the tracker (if any).
   * \param mappingMode       The mapping mode to use.
   * \param trackingMode      The tracking mode to use.
   */
  SLAMComponentWithScoreForest(const SLAMContext_Ptr& context,
      const std::string& sceneID,
      const ImageSourceEngine_Ptr& imageSourceEngine, TrackerType trackerType,
      const std::vector<std::string>& trackerParams, MappingMode mappingMode =
          MAP_VOXELS_ONLY, TrackingMode trackingMode = TRACK_VOXELS);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys a SLAM component.
   */
  virtual ~SLAMComponentWithScoreForest();

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  virtual TrackingResult process_relocalisation(TrackingResult trackingResult);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  cv::Mat build_rgbd_image(const ITMUChar4Image_Ptr &rgb,
      const ITMShortImage_Ptr &depth) const;
  void generate_pose_candidates(std::vector<PoseCandidate> &poseCandidates);
  bool hypothesize_pose(PoseCandidate &res, std::mt19937 &eng);
  PoseCandidate estimate_pose(std::vector<PoseCandidate> &candidates);
  void sample_pixels_for_ransac(std::vector<bool> &maskSampledPixels,
      std::vector<std::pair<int, int>> &sampledPixelIdx, std::mt19937 &eng,
      int batchSize);
  void update_inliers_for_optimization(
      const std::vector<std::pair<int, int>> &sampledPixelIdx,
      std::vector<PoseCandidate> &poseCandidates) const;
  void compute_and_sort_energies(
      std::vector<PoseCandidate> &poseCandidates) const;
  float compute_pose_energy(const Eigen::Matrix4f &candidateCameraPose,
      const std::vector<std::pair<int, int>> &inliersIndices) const;

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  boost::shared_ptr<DatasetRGBD7Scenes> m_dataset;
  boost::shared_ptr<DFBP> m_forest;
  RGBDPatchFeatureCalculator_CPtr m_featureExtractor;
  RGBDPatchFeatureImage_Ptr m_featureImage;
  ITMIntImage_Ptr m_leafImage;
  GPUForest_Ptr m_gpuForest;
  std::vector<boost::shared_ptr<EnsemblePredictionGaussianMean>> m_featurePredictions;

  // Member variables from scoreforests
  size_t m_kInitRansac;
  int m_nbPointsForKabschBoostrap;
  bool m_useAllModesPerLeafInPoseHypothesisGeneration;
  bool m_checkMinDistanceBetweenSampledModes;
  float m_minDistanceBetweenSampledModes;
  bool m_checkRigidTransformationConstraint;
  float m_translationErrorMaxForCorrectPose;
  int m_batchSizeRansac;
  int m_trimKinitAfterFirstEnergyComputation;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SLAMComponentWithScoreForest> SLAMComponentWithScoreForest_Ptr;

}

#endif
