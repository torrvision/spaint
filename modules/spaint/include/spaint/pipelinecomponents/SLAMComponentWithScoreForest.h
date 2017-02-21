/**
 * spaint: SLAMComponentWithScoreForest.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SLAMCOMPONENTWITHSCOREFOREST
#define H_SPAINT_SLAMCOMPONENTWITHSCOREFOREST

#include "SLAMComponent.h"

#include <tuple>
#include <random>

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/timer/timer.hpp>

#include <grove/features/FeatureCalculatorFactory.h>
using namespace grove;

#include "../randomforest/interface/ScoreForest.h"
#include "../randomforest/interface/PreemptiveRansac.h"

#include "tvgutil/filesystem/SequentialPathGenerator.h"

namespace spaint
{

/**
 * \brief An instance of this pipeline component can be used to perform simultaneous localisation and mapping (SLAM).
 */
class SLAMComponentWithScoreForest: public SLAMComponent
{
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
  boost::optional<PoseCandidate> estimate_pose();
  void compute_features(const ITMUChar4Image *inputRgbImage,
      const ITMFloatImage *inputDepthImage, const Vector4f &depthIntrinsics,
      const Matrix4f &invCameraPose);
  void compute_features(const ITMUChar4Image *inputRgbImage,
      const ITMFloatImage *inputDepthImage, const Vector4f &depthIntrinsics);
  void evaluate_forest();

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  DA_RGBDPatchFeatureCalculator_CPtr m_featureExtractor;
  Keypoint3DColourImage_Ptr m_rgbdPatchKeypointsImage;
  RGBDPatchDescriptorImage_Ptr m_rgbdPatchDescriptorImage;
  ScorePredictionsImage_Ptr m_predictionsImage;
  ScoreForest_Ptr m_relocalisationForest;
  std::string m_relocalisationForestPath;
  PreemptiveRansac_Ptr m_preemptiveRansac;
  bool m_updateForestModesEveryFrame;

  boost::optional<tvgutil::SequentialPathGenerator> m_sequentialPathGenerator;
  Tracker_Ptr m_refinementTracker;
  std::string m_refinementTrackerParams;
  TrackingController_Ptr m_refinementTrackingController;

  // For evaluation
  bool m_relocaliseAfterEveryFrame;
  bool m_saveRelocalisationPoses;

  // For profiling
  bool m_timeRelocalizer;
  size_t m_learningCalls;
  boost::timer::cpu_times m_learningTimes;
  size_t m_relocalizationCalls;
  boost::timer::cpu_times m_relocalizationTimes;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SLAMComponentWithScoreForest> SLAMComponentWithScoreForest_Ptr;

}

#endif
