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

#include "../features/FeatureCalculatorFactory.h"
#include "../randomforest/interface/SCoReForest.h"
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
  RGBDPatchFeatureCalculator_CPtr m_featureExtractor;
  RGBDPatchFeatureImage_Ptr m_featureImage;
  SCoRePredictionsImage_Ptr m_predictionsImage;
  SCoReForest_Ptr m_scoreForest;
  PreemptiveRansac_Ptr m_preemptiveRansac;
  bool m_updateForestModesEveryFrame;

  Tracker_Ptr m_refineTracker;
  boost::optional<tvgutil::SequentialPathGenerator> m_sequentialPathGenerator;

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
