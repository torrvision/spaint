/**
 * grove: ScoreRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCORERELOCALISER
#define H_GROVE_SCORERELOCALISER

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

#include <ITMLib/Utils/ITMLibSettings.h>
#include <ITMLib/Utils/ITMImageTypes.h>
#include <ORUtils/Image.h>
#include <ORUtils/SE3Pose.h>

#include "../../clustering/base/Prediction3DColour.h"
#include "../../clustering/interface/ExampleClusterer.h"
#include "../../features/interface/RGBDPatchFeatureCalculator.h"
#include "../../forests/interface/DecisionForest.h"
#include "../../ransac/interface/PreemptiveRansac.h"
#include "../../reservoirs/interface/ExampleReservoirs.h"

namespace grove {

class ScoreRelocaliser
{
public:
  typedef Prediction3DColour ClusterType;
  typedef Keypoint3DColour ExampleType;
  typedef RGBDPatchDescriptor DescriptorType;
  enum { TREE_COUNT = 5 };

  typedef ORUtils::VectorX<int, TREE_COUNT> LeafIndices;
  typedef ORUtils::Image<LeafIndices> LeafIndicesImage;
  typedef boost::shared_ptr<LeafIndicesImage> LeafIndicesImage_Ptr;

  typedef ExampleClusterer<ExampleType, ClusterType> Clusterer;
  typedef boost::shared_ptr<Clusterer> Clusterer_Ptr;

  typedef DecisionForest<DescriptorType, TREE_COUNT> ScoreForest;
  typedef boost::shared_ptr<ScoreForest> ScoreForest_Ptr;

  typedef ExampleReservoirs<ExampleType> Reservoirs;
  typedef boost::shared_ptr<Reservoirs> Reservoirs_Ptr;

  explicit ScoreRelocaliser(ITMLib::ITMLibSettings::DeviceType deviceType, const std::string &forestFilename);

  void reset();
  void integrate_measurements(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f &depthIntrinsics, const ORUtils::SE3Pose &cameraPose);
  void idle_update();
  boost::optional<ORUtils::SE3Pose> estimate_pose(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage) const;

private:
  DA_RGBDPatchFeatureCalculator_Ptr m_featureCalculator;
  ScoreForest_Ptr m_scoreForest;
  Reservoirs_Ptr m_exampleReservoirs;
  Clusterer_Ptr m_exampleClusterer;
  PreemptiveRansac_Ptr m_preemptiveRansac;

  ScorePredictionsBlock_Ptr m_predictionsBlock;

  // Update-related data
  uint32_t m_reservoirsCount;
  uint32_t m_maxReservoirsToUpdate;
  uint32_t m_lastFeaturesAddedStartIdx;
  uint32_t m_reservoirUpdateStartIdx;

  // Per-frame data
  Keypoint3DColourImage_Ptr m_rgbdPatchKeypointsImage;
  RGBDPatchDescriptorImage_Ptr m_rgbdPatchDescriptorImage;
  ScorePredictionsImage_Ptr m_predictionsImage;
  LeafIndicesImage_Ptr m_leafIndicesImage;

private:
  void compute_features(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f &depthIntrinsics, const Matrix4f &invCameraPose) const;
  uint32_t compute_nb_reservoirs_to_update() const;
  void update_reservoir_start_idx();
};

}

#endif
