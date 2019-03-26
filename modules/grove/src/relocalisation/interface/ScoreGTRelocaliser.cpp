/**
 * grove: ScoreGTRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "relocalisation/interface/ScoreGTRelocaliser.h"
using namespace ORUtils;

namespace grove {

//#################### CONSTRUCTORS ####################

ScoreGTRelocaliser::ScoreGTRelocaliser(const tvgutil::SettingsContainer_CPtr& settings, const std::string& settingsNamespace, DeviceType deviceType)
: ScoreRelocaliser(settings, settingsNamespace, deviceType)
{
  // Since we're using ground truth SCoRe predictions, there's no need to train a model, and thus no need for any example reservoirs,
  // so set the number of such reservoirs to zero.
  m_reservoirCount = 0;

  // Set up the example clusterer and the relocaliser's internal state.
  reset();
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

void ScoreGTRelocaliser::make_predictions(const ORUChar4Image *colourImage) const
{
  // If a ground truth pose is available for this frame, create a "ground truth" SCoRe prediction
  // that has only one cluster (containing a single world-space point) for each keypoint.
  if(m_groundTruthTrajectory && m_groundTruthFrameIndex < m_groundTruthTrajectory->size())
  {
    const Matrix4f& cameraToWorld = (*m_groundTruthTrajectory)[m_groundTruthFrameIndex].GetInvM();
    set_ground_truth_predictions_for_keypoints(m_keypointsImage, cameraToWorld, m_predictionsImage);
  }
  else throw std::runtime_error("Error: Ground truth pose not available for frame " + boost::lexical_cast<std::string>(m_groundTruthFrameIndex));
}

void ScoreGTRelocaliser::train_sub(const ORUChar4Image *colourImage, const ORFloatImage *depthImage,
                                   const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose)
{
  // No-op
}

}
