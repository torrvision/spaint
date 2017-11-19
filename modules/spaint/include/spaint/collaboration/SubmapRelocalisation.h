/**
 * spaint: SubmapRelocalisation.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_SPAINT_SUBMAPRELOCALISATION
#define H_SPAINT_SUBMAPRELOCALISATION

#include <boost/optional.hpp>

#ifdef WITH_OPENCV
#include <opencv2/core/core.hpp>
#endif

#include <itmx/base/ITMImagePtrTypes.h>
#include <itmx/base/ITMObjectPtrTypes.h>
#include <itmx/relocalisation/Relocaliser.h>

namespace spaint {

/**
 * \brief TODO
 */
struct SubmapRelocalisation
{
  //#################### PUBLIC VARIABLES ####################

  /** TODO */
  float m_candidateScore;

  /** TODO */
  Vector4f m_depthIntrinsicsJ;

  /** TODO */
  int m_frameIndexJ;

  /** TODO */
  itmx::Relocaliser::Quality m_initialRelocalisationQuality;

  /** TODO */
  ORUtils::SE3Pose m_localPoseJ;

#ifdef WITH_OPENCV
  /** TODO */
  cv::Scalar m_meanDepthDiff;
#endif

  /** TODO */
  boost::optional<ORUtils::SE3Pose> m_relativePose;

  /** TODO */
  std::string m_sceneI;

  /** TODO */
  std::string m_sceneJ;

  /** TODO */
  float m_targetValidFraction;

  //#################### CONSTRUCTORS ####################

  SubmapRelocalisation(const std::string& sceneI, const std::string& sceneJ, int frameIndexJ, const Vector4f& depthIntrinsicsJ, const ORUtils::SE3Pose& localPoseJ)
  : m_candidateScore(0.0f),
    m_depthIntrinsicsJ(depthIntrinsicsJ),
    m_frameIndexJ(frameIndexJ),
    m_initialRelocalisationQuality(itmx::Relocaliser::RELOCALISATION_POOR),
    m_localPoseJ(localPoseJ),
    m_sceneI(sceneI),
    m_sceneJ(sceneJ),
    m_targetValidFraction(0.0f)
  {}
};

}

#endif
