/**
 * spaint: SubmapRelocalisation.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_SPAINT_SUBMAPRELOCALISATION
#define H_SPAINT_SUBMAPRELOCALISATION

#include <boost/optional.hpp>

#include <itmx/base/ITMImagePtrTypes.h>
#include <itmx/base/ITMObjectPtrTypes.h>

namespace spaint {

/**
 * \brief TODO
 */
struct SubmapRelocalisation
{
  //#################### PUBLIC VARIABLES ####################

  /** TODO */
  Vector4f m_depthIntrinsicsJ;

  /** TODO */
  ITMFloatImage_Ptr m_depthJ;

  /** TODO */
  int m_frameIndexJ;

  /** TODO */
  ORUtils::SE3Pose m_localPoseJ;

  /** TODO */
  boost::optional<ORUtils::SE3Pose> m_relativePose;

  /** TODO */
  ITMUChar4Image_Ptr m_rgbJ;

  /** TODO */
  std::string m_sceneI;

  /** TODO */
  std::string m_sceneJ;

  //#################### CONSTRUCTORS ####################

  SubmapRelocalisation(const std::string& sceneI, const std::string& sceneJ, int frameIndexJ, const Vector4f& depthIntrinsicsJ, const ORUtils::SE3Pose& localPoseJ,
                       const ITMFloatImage_Ptr& depthJ = ITMFloatImage_Ptr(), const ITMUChar4Image_Ptr& rgbJ = ITMUChar4Image_Ptr())
  : m_depthJ(depthJ),
    m_depthIntrinsicsJ(depthIntrinsicsJ),
    m_frameIndexJ(frameIndexJ),
    m_localPoseJ(localPoseJ),
    m_rgbJ(rgbJ),
    m_sceneI(sceneI),
    m_sceneJ(sceneJ)
  {}
};

}

#endif
