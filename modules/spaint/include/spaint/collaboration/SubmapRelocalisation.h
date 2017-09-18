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
  ITMFloatImage_Ptr m_depthJ;

  /** TODO */
  Vector4f m_depthIntrinsicsJ;

  /** TODO */
  int m_frameIndex;

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

  SubmapRelocalisation(const std::string& sceneI, const std::string& sceneJ, int frameIndex, const View_CPtr& viewJ, const ORUtils::SE3Pose& localPoseJ)
  : m_depthIntrinsicsJ(viewJ->calib.intrinsics_d.projectionParamsSimple.all),
    m_frameIndex(frameIndex),
    m_localPoseJ(localPoseJ),
    m_sceneI(sceneI),
    m_sceneJ(sceneJ)
  {
    viewJ->depth->UpdateHostFromDevice();
    viewJ->rgb->UpdateHostFromDevice();

    m_depthJ.reset(new ITMFloatImage(viewJ->depth->noDims, true, false));
    m_rgbJ.reset(new ITMUChar4Image(viewJ->rgb->noDims, true, false));

    m_depthJ->SetFrom(viewJ->depth, ITMFloatImage::CPU_TO_CPU);
    m_rgbJ->SetFrom(viewJ->rgb, ITMUChar4Image::CPU_TO_CPU);
  }
};

}

#endif
