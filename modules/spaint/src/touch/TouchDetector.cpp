/**
 * spaint: TouchDetector.cpp
 */

#include "touch/TouchDetector.h"

#include <iostream>

#include "imageprocessing/cpu/ImageProcessing_CPU.h"
#include "visualisers/cpu/DepthCalculator_CPU.h"
#ifdef WITH_CUDA
#include "imageprocessing/cuda/ImageProcessing_CUDA.h"
#include "visualisers/cuda/DepthCalculator_CUDA.h"
#endif

#ifdef WITH_OPENCV
#include "util/OCVdebugger.h"
#include "util/OpenCVExtra.h"
#endif

namespace spaint {

//#################### CONSTRUCTORS #################### 
TouchDetector::TouchDetector(const Vector2i& imgSize)
{
#ifdef WITH_CUDA
  m_diffRawRaycast.reset(new ITMFloatImage(imgSize, true, true));
  m_raycastedDepthResult.reset(new ITMFloatImage(imgSize, true, true));

  m_depthCalculator.reset(new DepthCalculator_CUDA);
  m_imageProcessor.reset(new ImageProcessing_CUDA);
#else
  m_diffRawRaycast.reset(new ITMFloatImage(imgSize, true, false));
  m_raycastedDepthResult.reset(new ITMFloatImage(imgSize, true, false));

  m_depthCalculator.reset(new DepthCalculator_CPU);
  m_imageProcessor.reset(new ImageProcessing_CPU);
#endif
}

//#################### PUBLIC MEMBER FUNCTIONS #################### 
void TouchDetector::run_touch_detector_on_frame(const RenderState_Ptr& renderState, const rigging::SimpleCamera_Ptr camera, float voxelSize, ITMFloatImage *rawDepth) const
{
  // Calculate the depth raycast from the current scene, this is in meters.
  m_depthCalculator->render_depth(m_raycastedDepthResult.get(), renderState.get(), camera.get(), voxelSize, DepthCalculator::DT_ORTHOGRAPHIC);

  // Calculate the difference between the raw depth and the raycasted depth.
  //rawDepth->UpdateDeviceFromHost();
  m_imageProcessor->absolute_difference_calculator(m_diffRawRaycast.get(), rawDepth, m_raycastedDepthResult.get());

#if defined(WITH_OPENCV)
//#if defined(WITH_OPENCV) && !defined(WITH_CUDA)
  OCVdebugger::display_image_scale_to_range(rawDepth, "Current raw depth from camera in millimeters");
  OCVdebugger::display_image_scale_to_range(m_raycastedDepthResult.get(), "Current depth raycast in millimeters");
  OCVdebugger::display_image_and_scale(m_diffRawRaycast.get(), 1000.0f, "Difference between raw and raycasted depth");
#endif

#ifdef WITH_OPENCV
  // Run the OpenCV CPU only pipeline.
  //m_diffRawRaycast->UpdateHostFromDevice();
  //opencv_cpu_pipeline(m_diffRawRaycast);
#else
#endif
}

const TouchState& TouchDetector::get_touch_state() const
{
  return m_touchState;
}

//#################### PRIVATE MEMBER FUNCTIONS #################### 

#ifdef WITH_OPENCV
void TouchDetector::opencv_cpu_pipeline(const FloatImage_Ptr& rawDiff) const
{
  static cv::Mat tmpMat32F(rawDiff->noDims.y, rawDiff->noDims.x, CV_32F);
  OpenCVExtra::ITM2MAT(rawDiff.get(), &tmpMat32F);
  OpenCVExtra::imshow_float_and_scale("cvDiffRawRaycast", tmpMat32F, 1000.0f);

  static cv::Mat tmpMat8U;

  cv::threshold(tmpMat32F, tmpMat8U, 0.01f, 255.0f, cv::THRESH_BINARY);
  cv::erode(tmpMat8U, tmpMat8U, cv::Mat(), cv::Point(-1,-1), 3);
  cv::dilate(tmpMat8U, tmpMat8U, cv::Mat(), cv::Point(-1,-1), 3);
  
  cv::imshow("Thresholded above 1cm", tmpMat8U);
}
#endif

}

