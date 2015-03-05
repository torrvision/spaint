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
#include "opencv2/core/ocl.hpp"
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
  m_depthCalculator->render_orthographic_distance(m_raycastedDepthResult.get(), renderState.get(), camera.get(), voxelSize);

  // Calculate the difference between the raw depth and the raycasted depth.
  m_imageProcessor->absolute_difference_calculator(m_diffRawRaycast.get(), rawDepth, m_raycastedDepthResult.get());

  static cv::Mat cvDiffRawRaycast(rawDepth->noDims.y, rawDepth->noDims.x, CV_32F);
  OpenCVExtra::ITM2MAT(m_diffRawRaycast.get(), &cvDiffRawRaycast);
  OpenCVExtra::imshow_float_and_scale("cvDiffRawRaycast", cvDiffRawRaycast, 1000.0f);

  static cv::UMat tmpUMat;
  cvDiffRawRaycast.copyTo(tmpUMat);
  
  //std::cout << (cv::ocl::useOpenCL() ? "OpenCL enabled" : "CPU") << "mode\n";

  cv::threshold(tmpUMat, tmpUMat, 0.01f, 5.0f, cv::THRESH_BINARY);
  cv::erode(tmpUMat, tmpUMat, cv::Mat(), cv::Point(-1,-1), 3);
  cv::dilate(tmpUMat, tmpUMat, cv::Mat(), cv::Point(-1,-1), 3);
  
  OpenCVExtra::imshow_float_scale_to_range("Thresholded above 1cm", tmpUMat.getMat(cv::ACCESS_RW));

#ifndef WITH_CUDA
#ifdef WITH_OPENCV
  OCVdebugger::display_image_scale_to_range(rawDepth, "Current raw depth from camera in millimeters");
  OCVdebugger::display_image_scale_to_range(m_raycastedDepthResult.get(), "Current depth raycast in millimeters");
  OCVdebugger::display_image_and_scale(m_diffRawRaycast.get(), 1000.0f, "Difference between raw and raycasted depth");
#endif
#endif
}

const TouchState& TouchDetector::get_touch_state() const
{
  return m_touchState;
}

//#################### PRIVATE MEMBER FUNCTIONS #################### 
}

