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
#include "util/OpenCVExtra.h"
#endif

#include "util/ArrayFireExtra.h"

namespace spaint {

//#################### TEMPORARY FUNCTIONS #################### 
Vector3f convert_eigenv3f_to_itmv3f(const Eigen::Vector3f& v)
{
  Vector3f itmv;
  itmv.x = v[0];
  itmv.y = v[1];
  itmv.z = v[2];
  return itmv;
}

//#################### CONSTRUCTORS #################### 
TouchDetector::TouchDetector(const Vector2i& imgSize)
: m_depthThreshold(0.010f), 
  m_thresholded(imgSize.y, imgSize.x), 
  m_workspace(new af::array(imgSize.y, imgSize.x))
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

#ifdef DEBUG_TOUCH
  m_depthThresholdmm = m_depthThreshold * 1000.0f;
#endif
}

//#################### PUBLIC MEMBER FUNCTIONS #################### 
void TouchDetector::run_touch_detector_on_frame(const RenderState_Ptr& renderState, const rigging::SimpleCamera_Ptr camera, float voxelSize, ITMFloatImage *rawDepth)
{
  // Calculate the depth raycast from the current scene, this is in meters.
  m_depthCalculator->render_depth(m_raycastedDepthResult.get(), renderState.get(), convert_eigenv3f_to_itmv3f(camera->p()), convert_eigenv3f_to_itmv3f(camera->n()), voxelSize, DepthCalculator::DT_ORTHOGRAPHIC);
#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH)
  OpenCVExtra::display_image_scale_to_range(rawDepth, "Current raw depth from camera in millimeters");
  OpenCVExtra::display_image_scale_to_range(m_raycastedDepthResult.get(), "Current depth raycast in millimeters");
#endif

  // Calculate the difference between the raw depth and the raycasted depth.
  //m_imageProcessor->absolute_difference_calculator(m_diffRawRaycast.get(), rawDepth, m_raycastedDepthResult.get());
  m_imageProcessor->absolute_difference_calculator(m_workspace.get(), rawDepth, m_raycastedDepthResult.get());
  
#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH)
  //OpenCVExtra::display_image_and_scale(m_diffRawRaycast.get(), 1000.0f, "Difference between raw and raycasted depth");
  static int rows = m_workspace->dims(0);
  static int cols = m_workspace->dims(1);
  static af::array tmp;
  tmp = *m_workspace * 1000.0f;
  OpenCVExtra::ocvfig("Diff image in arrayfire", tmp.as(u8).host<unsigned char>(), cols, rows, OpenCVExtra::Order::COL_MAJOR);
#endif

  // Threshold the difference image.
  m_thresholded = *m_workspace > m_depthThreshold;

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH)
  // Initialise the OpenCV Trackbar if it's the first iteration.
  static bool initialised = false;
  if(!initialised)
  {
    cv::namedWindow("DebuggingOutputWindow", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("depthThresholdmm", "DebuggingOutputWindow", &m_depthThresholdmm, 50);
    initialised = true;
  }
  // Update the current depth Threshold in meters;
  m_depthThreshold = m_depthThresholdmm / 1000.0f;

  static af::array thresholdedDisplay;
  thresholdedDisplay = m_thresholded * 255.0f;
  OpenCVExtra::ocvfig("DebuggingOutputWindow", thresholdedDisplay.as(u8).host<unsigned char>(), cols, rows, OpenCVExtra::Order::COL_MAJOR);
#endif


#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH)
  cv::waitKey(10);
#endif

#ifdef DEBUG_TOUCH
#undef DEBUG_TOUCH
#endif
}

const TouchState& TouchDetector::get_touch_state() const
{
  return m_touchState;
}

//#################### PRIVATE MEMBER FUNCTIONS #################### 

/*#ifdef WITH_OPENCV
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
  cv::waitKey(10);
}
#endif*/

}

