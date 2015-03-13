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
: m_areaPercentageThreshold(1),
  m_connectedComponents(imgSize.y, imgSize.x),
  m_depthThreshold(0.010f), 
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

  m_areaThreshold = (m_areaPercentageThreshold / 100.0f) * (imgSize.y * imgSize.x);

#ifdef DEBUG_TOUCH
  m_debugDelayms = 10;
  m_depthThresholdmm = m_depthThreshold * 1000;
  m_morphKernelSize = 5;
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
  static bool initialised = false;

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
  if(!initialised)
  {
    cv::namedWindow("DebuggingOutputWindow", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("depthThresholdmm", "DebuggingOutputWindow", &m_depthThresholdmm, 50);
    cv::createTrackbar("debugDelaymx", "DebuggingOutputWindow", &m_debugDelayms, 3000);
    cv::createTrackbar("areaThreshold", "DebuggingOutputWindow", &m_areaThreshold, rows*cols);
  }
  // Update the current depth Threshold in meters;
  m_depthThreshold = m_depthThresholdmm / 1000.0f;

  static af::array thresholdedDisplay;
  thresholdedDisplay = m_thresholded * 255.0f;
  OpenCVExtra::ocvfig("DebuggingOutputWindow", thresholdedDisplay.as(u8).host<unsigned char>(), cols, rows, OpenCVExtra::Order::COL_MAJOR);
  cv::waitKey(5);
#endif

  // Perform morphological operations on the image to get rid of small segments.
#ifdef DEBUG_TOUCH
  m_morphKernelSize = (m_morphKernelSize < 3) ? 3 : m_morphKernelSize;

  // Keep the morphological kernel size odd;
  if((m_morphKernelSize % 2) == 0)
  {
    ++m_morphKernelSize;
  }
  af::array mask8 = af::constant(1, m_morphKernelSize, m_morphKernelSize);
#else
  static af::array mask8 = af::constant(1, m_morphKernelSize, m_morphKernelSize);
#endif

  m_thresholded = af::erode(m_thresholded, mask8);
  m_thresholded = af::dilate(m_thresholded, mask8);

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH)
  if(!initialised)
  {
    cv::namedWindow("MorphologicalOperatorWindow", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("KernelSize", "MorphologicalOperatorWindow", &m_morphKernelSize, 15);
  }

  static af::array morphDisplay;
  morphDisplay = m_thresholded * 255.0f;
  OpenCVExtra::ocvfig("MorphologicalOperatorWindow", morphDisplay.as(u8).host<unsigned char>(), cols, rows, OpenCVExtra::Order::COL_MAJOR);
  cv::waitKey(5);
#endif

  m_connectedComponents = af::regions(m_thresholded); 
  static af::array histogram;
  histogram = af::histogram(m_connectedComponents, af::max<int>(m_connectedComponents));
  // Set the first element to zero as this corresponds to the background.
  histogram(0) = 0;

#ifdef DEBUG_TOUCH
  af::print("Histogram of connected component image", histogram);
#endif

  af::array goodCandidates = af::where(histogram > m_areaThreshold);

#ifdef DEBUG_TOUCH
  if(goodCandidates.elements() > 0)
  {
    af::print("goodCandidates", goodCandidates);
  }
#endif

  if(goodCandidates.elements() > 0)
  {
    //float closestValueToSurface = 100000;
    //unsigned int closestIndexToSurface = -1;
    int *candidateIds = goodCandidates.as(s32).host<int>();
    static af::array temporaryCandidate;
    static af::array mask;
    for(size_t i = 0, iend = goodCandidates.dims(0); i < iend; ++i)
    {
      mask = m_connectedComponents == candidateIds[i];
      temporaryCandidate = (*m_workspace) * mask.as(f32);

      /*af::array closestElements = temporaryCandidate > m_depthThreshold;
      af::print("closestElements", closestElements);

      float value;
      unsigned int index;
      af::min<float>(&value, &index, temporaryCandidate);

      if(closestValueToSurface > value)
      {
        closestValueToSurface = value;
        closestIndexToSurface = index;
        std::cout << "\n\n Value=" << closestValueToSurface << ", Index=" << closestIndexToSurface << "\n\n";
      }*/

#ifdef DEBUG_TOUCH
      af::array temporaryCandidateDisplay = temporaryCandidate * 1000.0f + (50.0f * (temporaryCandidate == 0));
      OpenCVExtra::ocvfig("tmp" + std::to_string(i), temporaryCandidateDisplay.as(u8).host<unsigned char>(), cols, rows, OpenCVExtra::Order::COL_MAJOR);
      cv::waitKey(5);
#endif


    }
  }

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH)
  cv::waitKey(m_debugDelayms);
  initialised = true;
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
}

