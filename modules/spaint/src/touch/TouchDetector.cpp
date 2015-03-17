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
  m_depthLowerThreshold(0.010f), 
  m_depthUpperThreshold(0.255f),
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
  m_depthLowerThresholdmm = m_depthLowerThreshold * 1000;
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
  m_thresholded = (*m_workspace > m_depthLowerThreshold) && (*m_workspace < m_depthUpperThreshold);

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH)
  // Initialise the OpenCV Trackbar if it's the first iteration.
  if(!initialised)
  {
    cv::namedWindow("DebuggingOutputWindow", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("depthThresholdmm", "DebuggingOutputWindow", &m_depthLowerThresholdmm, 50);
    cv::createTrackbar("debugDelaymx", "DebuggingOutputWindow", &m_debugDelayms, 3000);
    cv::createTrackbar("areaThreshold", "DebuggingOutputWindow", &m_areaThreshold, rows*cols);
  }
  // Update the current depth Threshold in meters;
  m_depthLowerThreshold = m_depthLowerThresholdmm / 1000.0f;

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

/*
#ifdef DEBUG_TOUCH
  af::print("Histogram of connected component image", histogram);
#endif
*/

  af::array goodCandidates = af::where(histogram > m_areaThreshold);

/*
#ifdef DEBUG_TOUCH
  if(goodCandidates.elements() > 0)
  {
    af::print("goodCandidates", goodCandidates);
  }
#endif
*/

  if(goodCandidates.elements() > 0)
  {
    int numberOfGoodCandidates = goodCandidates.dims(0);
    int bestCandidate;
    int *candidateIds = goodCandidates.as(s32).host<int>();

    static af::array mask(rows, cols, u8);
    static af::array temporaryCandidate(rows, cols, u8);
    static af::array workspaceCopyMillimeters32F(rows, cols, f32);
    static af::array workspaceCopyMillimetersU8(rows, cols, u8);

    workspaceCopyMillimeters32F = ((*m_workspace) * 1000.0f);
    workspaceCopyMillimeters32F = workspaceCopyMillimeters32F - ((workspaceCopyMillimeters32F > 255.0f) | (workspaceCopyMillimeters32F < 0)) * workspaceCopyMillimeters32F;
    workspaceCopyMillimetersU8 = workspaceCopyMillimeters32F.as(u8);

    if(numberOfGoodCandidates > 1)
    {
      std::vector<float> means(numberOfGoodCandidates);
      int minElement;
      for(int i = 0; i < numberOfGoodCandidates; ++i)
      {
        mask = m_connectedComponents == candidateIds[i];
        temporaryCandidate = workspaceCopyMillimetersU8 * mask;
        means[i] = af::mean<float>(temporaryCandidate);
        //std::cout << "mean" << i << "=" << means[i] << '\n';
      }
      minElement = *std::min_element(means.begin(), means.end());
      bestCandidate = candidateIds[minElement];
      //std::cout << "bestCandidate=" << bestCandidate << '\n';
    }
    else
    {
      bestCandidate = candidateIds[0];
    }

    mask = m_connectedComponents == bestCandidate;
    temporaryCandidate = workspaceCopyMillimetersU8 * mask;

    // Quantize the intensity levels to 32 levels from 256.
    temporaryCandidate = (temporaryCandidate / 8).as(u8) * 8;
    // Filter the best candidate region prior to calculating the histogram.
    af::medfilt(temporaryCandidate, 5, 5); // NOt sure if this is needed yet.

    af::array goodPixelPositions = af::where((temporaryCandidate > m_depthLowerThreshold * 1000.0f ) && (temporaryCandidate < 20));

    if(goodPixelPositions.elements() > 0.0001 * cols * rows)
    {
#ifdef DEBUG_TOUCH
      /*
      af::print("goodPixelPositions", goodPixelPositions);
      */
      cv::Mat touchPointImage = cv::Mat::zeros(rows, cols, CV_8UC1);
#endif

      std::vector<int> pointsx;
      std::vector<int> pointsy;
      int numberOfTouchPoints = goodPixelPositions.dims(0);
      int *touchIndices = goodPixelPositions.as(s32).host<int>();

      for(int i = 0; i < numberOfTouchPoints; ++i)
      {
        pointsx.push_back(touchIndices[i] / rows); // Column.
        pointsy.push_back(touchIndices[i] % rows); // Row.

#ifdef DEBUG_TOUCH
        cv::circle(touchPointImage, cv::Point(pointsx.back(), pointsy.back()), 5, cv::Scalar(255), 2);
#endif
      }

#ifdef DEBUG_TOUCH
      cv::imshow("touchPointImage", touchPointImage);
      cv::waitKey(5);
#endif
      m_touchState.set_touch_state(pointsx[0], pointsy[0], true, true);
    }
    else
    {
      m_touchState.set_touch_state(-1, -1, false, false);
    }

#ifdef DEBUG_TOUCH
    static af::array temporaryCandidateDisplay(rows, cols, u8);
    temporaryCandidateDisplay = temporaryCandidate;
    OpenCVExtra::ocvfig("bestCandidate", temporaryCandidateDisplay.host<unsigned char>(), cols, rows, OpenCVExtra::Order::COL_MAJOR);
    cv::waitKey(5);
#endif


    /*
    static af::array depthHistogram;
    depthHistogram = af::histogram(temporaryCandidate, 256);
    depthHistogram(0) = 0; // These pixels correspond to the background.

    af::array goodPointerCandidates = af::where(depthHistogram > 10); //10 pixels.

#ifdef DEBUG_TOUCH
    if(goodPointerCandidates.elements() > 0)
    {
      af::print("goodPointerCandidates", goodPointerCandidates);
    }
#endif

    int numberOfGoodPointerCandidates = goodPointerCandidates.dims(0);
    int *goodPointerCandidateIds = goodPointerCandidates.as(s32).host<int>();
    for(int i = 0; i < numberOfGoodPointerCandidates; ++i)
    {
      if(goodPointerCandidateIds[i] < 15) //15 millimeters.
      {
        std::cout << "We have touchdown!\n";
        // Calculate the pixel positions at this depth value.

        af::array pixelPositions = af::where(temporaryCandidate == goodPointerCandidateIds[i]);
        if(pixelPositions.elements() > 0)
        {
          af::print("pixelPositions", pixelPositions);
        }
      }
    }

#ifdef DEBUG_TOUCH
    af::print("Histogram of depth values in best candidate", depthHistogram);
#endif
    */
  }
  else{
    m_touchState.set_touch_state(-1, -1, false, false);
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

