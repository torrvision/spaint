/**
 * spaint: TouchDetector.cpp
 */

#include "touch/TouchDetector.h"

#include <iostream>

#include "imageprocessing/cpu/ImageProcessor_CPU.h"
#include "visualisers/cpu/DepthVisualiser_CPU.h"
#ifdef WITH_CUDA
#include "imageprocessing/cuda/ImageProcessor_CUDA.h"
#include "visualisers/cuda/DepthVisualiser_CUDA.h"
#endif

#ifdef WITH_OPENCV
#include "ocv/OpenCVExtra.h"
#endif
#include "util/TypeConversions.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

TouchDetector::TouchDetector(const Vector2i& imgSize)
: m_areaPercentageThreshold(1), // 1%.
  m_cols(imgSize.x),
  m_connectedComponents(imgSize.y, imgSize.x),
  m_depthLowerThreshold(0.010f),
  m_depthUpperThreshold(0.255f),
  m_diffRawRaycast(new af::array(imgSize.y, imgSize.x, f32)),
  m_morphKernelSize(5),
  m_rows(imgSize.y),
  m_thresholded(imgSize.y, imgSize.x)
{
#ifdef WITH_CUDA
  m_raycastedDepthResult.reset(new ITMFloatImage(imgSize, true, true));

  m_depthCalculator.reset(new DepthVisualiser_CUDA);
  m_imageProcessor.reset(new ImageProcessor_CUDA);
#else
  m_raycastedDepthResult.reset(new ITMFloatImage(imgSize, true, false));

  m_depthCalculator.reset(new DepthVisualiser_CPU);
  m_imageProcessor.reset(new ImageProcessor_CPU);
#endif

  // The minimum area threshold is set to a percentage of the total image area.
  m_minimumAreaThreshold = (m_areaPercentageThreshold / 100.0f) * (m_rows * m_cols);

#ifdef DEBUG_TOUCH_DISPLAY
  m_debugDelayms = 30;
  m_depthLowerThresholdmm = m_depthLowerThreshold * 1000.0f;
#endif
}

//#################### PUBLIC MEMBER FUNCTIONS ####################
void TouchDetector::run_touch_detector_on_frame(const RenderState_CPtr& renderState, const rigging::MoveableCamera_Ptr camera, float voxelSize, ITMFloatImage *rawDepth)
{
  // Calculate the depth raycast from the current scene, this is in metres.
  m_depthCalculator->render_depth(m_raycastedDepthResult.get(),
      renderState.get(),
      TypeConversions::to_itm(camera->p()),
      TypeConversions::to_itm(camera->n()),
      voxelSize,
      DepthVisualiser::DT_ORTHOGRAPHIC);

  // Calculate the difference between the raw depth and the raycasted depth.
  m_imageProcessor->absolute_difference_calculator(m_diffRawRaycast.get(), rawDepth, m_raycastedDepthResult.get());

#if 0
  // Display the raw depth and the raycasted depth.
  OpenCVExtra::display_image_and_scale(rawDepth, 100.0f, "Current raw depth from camera in millimeters");
  OpenCVExtra::display_image_and_scale(m_raycastedDepthResult.get(), 100.0f, "Current depth raycast in millimeters");

  // Display the absolute difference between the raw and raycasted depth.
  static af::array tmp;
  tmp = *m_diffRawRaycast * 1000.0f;
  OpenCVExtra::ocvfig("Diff image in arrayfire", tmp.as(u8).host<unsigned char>(), m_cols, m_rows, OpenCVExtra::Order::COL_MAJOR);

  cv::waitKey(100);
#endif

#if 1
  // Threshold the difference image.
  m_thresholded = (*m_diffRawRaycast > m_depthLowerThreshold) && (*m_diffRawRaycast < m_depthUpperThreshold);

  // Perform morphological operations on the image to get rid of small segments.
  static af::array morphKernel;
#ifdef DEBUG_TOUCH_DISPLAY
  m_morphKernelSize = (m_morphKernelSize < 3) ? 3 : m_morphKernelSize;

  // Keep the morphological kernel size odd;
  if((m_morphKernelSize % 2) == 0)
  {
    ++m_morphKernelSize;
  }
#endif
  morphKernel = af::constant(1, m_morphKernelSize, m_morphKernelSize);

  // Apply morphological operations on the thresholded image.
  m_thresholded = af::erode(m_thresholded, morphKernel);
  m_thresholded = af::dilate(m_thresholded, morphKernel);

  // Calculate the connected components.
  m_connectedComponents = af::regions(m_thresholded);
  int numberOfConnectedComponents = af::max<int>(m_connectedComponents) + 1;

  // Create a histogram of the connected components to identify the size of each region.
  static af::array histogram;
  histogram = af::histogram(m_connectedComponents, numberOfConnectedComponents);

  // Set the first element to zero as this corresponds to the background.
  histogram(0) = 0;

#ifdef DEBUG_TOUCH_VERBOSE
  af::print("Histogram of connected component image", histogram);
#endif

  // The good candidates are those whose area is greater than some specified threshold.
  af::array goodCandidates = af::where(histogram > m_minimumAreaThreshold);

#ifdef DEBUG_TOUCH_VERBOSE
  if(goodCandidates.elements() > 0) af::print("goodCandidates", goodCandidates);
#endif

  // Post-process the good candidates to identify the region which is most likely to be touching a surface.
  static af::array temporaryCandidate(m_rows, m_cols, u8);
  static boost::shared_ptr<std::vector<Eigen::Vector2i> > points;
  points.reset(new std::vector<Eigen::Vector2i>());

  if(goodCandidates.elements() > 0)
  {
    int numberOfGoodCandidates = goodCandidates.dims(0);
    int bestCandidate;
    int *candidateIds = goodCandidates.as(s32).host<int>();

    static af::array mask(m_rows, m_cols, u8);
    static af::array diffCopyMillimeters32F(m_rows, m_cols, f32);
    static af::array diffCopyMillimetersU8(m_rows, m_cols, u8);

    // Convert the difference between raw and raycasted depth to millimeters.
    diffCopyMillimeters32F = ((*m_diffRawRaycast) * 1000.0f);

    // Truncate any values outside the range [0-255], before converting to unsigned 8-bit.
    diffCopyMillimeters32F = diffCopyMillimeters32F - ((diffCopyMillimeters32F > 255.0f) | (diffCopyMillimeters32F < 0)) * diffCopyMillimeters32F;
    diffCopyMillimetersU8 = diffCopyMillimeters32F.as(u8);

    // If there are several good candidate then select the one which is closest to a surface.
    if(numberOfGoodCandidates > 1)
    {
      std::vector<float> means(numberOfGoodCandidates);
      int minElement;
      for(int i = 0; i < numberOfGoodCandidates; ++i)
      {
        mask = m_connectedComponents == candidateIds[i];
        temporaryCandidate = diffCopyMillimetersU8 * mask;
        means[i] = af::mean<float>(temporaryCandidate);
      }
      minElement = *std::min_element(means.begin(), means.end());
      bestCandidate = candidateIds[minElement];
    }
    else
    {
      bestCandidate = candidateIds[0];
    }

    // Binary mask identifying the region with the best candidate.
    mask = m_connectedComponents == bestCandidate;

    // Get the diff values corresponding to the best candidate region.
    temporaryCandidate = diffCopyMillimetersU8 * mask;

    // Quantize the intensity levels to 32 levels from 256.
    temporaryCandidate = (temporaryCandidate / 8).as(u8) * 8;

    // Filter the best candidate region prior to calculating the histogram.
    af::medfilt(temporaryCandidate, 5, 5);

    // Find the array positions which are within a narrow range close to a surface.
    static float depthLowerThresholdMillimeters = m_depthLowerThreshold * 1000.0f;
    static float depthUpperThresholdMillimeters = depthLowerThresholdMillimeters + 10.0f;

    static float scaleFactor = 0.5f;
    af::array goodPixelPositions = af::where(af::resize(scaleFactor, (temporaryCandidate > depthLowerThresholdMillimeters) && (temporaryCandidate < depthUpperThresholdMillimeters)));

    static float touchAreaLowerThreshold = 0.0001 * m_cols * m_rows;
    if(goodPixelPositions.elements() > touchAreaLowerThreshold)
    {
#ifdef DEBUG_TOUCH_VERBOSE
      af::print("goodPixelPositions", goodPixelPositions);
#endif

      int numberOfTouchPoints = goodPixelPositions.dims(0);
      int *touchIndices = goodPixelPositions.as(s32).host<int>();

      for(int i = 0; i < numberOfTouchPoints; ++i)
      {
        /*static float quantizationRows = 0.006 * m_rows;
        static float quantizationCols = 0.006 * m_cols;

        int column = touchIndices[i] / m_rows;
        int row = touchIndices[i] % m_rows;
        int qcolumn = (int)(column / quantizationCols) * quantizationCols;
        int qrow = (int)(row / quantizationRows) * quantizationRows;

        pointsx.push_back(qcolumn); // Column.
        pointsy.push_back(qrow); // Row.
        */
        Eigen::Vector2i point((touchIndices[i] / (int)(m_rows * scaleFactor)) * float(1.0f / (scaleFactor)), (touchIndices[i] % (int)(m_rows * scaleFactor)) * float(1.0f / (scaleFactor)));

        points->push_back(point);
        //pointsx.push_back((touchIndices[i] / (int)(m_rows * scaleFactor)) * float(1.0f / (scaleFactor)));
        //pointsy.push_back((touchIndices[i] % (int)(m_rows * scaleFactor)) * float(1.0f / (scaleFactor)));
      }

      m_touchState.set_touch_state(points, true, true);
    }
    else
    {
      m_touchState.set_touch_state(points, false, false);
    }
  }
  else{
    m_touchState.set_touch_state(points, false, false);
  }
#endif

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
    run_debugging_display(rawDepth, temporaryCandidate);
#endif
}

const TouchState& TouchDetector::get_touch_state() const
{
  return m_touchState;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
void TouchDetector::run_debugging_display(ITMFloatImage *rawDepth, const af::array& temporaryCandidate)
{
  static bool initialised = false;

  // Display the raw depth and the raycasted depth.
  OpenCVExtra::display_image_scale_to_range(rawDepth, "Current raw depth from camera in millimeters");
  OpenCVExtra::display_image_scale_to_range(m_raycastedDepthResult.get(), "Current depth raycast in millimeters");

  // Display the absolute difference between the raw and raycasted depth.
  static af::array tmp;
  tmp = *m_diffRawRaycast * 1000.0f;
  OpenCVExtra::ocvfig("Diff image in arrayfire", tmp.as(u8).host<unsigned char>(), m_cols, m_rows, OpenCVExtra::COL_MAJOR);

  // Initialise the OpenCV Trackbar if it's the first iteration.
  if(!initialised)
  {
    cv::namedWindow("DebuggingOutputWindow", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("depthThresholdmm", "DebuggingOutputWindow", &m_depthLowerThresholdmm, 50);
    cv::createTrackbar("debugDelaymx", "DebuggingOutputWindow", &m_debugDelayms, 3000);
    cv::createTrackbar("areaThreshold", "DebuggingOutputWindow", &m_minimumAreaThreshold, m_rows * m_cols);
  }

  // Update the current depth Threshold in meters;
  m_depthLowerThreshold = m_depthLowerThresholdmm / 1000.0f;

  // Display the thresholded image.
  static af::array thresholdedDisplay;
  thresholdedDisplay = m_thresholded * 255.0f;
  OpenCVExtra::ocvfig("DebuggingOutputWindow", thresholdedDisplay.as(u8).host<unsigned char>(), m_cols, m_rows, OpenCVExtra::COL_MAJOR);

#if 1
  // Initialise the OpenCV Trackbar for the morphological kernel size.
  if(!initialised)
  {
    cv::namedWindow("MorphologicalOperatorWindow", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("KernelSize", "MorphologicalOperatorWindow", &m_morphKernelSize, 15);
  }

  // Display the threholded image after applying morphological operations.
  static af::array morphDisplay;
  morphDisplay = m_thresholded * 255.0f;
  OpenCVExtra::ocvfig("MorphologicalOperatorWindow", morphDisplay.as(u8).host<unsigned char>(), m_cols, m_rows, OpenCVExtra::COL_MAJOR);

  // Display the best candidate's difference image.
  static af::array temporaryCandidateDisplay(m_rows, m_cols, u8);
  temporaryCandidateDisplay = temporaryCandidate;
  OpenCVExtra::ocvfig("bestCandidate", temporaryCandidateDisplay.host<unsigned char>(), m_cols, m_rows, OpenCVExtra::COL_MAJOR);

  // Display the touch points.
  cv::Mat touchPointImage = cv::Mat::zeros(m_rows, m_cols, CV_8UC1);
  const boost::shared_ptr<const std::vector<Eigen::Vector2i> >& points = m_touchState.get_positions();
  for(size_t i = 0, iend = points->size(); i < iend; ++i)
  {
    const Eigen::Vector2i v = points->at(i);
    cv::circle(touchPointImage, cv::Point(v[0], v[1]), 5, cv::Scalar(255), 2);
  }
  cv::imshow("touchPointImage", touchPointImage);
#endif

  cv::waitKey(m_debugDelayms);
  initialised = true;
}
#endif

}

