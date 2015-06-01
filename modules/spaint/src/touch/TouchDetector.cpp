/**
 * spaint: TouchDetector.cpp
 */

#include "touch/TouchDetector.h"

#include <iostream>

#include "imageprocessing/cpu/ImageProcessor_CPU.h"
#include "tvgutil/ArgUtil.h"
#include "visualisers/cpu/DepthVisualiser_CPU.h"
#ifdef WITH_CUDA
#include "imageprocessing/cuda/ImageProcessor_CUDA.h"
#include "visualisers/cuda/DepthVisualiser_CUDA.h"
#endif

#ifdef WITH_OPENCV
#include "ocv/OpenCVExtra.h"
#endif

namespace spaint {

//#################### CONSTRUCTORS ####################

TouchDetector::TouchDetector(const Vector2i& imgSize)
: m_cols(imgSize.x),
  m_connectedComponents(imgSize.y, imgSize.x, u32),
  m_depthLowerThreshold(0.010f),
  m_depthUpperThreshold(0.255f),
  m_diffRawRaycast(new af::array(imgSize.y, imgSize.x, f32)),
  m_maximumConnectedComponentAreaPercentage(20),
  m_minimumConnectedComponentAreaPercentage(1), // 1%.
  m_morphKernelSize(5),
  m_rows(imgSize.y),
  m_thresholded(imgSize.y, imgSize.x)
{
#ifdef WITH_CUDA
  m_rawDepthCopy.reset(new ITMFloatImage(imgSize, true, true));
  m_raycastedDepthResult.reset(new ITMFloatImage(imgSize, true, true));

  m_depthCalculator.reset(new DepthVisualiser_CUDA);
  m_imageProcessor.reset(new ImageProcessor_CUDA);
#else
  m_rawDepthCopy.reset(new ITMFloatImage(imgSize, true, false));
  m_raycastedDepthResult.reset(new ITMFloatImage(imgSize, true, false));

  m_depthCalculator.reset(new DepthVisualiser_CPU);
  m_imageProcessor.reset(new ImageProcessor_CPU);
#endif

  // The minimum area threshold is set to a percentage of the total image area.
  m_minimumConnectedComponentAreaThreshold = (m_minimumConnectedComponentAreaPercentage / 100.0f) * (m_rows * m_cols);

  // The maximum are threshold is set to a precentage of the total image area.
  m_maximumConnectedComponentAreaThreshold = (m_maximumConnectedComponentAreaPercentage / 100.0f) * (m_rows * m_cols);

#ifdef DEBUG_TOUCH_DISPLAY
  m_debugDelayms = 30;
  m_depthLowerThresholdmm = m_depthLowerThreshold * 1000.0f;
#endif
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void TouchDetector::run_touch_detector_on_frame(const RenderState_CPtr& renderState, const rigging::MoveableCamera_CPtr camera, float voxelSize, const FloatImage_CPtr& rawDepth)
{
  calculate_binary_difference_image(renderState, camera, voxelSize, rawDepth);

  filter_binary_image();

  // Calculate the connected components.
  m_connectedComponents = af::regions(m_thresholded);

  af::array goodCandidates = select_good_connected_components();

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
  // Display the connected components.
  static af::array connectedComponentsDisplay(m_rows, m_cols, u8);
  int numberOfConnectedComponents = af::max<int>(m_connectedComponents) + 1;
  connectedComponentsDisplay = m_connectedComponents * (255/numberOfConnectedComponents);
  OpenCVExtra::ocvfig("connectedComponentsDisplay", connectedComponentsDisplay.as(u8).host<unsigned char>(), m_cols, m_rows, OpenCVExtra::COL_MAJOR);
#endif

  // Post-process the good candidates to identify the region which is most likely to be touching a surface.
  if(goodCandidates.elements() > 0)
  {
    static af::array diffCopyMillimetersU8(m_rows, m_cols, u8);

    // Convert the difference between raw and raycasted depth to millimeters.
    diffCopyMillimetersU8 = truncate_to_unsigned_char((*m_diffRawRaycast) * 1000.0f);

    // Find the connected component most likely to be a touch iteractor.
    int bestConnectedComponent = find_best_connected_component(goodCandidates, diffCopyMillimetersU8);

    // Get the touchPoints, will return empty if the best connected component is not touching the scene.
    Points_CPtr touchPoints = get_touch_points(bestConnectedComponent, diffCopyMillimetersU8);

    if(touchPoints->size() > 0) m_touchState.set_touch_state(touchPoints, true, true);
    else m_touchState.set_touch_state(touchPoints, false, false);
  }
  else{
    m_touchState.set_touch_state(Points_CPtr(new Points), false, false);
  }

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
  // Display the touch points.
  cv::Mat touchPointImage = cv::Mat::zeros(m_rows, m_cols, CV_8UC1);
  const Points_CPtr& touchPoints = m_touchState.get_positions();
  for(size_t i = 0, iend = touchPoints->size(); i < iend; ++i)
  {
    const Eigen::Vector2i& v = touchPoints->at(i);
    cv::circle(touchPointImage, cv::Point(v[0], v[1]), 5, cv::Scalar(255), 2);
  }
  cv::imshow("touchPointImage", touchPointImage);
#endif

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
  cv::waitKey(m_debugDelayms);
#endif
}

const TouchState& TouchDetector::get_touch_state() const
{
  return m_touchState;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void TouchDetector::calculate_binary_difference_image(const RenderState_CPtr& renderState, const rigging::MoveableCamera_CPtr camera, float voxelSize, const FloatImage_CPtr& rawDepth)
{
  // The camera is assumed to be positioned close to the user.
  // This allows a threshold on the maximum depth that a touch interaction may occur.
  // For example the user's hand or leg cannot extend more than 2 meters away from the camera.
  // This turns out to be crucial since there may be large areas of the scene far away that are picked up by the Kinect but which are not integrated into the scene (InfiniTAM has a
  // depth threshold hard coded into its scene settings).
  m_imageProcessor->pixel_setter(m_rawDepthCopy, rawDepth, 2.0f, ImageProcessor::CO_GREATER, -1.0f);

  // Calculate the depth raycast from the current scene, this is in metres.
  m_depthCalculator->render_depth(
    renderState.get(),
    to_itm(camera->p()),
    to_itm(camera->n()),
    voxelSize,
    DepthVisualiser::DT_ORTHOGRAPHIC,
    m_raycastedDepthResult.get()
  );

  // Pre-process the raycasted depth result, so that regions of the image which are not valid are assigned a large depth value (infinity).
  // In this case 100.0f meters.
  m_imageProcessor->pixel_setter(m_raycastedDepthResult, m_raycastedDepthResult, 0.0f, ImageProcessor::CO_LESS, 100.0f);

  // Calculate the difference between the raw depth and the raycasted depth.
  m_imageProcessor->calculate_absolute_difference(m_rawDepthCopy, m_raycastedDepthResult, m_diffRawRaycast);

  // Threshold the difference image - if there is a difference, there is something in the image!
  m_thresholded = *m_diffRawRaycast > m_depthLowerThreshold;

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
  // Display the raw depth and the raycasted depth.
  OpenCVExtra::display_image_and_scale(m_rawDepthCopy.get(), 100.0f, "Current raw depth from camera in centimeters");
  OpenCVExtra::display_image_and_scale(m_raycastedDepthResult.get(), 100.0f, "Current depth raycast in centimeters");

  // Display the absolute difference between the raw and raycasted depth.
  static af::array tmp;
  tmp = *m_diffRawRaycast * 100.0f; // Convert to centimeters.
  tmp = truncate_to_unsigned_char(tmp);
  OpenCVExtra::ocvfig("Diff image in arrayfire", tmp.as(u8).host<unsigned char>(), m_cols, m_rows, OpenCVExtra::COL_MAJOR);

  static bool initialised = false;

  // Initialise the OpenCV Trackbar if it's the first iteration.
  if(!initialised)
  {
    cv::namedWindow("DebuggingOutputWindow", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("depthThresholdmm", "DebuggingOutputWindow", &m_depthLowerThresholdmm, 50);
    cv::createTrackbar("debugDelaymx", "DebuggingOutputWindow", &m_debugDelayms, 3000);
  }
  // Get the values from the trackBars
  m_depthLowerThresholdmm = cv::getTrackbarPos("depthThresholdmm", "DebuggingOutputWindow");
  m_debugDelayms = cv::getTrackbarPos("debugDelaymx", "DebuggingOutputWindow");

  // Update the current depth Threshold in meters;
  m_depthLowerThreshold = m_depthLowerThresholdmm / 1000.0f;

  // Display the thresholded image.
  static af::array thresholdedDisplay;
  thresholdedDisplay = m_thresholded * 255.0f;
  OpenCVExtra::ocvfig("DebuggingOutputWindow", thresholdedDisplay.as(u8).host<unsigned char>(), m_cols, m_rows, OpenCVExtra::COL_MAJOR);

  initialised = true;
#endif
}

void TouchDetector::filter_binary_image()
{
  // Perform morphological operations on the image to get rid of small segments.
  static af::array morphKernel;

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
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

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
  static bool initialised = false;

  if(!initialised)
  {
    cv::namedWindow("MorphologicalOperatorWindow", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("KernelSize", "MorphologicalOperatorWindow", &m_morphKernelSize, 15);
  }
  m_morphKernelSize = cv::getTrackbarPos("KernelSize", "MorphologicalOperatorWindow");

  // Display the threholded image after applying morphological operations.
  static af::array morphDisplay;
  morphDisplay = m_thresholded * 255.0f;
  OpenCVExtra::ocvfig("MorphologicalOperatorWindow", morphDisplay.as(u8).host<unsigned char>(), m_cols, m_rows, OpenCVExtra::COL_MAJOR);

  initialised = true;
#endif
}

int TouchDetector::find_best_connected_component(const af::array& goodCandidates, const af::array& diffCopyMillimetersU8)
{
  int numberOfGoodCandidates = goodCandidates.dims(0);
  int *candidateIds = goodCandidates.as(s32).host<int>();
  int bestConnectedComponent = -1;

  static af::array temporaryCandidate(m_rows, m_cols, u8);
  static af::array mask(m_rows, m_cols, b8);

  // If there are several good candidate then select the one which is closest to a surface.
  if(numberOfGoodCandidates > 1)
  {
    std::vector<float> means(numberOfGoodCandidates);
    for(int i = 0; i < numberOfGoodCandidates; ++i)
    {
      mask = (m_connectedComponents == candidateIds[i]);
      temporaryCandidate = diffCopyMillimetersU8 * mask;
      means[i] = af::mean<float>(temporaryCandidate);
    }
    size_t minIndex = tvgutil::ArgUtil::argmin(means);
    if(minIndex < 0 || minIndex >= numberOfGoodCandidates) throw std::runtime_error("Out of bounds error");
    bestConnectedComponent = candidateIds[minIndex];
  }
  else
  {
    bestConnectedComponent = candidateIds[0];
  }

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
  mask = (m_connectedComponents.as(u32) == bestConnectedComponent);
  //af::print("m_connectedComponents", m_connectedComponents);
  temporaryCandidate = diffCopyMillimetersU8 * mask;

  // Display the best candidate's difference image.
  static af::array temporaryCandidateDisplay(m_rows, m_cols, u8);
  temporaryCandidateDisplay = temporaryCandidate;
  OpenCVExtra::ocvfig("bestConnectedComponent", temporaryCandidateDisplay.host<unsigned char>(), m_cols, m_rows, OpenCVExtra::COL_MAJOR);

  static af::array maskDisplay(m_rows, m_cols, u8);
  maskDisplay = mask * 255;
  OpenCVExtra::ocvfig("maskDisplay", maskDisplay.as(u8).host<unsigned char>(), m_cols, m_rows, OpenCVExtra::COL_MAJOR);
#endif

  return bestConnectedComponent;
}

TouchDetector::Points_CPtr TouchDetector::get_touch_points(int bestConnectedComponent, const af::array& diffCopyMillimetersU8)
{
  Points_Ptr touchPoints(new Points);

  // Binary mask identifying the region with the best candidate.
  static af::array mask;
  mask = m_connectedComponents == bestConnectedComponent;

  // Get the diff values corresponding to the best candidate region.
  static af::array temporaryCandidate;
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

  static float touchAreaLowerThreshold = 0.0001f * m_cols * m_rows;
  if(goodPixelPositions.elements() > touchAreaLowerThreshold)
  {
#ifdef DEBUG_TOUCH_VERBOSE
    af::print("goodPixelPositions", goodPixelPositions);
#endif

    int numberOfTouchPoints = goodPixelPositions.dims(0);
    int *touchIndices = goodPixelPositions.as(s32).host<int>();

    for(int i = 0; i < numberOfTouchPoints; ++i)
    {
      Eigen::Vector2i point((touchIndices[i] / (int)(m_rows * scaleFactor)) * float(1.0f / (scaleFactor)), (touchIndices[i] % (int)(m_rows * scaleFactor)) * float(1.0f / (scaleFactor)));

      touchPoints->push_back(point);
    }
  }

  return touchPoints;
}

af::array TouchDetector::select_good_connected_components()
{
  int numberOfConnectedComponents = af::max<int>(m_connectedComponents) + 1;

  // Create a histogram of the connected components to identify the size of each region.
  static af::array histogram;
  histogram = af::histogram(m_connectedComponents, numberOfConnectedComponents);

  // Remove connected components which are too small or too large.
  histogram = histogram - (histogram < m_minimumConnectedComponentAreaThreshold) * histogram;
  histogram = histogram - (histogram > m_maximumConnectedComponentAreaThreshold) * histogram;
  af::array goodCandidates = af::where(histogram);
#ifdef DEBUG_TOUCH_VERBOSE
  af::print("Histogram of connected component image", histogram);
  if(goodCandidates.elements() > 0) af::print("goodCandidates", goodCandidates);
#endif

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
  static bool initialised = false;

  // Initialise the OpenCV Trackbar if it's the first iteration.
  if(!initialised)
  {
    cv::createTrackbar("minimumAreaThreshold", "DebuggingOutputWindow", &m_minimumConnectedComponentAreaThreshold, m_rows * m_cols);
    cv::createTrackbar("maximumAreaThreshold", "DebuggingOutputWindow", &m_maximumConnectedComponentAreaThreshold, m_rows * m_cols);
  }
  m_minimumConnectedComponentAreaThreshold = cv::getTrackbarPos("minimumAreaThreshold", "DebuggingOutputWindow");
  m_maximumConnectedComponentAreaThreshold = cv::getTrackbarPos("maximumAreaThreshold", "DebuggingOutputWindow");

  initialised = true;
#endif

  // The good candidates are those whose area is greater than some specified threshold.
  return goodCandidates;
}

af::array TouchDetector::truncate_to_unsigned_char(const af::array& array)
{
  // Truncate any values outside the range [0-255], before converting to unsigned 8-bit.
  static af::array lowmask;
  lowmask = array < 0.0f;

  static af::array arrayCopy;
  arrayCopy = array - lowmask * array;

  static af::array highmask;
  highmask = arrayCopy > 255.0f;
  arrayCopy = arrayCopy - (highmask * arrayCopy) + (255 * highmask);

  return arrayCopy.as(u8);
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

Vector3f TouchDetector::to_itm(const Eigen::Vector3f& v)
{
  return Vector3f(v[0], v[1], v[2]);
}

}
