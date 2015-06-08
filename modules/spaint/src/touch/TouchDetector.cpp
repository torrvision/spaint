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
#include "ocv/OpenCVUtil.h"
#endif

//#define DEBUG_TOUCH_VERBOSE
#define DEBUG_TOUCH_DISPLAY

namespace spaint {

//#################### CONSTRUCTORS ####################

TouchDetector::TouchDetector(const Vector2i& imgSize, const Settings_CPtr& settings)
: m_changeMask(imgSize.y, imgSize.x),
  m_cols(imgSize.x),
  m_connectedComponentImage(imgSize.y, imgSize.x, u32),
  m_depthLowerThreshold(0.010f),
  m_depthUpperThreshold(0.255f),
  m_diffRawRaycast(new af::array(imgSize.y, imgSize.x, f32)),
  m_maximumConnectedComponentAreaPercentage(20),
  m_minimumConnectedComponentAreaPercentage(1), // 1%.
  m_morphKernelSize(5),
  m_rows(imgSize.y),
  m_settings(settings)
{
  m_thresholdedRawDepth.reset(new ITMFloatImage(imgSize, true, true));
  m_depthRaycast.reset(new ITMFloatImage(imgSize, true, true));

  // FIXME: Take the device type in settings into account.
#ifdef WITH_CUDA
  m_depthCalculator.reset(new DepthVisualiser_CUDA);
  m_imageProcessor.reset(new ImageProcessor_CUDA);
#else
  m_depthCalculator.reset(new DepthVisualiser_CPU);
  m_imageProcessor.reset(new ImageProcessor_CPU);
#endif

  // The minimum area threshold is set to a percentage of the total image area.
  m_minimumConnectedComponentAreaThreshold = static_cast<int>((m_minimumConnectedComponentAreaPercentage / 100.0f) * (m_rows * m_cols));

  // The maximum are threshold is set to a percentage of the total image area.
  m_maximumConnectedComponentAreaThreshold = static_cast<int>((m_maximumConnectedComponentAreaPercentage / 100.0f) * (m_rows * m_cols));

#ifdef DEBUG_TOUCH_DISPLAY
  m_debugDelayms = 30;
  m_depthLowerThresholdmm = static_cast<int>(m_depthLowerThreshold * 1000.0f);
#endif
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

std::vector<Eigen::Vector2i> TouchDetector::determine_touch_points(const rigging::MoveableCamera_CPtr& camera, const ITMFloatImage_CPtr& rawDepth, const RenderState_CPtr& renderState)
try
{
  // Prepare a thresholded version of the raw depth image and a depth raycast ready for change detection.
  prepare_inputs(camera, rawDepth, renderState);

  // Detect changes in the scene with respect to the reconstructed model.
  detect_changes();

  // Make a connected-component image from the change mask.
  m_connectedComponentImage = af::regions(m_changeMask);

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
  // Display the connected components.
  static af::array connectedComponentsDisplay(m_rows, m_cols, u8);
  int numberOfConnectedComponents = af::max<int>(m_connectedComponentImage) + 1;
  connectedComponentsDisplay = m_connectedComponentImage * (255/numberOfConnectedComponents);
  OpenCVUtil::show_greyscale_figure("connectedComponentsDisplay", connectedComponentsDisplay.as(u8).host<unsigned char>(), m_cols, m_rows, OpenCVUtil::COL_MAJOR);
#endif

  // Select candidate connected components that fall within a certain size range. If no components meet the size constraints, early out.
  af::array candidateComponents = select_candidate_components();
  if(candidateComponents.isempty()) return std::vector<Eigen::Vector2i>();

  // Convert the differences between the raw depth image and the depth raycast to millimetres.
  af::array diffRawRaycastInMm = clamp_to_range(*m_diffRawRaycast * 1000.0f, 0.0f, 255.0f).as(u8);

  // Pick the candidate component most likely to correspond to a touch interaction.
  int bestConnectedComponent = pick_best_candidate_component(candidateComponents, diffRawRaycastInMm);

  // Extract a set of touch points from the chosen connected component that denote the parts of the scene touched by the user.
  // Note that the set of touch points may end up being empty if the user is not touching the scene.
  std::vector<Eigen::Vector2i> touchPoints = extract_touch_points(bestConnectedComponent, diffRawRaycastInMm);

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
  // Display the touch points.
  cv::Mat touchPointImage = cv::Mat::zeros(m_rows, m_cols, CV_8UC1);
  for(size_t i = 0, iend = touchPoints.size(); i < iend; ++i)
  {
    const Eigen::Vector2i& v = touchPoints[i];
    cv::circle(touchPointImage, cv::Point(v[0], v[1]), 5, cv::Scalar(255), 2);
  }
  cv::imshow("touchPointImage", touchPointImage);
#endif

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
  cv::waitKey(m_debugDelayms);
#endif

  return touchPoints;
}
catch(af::exception&)
{
  // Prevent the touch detector from crashing when tracking is lost.
  return std::vector<Eigen::Vector2i>();
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void TouchDetector::detect_changes()
{
  // Calculate the difference between the raw depth image and the depth raycast.
  m_imageProcessor->calculate_depth_difference(m_thresholdedRawDepth, m_depthRaycast, m_diffRawRaycast);

  // Threshold the difference image to find significant differences between the raw depth image
  // and the depth raycast. Such differences indicate locations in which the scene has changed
  // since it was originally reconstructed, e.g. the locations of moving objects such as hands.
  m_changeMask = *m_diffRawRaycast > m_depthLowerThreshold;

  // Apply a morphological opening operation to the change mask to reduce noise.
  int morphKernelSize = m_morphKernelSize;
  if(morphKernelSize < 3) morphKernelSize = 3;
  if(morphKernelSize % 2 == 0) ++morphKernelSize;
  af::array morphKernel = af::constant(1, morphKernelSize, morphKernelSize);
  m_changeMask = af::erode(m_changeMask, morphKernel);
  m_changeMask = af::dilate(m_changeMask, morphKernel);

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
  // Display the raw depth image and the depth raycast.
  const float mToCm = 100.0f; // the scaling factor needed to convert metres to centimetres
  m_thresholdedRawDepth->UpdateHostFromDevice();
  m_depthRaycast->UpdateHostFromDevice();
  OpenCVUtil::show_scaled_greyscale_figure("Current raw depth from camera in centimetres", m_thresholdedRawDepth->GetData(MEMORYDEVICE_CPU), m_cols, m_rows, OpenCVUtil::ROW_MAJOR, mToCm);
  OpenCVUtil::show_scaled_greyscale_figure("Current depth raycast in centimetres", m_depthRaycast->GetData(MEMORYDEVICE_CPU), m_cols, m_rows, OpenCVUtil::ROW_MAJOR, mToCm);

  // Display the absolute difference between the raw depth image and the depth raycast.
  af::array diffRawRaycastInCm = clamp_to_range(*m_diffRawRaycast * mToCm, 0.0f, 255.0f);
  OpenCVUtil::show_greyscale_figure("Diff image in centimetres", diffRawRaycastInCm.as(u8).host<unsigned char>(), m_cols, m_rows, OpenCVUtil::COL_MAJOR);

  // If this is the first iteration, create a debug window with a number of trackbars that can be used to control the touch detection.
  const std::string debugWindowName = "DebuggingOutputWindow";
  static bool initialised = false;
  if(!initialised)
  {
    cv::namedWindow(debugWindowName, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("depthThresholdmm", debugWindowName, &m_depthLowerThresholdmm, 50);
    cv::createTrackbar("debugDelayms", debugWindowName, &m_debugDelayms, 3000);

    cv::namedWindow("MorphologicalOperatorWindow", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("KernelSize", "MorphologicalOperatorWindow", &m_morphKernelSize, 15);

    initialised = true;
  }

  // Update the relevant variables based on the values of the trackbars.
  m_debugDelayms = cv::getTrackbarPos("debugDelayms", debugWindowName);
  m_depthLowerThresholdmm = cv::getTrackbarPos("depthThresholdmm", debugWindowName);

  // Update the current depth Threshold in meters;
  m_depthLowerThreshold = m_depthLowerThresholdmm / 1000.0f;

  // Display the thresholded image.
  static af::array thresholdedDisplay;
  thresholdedDisplay = m_changeMask * 255.0f;
  OpenCVUtil::show_greyscale_figure(debugWindowName, thresholdedDisplay.as(u8).host<unsigned char>(), m_cols, m_rows, OpenCVUtil::COL_MAJOR);

  m_morphKernelSize = cv::getTrackbarPos("KernelSize", "MorphologicalOperatorWindow");

  // Display the threholded image after applying morphological operations.
  static af::array morphDisplay;
  morphDisplay = m_changeMask * 255.0f;
  OpenCVUtil::show_greyscale_figure("MorphologicalOperatorWindow", morphDisplay.as(u8).host<unsigned char>(), m_cols, m_rows, OpenCVUtil::COL_MAJOR);
#endif
}

std::vector<Eigen::Vector2i> TouchDetector::extract_touch_points(int component, const af::array& diffRawRaycastInMm)
{
  std::vector<Eigen::Vector2i> touchPoints;

  // Binary mask identifying the region with the best candidate.
  static af::array mask;
  mask = m_connectedComponentImage == component;

  // Get the diff values corresponding to the best candidate region.
  static af::array temporaryCandidate;
  temporaryCandidate = diffRawRaycastInMm * mask;

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

      touchPoints.push_back(point);
    }
  }

  return touchPoints;
}

int TouchDetector::pick_best_candidate_component(const af::array& goodCandidates, const af::array& diffRawRaycastInMm)
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
      mask = (m_connectedComponentImage == candidateIds[i]);
      temporaryCandidate = diffRawRaycastInMm * mask;
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
  mask = (m_connectedComponentImage.as(u32) == bestConnectedComponent);
  //af::print("m_connectedComponentImage", m_connectedComponentImage);
  temporaryCandidate = diffRawRaycastInMm * mask;

  // Display the best candidate's difference image.
  static af::array temporaryCandidateDisplay(m_rows, m_cols, u8);
  temporaryCandidateDisplay = temporaryCandidate;
  OpenCVUtil::show_greyscale_figure("bestConnectedComponent", temporaryCandidateDisplay.host<unsigned char>(), m_cols, m_rows, OpenCVUtil::COL_MAJOR);

  static af::array maskDisplay(m_rows, m_cols, u8);
  maskDisplay = mask * 255;
  OpenCVUtil::show_greyscale_figure("maskDisplay", maskDisplay.as(u8).host<unsigned char>(), m_cols, m_rows, OpenCVUtil::COL_MAJOR);
#endif

  return bestConnectedComponent;
}

void TouchDetector::prepare_inputs(const rigging::MoveableCamera_CPtr& camera, const ITMFloatImage_CPtr& rawDepth, const RenderState_CPtr& renderState)
{
  // Make a copy of the raw depth image in which any parts of the scene that are at a distance of > 2m are set to -1.
  // We deliberately ignore parts of the scene that are > 2m away, since although they are picked up by the camera,
  // they are not fused into the scene by InfiniTAM (which has a depth threshold hard-coded into its scene settings).
  // As a result, there will always be large expected differences between the raw and raycasted depth images in those
  // parts of the scene. A 2m threshold is reasonable because we assume that the camera is positioned close to the user
  // and that the user's hand or leg will therefore not extend more than two metres away from the camera position.
  m_imageProcessor->set_on_threshold(rawDepth, ImageProcessor::CO_GREATER, 2.0f, -1.0f, m_thresholdedRawDepth);

  // Generate an orthographic depth raycast of the current scene from the current camera position.
  // As with the raw depth image, the pixel values of this raycast denote depth values in metres.
  // We assume that parts of the scene for which we have no information are far away (at an
  // arbitrarily large depth of 100m).
  const float invalidDepthValue = 100.0f;
  m_depthCalculator->render_depth(
    DepthVisualiser::DT_ORTHOGRAPHIC,
    to_itm(camera->p()),
    to_itm(camera->n()),
    renderState.get(),
    m_settings->sceneParams.voxelSize,
    invalidDepthValue,
    m_depthRaycast
  );
}

af::array TouchDetector::select_candidate_components()
{
  // Calculate the areas of the connected components.
  const int componentCount = af::max<int>(m_connectedComponentImage) + 1;
  af::array componentAreas = af::histogram(m_connectedComponentImage, componentCount);

  // Zero out connected components that are either too small or too large.
  componentAreas -= (componentAreas < m_minimumConnectedComponentAreaThreshold) * componentAreas;
  componentAreas -= (componentAreas > m_maximumConnectedComponentAreaThreshold) * componentAreas;

  // Keep the remaining non-zero components as candidates.
  af::array candidates = af::where(componentAreas);

#ifdef DEBUG_TOUCH_VERBOSE
  af::print("componentAreas", componentAreas);
  if(candidates.elements() > 0) af::print("candidates", candidates);
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

  return candidates;
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

af::array TouchDetector::clamp_to_range(const af::array& arr, float lower, float upper)
{
  static af::array lowerMask, upperMask;
  lowerMask = arr < lower;
  upperMask = arr > upper;

  static af::array arrayCopy;
  arrayCopy = arr - lowerMask * arr;
  arrayCopy = arrayCopy - (upperMask * arrayCopy) + (upperMask * upper);

  return arrayCopy;
}

Vector3f TouchDetector::to_itm(const Eigen::Vector3f& v)
{
  return Vector3f(v[0], v[1], v[2]);
}

}
