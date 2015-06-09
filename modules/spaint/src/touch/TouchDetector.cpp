/**
 * spaint: TouchDetector.cpp
 */

#include "touch/TouchDetector.h"

#include <tvgutil/ArgUtil.h>

#include "imageprocessing/cpu/ImageProcessor_CPU.h"
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
  m_connectedComponentImage(imgSize.y, imgSize.x, u32),
  m_debugDelayMs(30),
  m_debuggingOutputWindowName("DebuggingOutputWindow"),
  m_depthRaycast(new ITMFloatImage(imgSize, true, true)),
  m_diffRawRaycast(new af::array(imgSize.y, imgSize.x, f32)),
  m_imageHeight(imgSize.y),
  m_imageWidth(imgSize.x),
  m_lowerDepthThresholdMm(10),
  m_morphKernelSize(5),
  m_settings(settings),
  m_thresholdedRawDepth(new ITMFloatImage(imgSize, true, true))
{
  // Set up the depth visualiser and image processor.
  if(settings->deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    m_depthVisualiser.reset(new DepthVisualiser_CUDA);
    m_imageProcessor.reset(new ImageProcessor_CUDA);
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU to false if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    m_depthVisualiser.reset(new DepthVisualiser_CPU);
    m_imageProcessor.reset(new ImageProcessor_CPU);
  }

  // Set the maximum and minimum areas (in pixels) of a connected change component for it to be considered a candidate touch interaction.
  // The thresholds are set relative to the image area to avoid depending on a particular size of image.
  const int imageArea = m_imageHeight * m_imageWidth;
  const float minCandidateFraction = 0.01f; // i.e. 1% of the image
  const float maxCandidateFraction = 0.2f;  // i.e. 20% of the image
  m_minCandidateArea = static_cast<int>(minCandidateFraction * imageArea);
  m_maxCandidateArea = static_cast<int>(maxCandidateFraction * imageArea);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

std::vector<Eigen::Vector2i> TouchDetector::determine_touch_points(const rigging::MoveableCamera_CPtr& camera, const ITMFloatImage_CPtr& rawDepth, const RenderState_CPtr& renderState)
try
{
#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
  // TODO
  //process_debug_windows();
#endif

  // Prepare a thresholded version of the raw depth image and a depth raycast ready for change detection.
  prepare_inputs(camera, rawDepth, renderState);

  // Detect changes in the scene with respect to the reconstructed model.
  detect_changes();

  // Make a connected-component image from the change mask.
  m_connectedComponentImage = af::regions(m_changeMask);

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
  // Display the connected components.
  int componentCount = af::max<int>(m_connectedComponentImage) + 1;
  af::array connectedComponentDebugImage = m_connectedComponentImage * (255.0f / componentCount);
  OpenCVUtil::show_greyscale_figure("connectedComponentDebugImage", connectedComponentDebugImage.as(u8).host<unsigned char>(), m_imageWidth, m_imageHeight, OpenCVUtil::COL_MAJOR);
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
  cv::Mat touchPointDebugImage = cv::Mat::zeros(m_imageHeight, m_imageWidth, CV_8UC1);
  for(size_t i = 0, size = touchPoints.size(); i < size; ++i)
  {
    const Eigen::Vector2i& p = touchPoints[i];
    cv::circle(touchPointDebugImage, cv::Point(p[0], p[1]), 5, cv::Scalar(255), 2);
  }
  cv::imshow("touchPointDebugImage", touchPointDebugImage);

  // Wait for the specified number of milliseconds (or until a key is pressed).
  cv::waitKey(m_debugDelayMs);
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

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
  // Display the raw depth image and the depth raycast.
  const float mToCm = 100.0f; // the scaling factor needed to convert metres to centimetres
  m_thresholdedRawDepth->UpdateHostFromDevice();
  m_depthRaycast->UpdateHostFromDevice();
  OpenCVUtil::show_scaled_greyscale_figure("Current raw depth from camera in centimetres", m_thresholdedRawDepth->GetData(MEMORYDEVICE_CPU), m_imageWidth, m_imageHeight, OpenCVUtil::ROW_MAJOR, mToCm);
  OpenCVUtil::show_scaled_greyscale_figure("Current depth raycast in centimetres", m_depthRaycast->GetData(MEMORYDEVICE_CPU), m_imageWidth, m_imageHeight, OpenCVUtil::ROW_MAJOR, mToCm);

  // Display the absolute difference between the raw depth image and the depth raycast.
  af::array diffRawRaycastInCm = clamp_to_range(*m_diffRawRaycast * mToCm, 0.0f, 255.0f);
  OpenCVUtil::show_greyscale_figure("Diff image in centimetres", diffRawRaycastInCm.as(u8).host<unsigned char>(), m_imageWidth, m_imageHeight, OpenCVUtil::COL_MAJOR);
#endif

  // Threshold the difference image to find significant differences between the raw depth image
  // and the depth raycast. Such differences indicate locations in which the scene has changed
  // since it was originally reconstructed, e.g. the locations of moving objects such as hands.
  m_changeMask = *m_diffRawRaycast > (m_lowerDepthThresholdMm / 1000.0f);

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
  // Display the change mask.
  OpenCVUtil::show_greyscale_figure(m_debuggingOutputWindowName, (m_changeMask * 255.0f).as(u8).host<unsigned char>(), m_imageWidth, m_imageHeight, OpenCVUtil::COL_MAJOR);
#endif

  // Apply a morphological opening operation to the change mask to reduce noise.
  int morphKernelSize = m_morphKernelSize;
  if(morphKernelSize < 3) morphKernelSize = 3;
  if(morphKernelSize % 2 == 0) ++morphKernelSize;
  af::array morphKernel = af::constant(1, morphKernelSize, morphKernelSize);
  m_changeMask = af::erode(m_changeMask, morphKernel);
  m_changeMask = af::dilate(m_changeMask, morphKernel);

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
  // If this is the first iteration, create debugging windows with trackbars that can be used to control the touch detection.
  const std::string morphologicalOperatorWindowName = "MorphologicalOperatorWindow";
  static bool initialised = false;
  if(!initialised)
  {
    cv::namedWindow(m_debuggingOutputWindowName, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("lowerDepthThresholdMm", m_debuggingOutputWindowName, &m_lowerDepthThresholdMm, 50);
    cv::createTrackbar("debugDelayMs", m_debuggingOutputWindowName, &m_debugDelayMs, 3000);

    cv::namedWindow(morphologicalOperatorWindowName, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("KernelSize", morphologicalOperatorWindowName, &m_morphKernelSize, 15);

    initialised = true;
  }

  // Update the relevant variables based on the values of the trackbars.
  m_debugDelayMs = cv::getTrackbarPos("debugDelayMs", m_debuggingOutputWindowName);
  m_lowerDepthThresholdMm = cv::getTrackbarPos("lowerDepthThresholdMm", m_debuggingOutputWindowName);
  m_morphKernelSize = cv::getTrackbarPos("KernelSize", morphologicalOperatorWindowName);

  // Display the thresholded image after applying morphological operations.
  OpenCVUtil::show_greyscale_figure(morphologicalOperatorWindowName, (m_changeMask * 255.0f).as(u8).host<unsigned char>(), m_imageWidth, m_imageHeight, OpenCVUtil::COL_MAJOR);
#endif
}

std::vector<Eigen::Vector2i> TouchDetector::extract_touch_points(int component, const af::array& diffRawRaycastInMm)
{
  // Determine the component's binary mask and difference image.
  af::array mask = m_connectedComponentImage == component;
  af::array diffImage = diffRawRaycastInMm * mask;

  // Quantize the intensites in the difference image to 32 levels (from a starting point of 256 levels).
  diffImage = (diffImage / 8).as(u8) * 8;

  // Threshold the difference image, keeping only parts that are close to the surface.
  const int upperDepthThresholdMm = m_lowerDepthThresholdMm + 10;
  diffImage = (diffImage > m_lowerDepthThresholdMm) && (diffImage < upperDepthThresholdMm);

  // Spatially quantize the difference image by resizing it to 50% of its current size. This has the effect of reducing the eventual number of touch points.
  const float scaleFactor = 0.5f;
  diffImage = af::resize(scaleFactor, diffImage);

  // Make a 1D array whose elements denote the pixels at which the user is touching the scene. Each element is a column-major index into the resized difference image.
  af::array touchIndicesImage = af::where(diffImage);

  // If there are too few touch indices, assume the user is not touching the scene in a meaningful way and early out.
  const float touchAreaLowerThreshold = 0.0001f * m_imageWidth * m_imageHeight;
  if(touchIndicesImage.elements() <= touchAreaLowerThreshold) return std::vector<Eigen::Vector2i>();

  // Otherwise, convert the touch indices to touch points and return them.
  const int resizedDiffHeight = static_cast<int>(m_imageHeight * scaleFactor);
  const int *touchIndices = touchIndicesImage.as(s32).host<int>();
  std::vector<Eigen::Vector2i> touchPoints;
  for(int i = 0, touchPointCount = touchIndicesImage.elements(); i < touchPointCount; ++i)
  {
    Eigen::Vector2f point(touchIndices[i] / resizedDiffHeight, touchIndices[i] % resizedDiffHeight);
    touchPoints.push_back((point / scaleFactor).cast<int>());
  }

  return touchPoints;
}

int TouchDetector::pick_best_candidate_component(const af::array& candidateComponents, const af::array& diffRawRaycastInMm)
{
  const int *candidateIDs = candidateComponents.host<int>();
  const int candidateCount = candidateComponents.dims(0);

  int bestCandidateID = -1;
  af::array mask;

  if(candidateCount == 1)
  {
    // If there is only one candidate, then by definition it's the best candidate.
    bestCandidateID = candidateIDs[0];
  }
  else
  {
    // Otherwise, select the candidate that is closest to a surface.
    std::vector<float> meanDistances(candidateCount);
    for(int i = 0; i < candidateCount; ++i)
    {
      mask = m_connectedComponentImage == candidateIDs[i];
      meanDistances[i] = af::mean<float>(diffRawRaycastInMm * mask);
    }
    size_t minIndex = tvgutil::ArgUtil::argmin(meanDistances);
    bestCandidateID = candidateIDs[minIndex];
  }

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
  // Display the best candidate's mask and difference image.
  mask = m_connectedComponentImage.as(s32) == bestCandidateID;
  OpenCVUtil::show_greyscale_figure("bestCandidateMask", (mask * 255).as(u8).host<unsigned char>(), m_imageWidth, m_imageHeight, OpenCVUtil::COL_MAJOR);
  OpenCVUtil::show_greyscale_figure("bestCandidateDiff", (diffRawRaycastInMm * mask).as(u8).host<unsigned char>(), m_imageWidth, m_imageHeight, OpenCVUtil::COL_MAJOR);
#endif

  return bestCandidateID;
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
  m_depthVisualiser->render_depth(
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
  componentAreas -= (componentAreas < m_minCandidateArea) * componentAreas;
  componentAreas -= (componentAreas > m_maxCandidateArea) * componentAreas;

  // Keep the remaining non-zero components as candidates.
  af::array candidates = af::where(componentAreas).as(s32);

#ifdef DEBUG_TOUCH_VERBOSE
  af::print("componentAreas", componentAreas);
  if(candidates.elements() > 0) af::print("candidates", candidates);
#endif

#if defined(WITH_OPENCV) && defined(DEBUG_TOUCH_DISPLAY)
  static bool initialised = false;

  // Initialise the OpenCV Trackbar if it's the first iteration.
  if(!initialised)
  {
    cv::createTrackbar("minCandidateArea", m_debuggingOutputWindowName, &m_minCandidateArea, m_imageHeight * m_imageWidth);
    cv::createTrackbar("maxCandidateArea", m_debuggingOutputWindowName, &m_maxCandidateArea, m_imageHeight * m_imageWidth);
  }
  m_minCandidateArea = cv::getTrackbarPos("minCandidateArea", m_debuggingOutputWindowName);
  m_maxCandidateArea = cv::getTrackbarPos("maxCandidateArea", m_debuggingOutputWindowName);

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
