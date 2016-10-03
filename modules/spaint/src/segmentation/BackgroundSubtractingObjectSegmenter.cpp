/**
 * spaint: BackgroundSubtractingObjectSegmenter.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "segmentation/BackgroundSubtractingObjectSegmenter.h"

#include <cmath>

#include <boost/serialization/shared_ptr.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ocv/OpenCVUtil.h"
#include "util/CameraPoseConverter.h"

#define DEBUGGING 1

namespace spaint {

//#################### CONSTRUCTORS ####################

BackgroundSubtractingObjectSegmenter::BackgroundSubtractingObjectSegmenter(const View_CPtr& view, const ITMSettings_CPtr& itmSettings, const TouchSettings_Ptr& touchSettings)
: Segmenter(view), m_touchDetector(new TouchDetector(view->depth->noDims, itmSettings, touchSettings))
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void BackgroundSubtractingObjectSegmenter::reset()
{
  m_handAppearanceModel.reset(new ColourAppearanceModel(30, 30));
}

ITMUCharImage_CPtr BackgroundSubtractingObjectSegmenter::segment(const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState) const
{
  // Set up the parameters for the object mask.
  static int componentSizeThreshold = 1000;
  static int objectProbThreshold = 80;

#if DEBUGGING
  // Set up the debugging window for the object mask.
  const std::string debugWindowName = "Object Mask";
  static bool initialised = false;
  if(!initialised)
  {
    cv::namedWindow(debugWindowName, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("componentSizeThreshold", debugWindowName, &componentSizeThreshold, 2000);
    cv::createTrackbar("objectProbThreshold", debugWindowName, &objectProbThreshold, 100);
    initialised = true;
  }
#endif

  // Copy the current colour and depth input images across to the CPU.
  ITMUChar4Image_CPtr rgbInput(m_view->rgb, boost::serialization::null_deleter());
  rgbInput->UpdateHostFromDevice();

  ITMFloatImage_CPtr depthInput(m_view->depth, boost::serialization::null_deleter());
  depthInput->UpdateHostFromDevice();

  // Make the change mask.
  ITMUCharImage_CPtr changeMask = make_change_mask(depthInput, pose, renderState);

  // Make the object mask.
  static cv::Mat1b objectMask = cv::Mat1b::zeros(m_view->rgb->noDims.y, m_view->rgb->noDims.x);
  const Vector4u *rgbPtr = rgbInput->GetData(MEMORYDEVICE_CPU);
  const uchar *changeMaskPtr = changeMask->GetData(MEMORYDEVICE_CPU);
  const int pixelCount = static_cast<int>(rgbInput->dataSize);

  // For each pixel in the current colour input image:
#if WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < pixelCount; ++i)
  {
    // Update the object mask based on whether the pixel is part of the object.
    unsigned char value = 0;
    if(changeMaskPtr[i])
    {
      float objectProb = m_handAppearanceModel ? 1.0f - m_handAppearanceModel->compute_posterior_probability(rgbPtr[i].toVector3()) : 1.0f;

#if 1
      if(objectProb >= objectProbThreshold / 100.0f) value = 255;
#else
      // For debugging purposes
      if(objectProb >= objectProbThreshold / 100.0f) value = (uchar)(objectProb * 255);
#endif
    }

    objectMask.data[i] = value;
  }

  // Find the connected components of the object mask.
  cv::Mat1i ccsImage, stats;
  cv::Mat1d centroids;
  cv::connectedComponentsWithStats(objectMask, ccsImage, stats, centroids);

  // Update the object mask to only contain components over a certain size.
  const int *ccsData = reinterpret_cast<int*>(ccsImage.data);

#if WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < pixelCount; ++i)
  {
    int componentSize = stats(ccsData[i], cv::CC_STAT_AREA);
    if(componentSize < componentSizeThreshold)
    {
      objectMask.data[i] = 0;
    }
 }

#if DEBUGGING
  // Show the debugging window for the object mask.
  cv::imshow(debugWindowName, objectMask);
  cv::waitKey(10);
#endif

  // Convert the object mask to InfiniTAM format and return it.
  std::copy(objectMask.data, objectMask.data + m_view->rgb->dataSize, m_targetMask->GetData(MEMORYDEVICE_CPU));
  return m_targetMask;
}

ITMUChar4Image_Ptr BackgroundSubtractingObjectSegmenter::train(const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState)
{
  // Copy the current colour and depth input images across to the CPU.
  ITMUChar4Image_CPtr rgbInput(m_view->rgb, boost::serialization::null_deleter());
  rgbInput->UpdateHostFromDevice();

  ITMFloatImage_CPtr depthInput(m_view->depth, boost::serialization::null_deleter());
  depthInput->UpdateHostFromDevice();

  // Train a colour appearance model to separate the user's hand from the scene background.
  if(!m_handAppearanceModel) reset();
  m_handAppearanceModel->train(rgbInput, make_hand_mask(depthInput, pose, renderState));

  // Generate a segmented image of the user's hand that can be shown to the user to provide them
  // with interactive feedback about the data that is being used to train the appearance model.
  return m_touchDetector->generate_touch_image(m_view);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

ITMUCharImage_CPtr BackgroundSubtractingObjectSegmenter::make_change_mask(const ITMFloatImage_CPtr& depthInput, const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState) const
{
  // Set up the parameters for the change mask.
  static int centreDistThreshold = 70;            // pixels greater than this percentage distance from the centre of the image will be ignored
  static int componentSizeThreshold = 150;        // components below this size will be ignored
  static int componentSizeThreshold2 = 800;
  static int componentSizeThreshold3 = 1000;
  static int depthEdgeThreshold = 3;              // pixels with values above this will be treated as edges in the gradient magnitude image of the depth raycast
  static int lowerCompactnessThreshold = 50;      // small components whose compactness is less than this percentage will be ignored
  static int lowerDiffThresholdMm = 15;           // pixels whose depth difference (in mm) is less than this will be ignored
  static int lowerDiffThresholdNearEdgesMm = 100; // pixels near depth edges whose depth difference (in mm) is less than this will be ignored
  static int upperDepthThresholdMm = 1000;        // pixels whose live depth value (in mm) is greater than this will be ignored

#if DEBUGGING
  // Set up the debugging window for the change mask.
  const std::string debugWindowName = "Change Mask";
  static bool initialised = false;
  if(!initialised)
  {
    cv::namedWindow(debugWindowName, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("centreDistThreshold", debugWindowName, &centreDistThreshold, 100);
    cv::createTrackbar("componentSizeThreshold", debugWindowName, &componentSizeThreshold, 2000);
    cv::createTrackbar("componentSizeThreshold2", debugWindowName, &componentSizeThreshold2, 2000);
    cv::createTrackbar("componentSizeThreshold3", debugWindowName, &componentSizeThreshold3, 2000);
    cv::createTrackbar("depthEdgeThreshold", debugWindowName, &depthEdgeThreshold, 255);
    cv::createTrackbar("lowerCompactnessThreshold", debugWindowName, &lowerCompactnessThreshold, 100);
    cv::createTrackbar("lowerDiffThresholdMm", debugWindowName, &lowerDiffThresholdMm, 100);
    cv::createTrackbar("lowerDiffThresholdNearEdgesMm", debugWindowName, &lowerDiffThresholdNearEdgesMm, 100);
    cv::createTrackbar("upperDepthThresholdMm", debugWindowName, &upperDepthThresholdMm, 2000);
    initialised = true;
  }
#endif

  // Run the touch detector.
  rigging::MoveableCamera_CPtr camera(new rigging::SimpleCamera(CameraPoseConverter::pose_to_camera(pose)));
  m_touchDetector->determine_touch_points(camera, depthInput, renderState);

  // Get a thresholded version of the live depth image.
  ITMFloatImage_CPtr thresholdedRawDepth = m_touchDetector->get_thresholded_raw_depth();
  thresholdedRawDepth->UpdateHostFromDevice();
  const float *thresholdedRawDepthPtr = thresholdedRawDepth->GetData(MEMORYDEVICE_CPU);

  // Get the depth raycast of the scene.
  ITMFloatImage_CPtr depthRaycast = m_touchDetector->get_depth_raycast();
  depthRaycast->UpdateHostFromDevice();
  const float *depthRaycastPtr = depthRaycast->GetData(MEMORYDEVICE_CPU);

  // Compute a dilated, thresholded version of the gradient magnitude of the depth raycast.
  const int width = depthRaycast->noDims.x, height = depthRaycast->noDims.y;
  cv::Mat1b cvDepthRaycast = OpenCVUtil::make_greyscale_image(depthRaycastPtr, width, height, OpenCVUtil::ROW_MAJOR, 100.0f);
  cv::Mat gradX, gradY, absGradX, absGradY, grad, thresholdedGrad, dilatedThresholdedGrad;
  cv::Sobel(cvDepthRaycast, gradX, CV_16S, 1, 0, 3);
  cv::convertScaleAbs(gradX, absGradX);
  cv::Sobel(cvDepthRaycast, gradY, CV_16S, 0, 1, 3);
  cv::convertScaleAbs(gradY, absGradY);
  cv::addWeighted(absGradX, 0.5, absGradY, 0.5, 0, grad);
  cv::threshold(grad, thresholdedGrad, depthEdgeThreshold, 255.0, cv::THRESH_BINARY);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
  cv::dilate(thresholdedGrad, dilatedThresholdedGrad, kernel);

  // Get the difference between the live depth image and the depth raycast of the scene.
  ITMFloatImage_Ptr diffRawRaycast = m_touchDetector->get_diff_raw_raycast();
  const float *diffRawRaycastPtr = diffRawRaycast->GetData(MEMORYDEVICE_CPU);

  // Make an initial change mask, starting from the whole image and filtering out pixels based on some simple criteria.
  static ITMUCharImage_Ptr changeMask(new ITMUCharImage(Vector2i(width, height), true, true));
  uchar *changeMaskPtr = changeMask->GetData(MEMORYDEVICE_CPU);
  const double halfWidth = width / 2.0, halfHeight = height / 2.0;
  const int pixelCount = static_cast<int>(changeMask->dataSize);

#if WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < pixelCount; ++i)
  {
    // Every pixel starts off as part of the change mask.
    changeMaskPtr[i] = 255;

    // If the live depth value for the pixel is invalid, remove it from the change mask (it can't form part of the final
    // mask that we will use for object reconstruction, since without depth it can't be fused).
    if(thresholdedRawDepthPtr[i] == -1.0f)
    {
      changeMaskPtr[i] = 0;
      continue;
    }

    // If the depth raycast value for the pixel is invalid, remove it from the change mask (without a depth raycast value,
    // we can't do background subtraction).
    if(fabs(depthRaycastPtr[i] - m_touchDetector->invalid_depth_value()) < 1e-3f)
    {
      changeMaskPtr[i] = 0;
      continue;
    }

    // If the live depth value for the pixel is too large, remove it from the change mask (the depth gets increasingly
    // unreliable as we get further away from the sensor, so this helps us avoid corrupting our mask with noise).
    if(thresholdedRawDepthPtr[i] * 1000.0f > upperDepthThresholdMm)
    {
      changeMaskPtr[i] = 0;
      continue;
    }

    // If the pixel is close to the corners of the image, remove it from the change mask (the depth gets increasingly
    // unreliable as we get further away from the centre of the image).
    const int x = i % width, y = i / width;
    const double xDist = fabs(x - halfWidth), yDist = fabs(y - halfHeight);
    const double centreDist = sqrt((xDist * xDist + yDist * yDist) / (halfWidth * halfWidth + halfHeight * halfHeight));
    if(static_cast<int>(centreDist * 100) > centreDistThreshold)
    {
      changeMaskPtr[i] = 0;
      continue;
    }

    // If the difference between the pixel's values in the live depth image and the depth raycast is quite small,
    // remove it from the change mask (this helps exclude minor differences that are caused by sensor noise).
    const float diffRawRaycastMm = diffRawRaycastPtr[i] * 1000.0f;
    if(diffRawRaycastMm < lowerDiffThresholdMm)
    {
      changeMaskPtr[i] = 0;
      continue;
    }

    // If the pixel is close to an edge in the depth raycast and there isn't a fairly significant difference between
    // its values in the live depth image and the depth raycast, remove it from the change mask (we insist on a larger
    // difference than normal near depth raycast edges because depth values tend to be unreliable along such boundaries).
    if(dilatedThresholdedGrad.data[i] && diffRawRaycastMm < lowerDiffThresholdNearEdgesMm)
    {
      changeMaskPtr[i] = 0;
      continue;
    }
  }

  // Copy the change mask across to an OpenCV image.
  static cv::Mat1b cvChangeMask = cv::Mat1b::zeros(height, width);

#if WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < pixelCount; ++i)
  {
    cvChangeMask.data[i] = changeMaskPtr[i];
  }

  // Find the connected components of the change mask.
  cv::Mat1i ccsImage, stats;
  cv::Mat1d centroids;
  cv::connectedComponentsWithStats(cvChangeMask, ccsImage, stats, centroids);

  // Update the change mask to only contain components over a certain size.
  const int *ccsData = reinterpret_cast<int*>(ccsImage.data);

#if WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < pixelCount; ++i)
  {
    int componentSize = stats(ccsData[i], cv::CC_STAT_AREA);
    if(componentSize < componentSizeThreshold)
    {
      cvChangeMask.data[i] = 0;
      changeMaskPtr[i] = 0;
    }
  }

  // Find the contours in the change mask.
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(cvChangeMask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  // Add any small contours that are not sufficiently compact to a bad contour mask, and determine the largest remaining contour and its area.
  cv::Mat1b badContourMask = cv::Mat1b::zeros(cvChangeMask.size());
  int largestContour = -1;
  double largestContourArea = 0.0;
  for(int i = 0, size = static_cast<int>(contours.size()); i < size; ++i)
  {
    // Calculate the compactness of the contour.
    double area = cv::contourArea(contours[i]);
    size_t perimeter = contours[i].size();
    double compactness = 4 * M_PI * area / (perimeter * perimeter);

    // If the contour is small and not sufficiently compact, add it to the bad contour mask.
    // Otherwise, update the largest contour and its area as necessary.
    if(static_cast<int>(area) < componentSizeThreshold2 && static_cast<int>(CLAMP(ROUND(compactness * 100), 0, 100)) < lowerCompactnessThreshold)
    {
      cv::drawContours(badContourMask, contours, i, cv::Scalar(255), cv::FILLED);
    }
    else if(area > largestContourArea)
    {
      largestContour = i;
      largestContourArea = area;
    }
  }

  // If there is a largest contour:
  if(largestContour != -1)
  {
    // Make a 200% bounding box around the largest contour.
    cv::Rect largestContourRect = cv::boundingRect(contours[largestContour]);
    largestContourRect.x -= largestContourRect.width / 2;
    largestContourRect.y -= largestContourRect.height / 2;
    largestContourRect.width *= 2;
    largestContourRect.height *= 2;

    // Add any relatively small contours (other than the largest contour itself) that are not completely within this box to the bad contour mask.
    std::set<int> componentsToRemove;
    for(int i = 0, size = static_cast<int>(contours.size()); i < size; ++i)
    {
      // If the contour is the largest, avoid removing it.
      if(i == largestContour) continue;

      // If the contour is sufficiently large, avoid removing it.
      if(cv::contourArea(contours[i]) > componentSizeThreshold3) continue;

      // Otherwise, if the contour is not within the 200% bounding box around the largest contour, add it to the bad contour mask.
      cv::Rect componentRect = cv::boundingRect(contours[i]);
      if(!largestContourRect.contains(componentRect.tl()) || !largestContourRect.contains(componentRect.br()))
      {
        cv::drawContours(badContourMask, contours, i, cv::Scalar(255), cv::FILLED);
      }
    }
  }

  // Remove any bad contours from the change mask.
#if WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < pixelCount; ++i)
  {
    if(badContourMask.data[i])
    {
      cvChangeMask.data[i] = 0;
      changeMaskPtr[i] = 0;
    }
  }

#if DEBUGGING
  // Show the debugging window for the change mask.
  OpenCVUtil::show_greyscale_figure(debugWindowName, changeMask->GetData(MEMORYDEVICE_CPU), width, height, OpenCVUtil::ROW_MAJOR);
#endif

  return changeMask;
}

ITMUCharImage_CPtr BackgroundSubtractingObjectSegmenter::make_hand_mask(const ITMFloatImage_CPtr& depthInput, const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState) const
{
  rigging::MoveableCamera_CPtr camera(new rigging::SimpleCamera(CameraPoseConverter::pose_to_camera(pose)));
  m_touchDetector->determine_touch_points(camera, depthInput, renderState);
  return m_touchDetector->get_touch_mask();
}

}
