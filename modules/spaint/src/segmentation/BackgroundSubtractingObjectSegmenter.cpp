/**
 * spaint: BackgroundSubtractingObjectSegmenter.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "segmentation/BackgroundSubtractingObjectSegmenter.h"

#include <cmath>

#include <boost/serialization/shared_ptr.hpp>

#include "ocv/OpenCVUtil.h"
#include "util/CameraPoseConverter.h"

#define DEBUGGING 1

namespace spaint {

//#################### CONSTRUCTORS ####################

BackgroundSubtractingObjectSegmenter::BackgroundSubtractingObjectSegmenter(const View_CPtr& view, const Settings_CPtr& itmSettings, const TouchSettings_Ptr& touchSettings)
: Segmenter(view), m_touchDetector(new TouchDetector(view->depth->noDims, itmSettings, touchSettings))
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void BackgroundSubtractingObjectSegmenter::reset()
{
  m_handAppearanceModel.reset(new ColourAppearanceModel(30, 30));
}

ITMUCharImage_CPtr BackgroundSubtractingObjectSegmenter::segment(const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState) const
{
  // Set up the segmentation parameters.
  static int handComponentSizeThreshold = 100;
  static int objectComponentSizeThreshold = 1000;
  static int objectProbThreshold = 80;
  static int removeSmallHandComponents = 1;

#if DEBUGGING
  // Set up the debugging window for the object mask.
  const std::string debugWindowName = "Object Mask";
  static bool initialised = false;
  if(!initialised)
  {
    cv::namedWindow(debugWindowName, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("handComponentSizeThreshold", debugWindowName, &handComponentSizeThreshold, 200);
    cv::createTrackbar("objectComponentSizeThreshold", debugWindowName, &objectComponentSizeThreshold, 2000);
    cv::createTrackbar("objectProbThreshold", debugWindowName, &objectProbThreshold, 100);
    cv::createTrackbar("removeSmallHandComponents", debugWindowName, &removeSmallHandComponents, 1);
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

  // Make the hand mask.
  static cv::Mat1b handMask = cv::Mat1b::zeros(m_view->rgb->noDims.y, m_view->rgb->noDims.x);
  const Vector4u *rgbPtr = rgbInput->GetData(MEMORYDEVICE_CPU);
  const uchar *changeMaskPtr = changeMask->GetData(MEMORYDEVICE_CPU);
  const int pixelCount = static_cast<int>(rgbInput->dataSize);

  // For each pixel in the current colour input image:
#if WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < pixelCount; ++i)
  {
    // Update the hand mask based on whether the pixel is part of the hand.
    unsigned char value = 0;
    if(changeMaskPtr[i])
    {
      float handProb = m_handAppearanceModel ? m_handAppearanceModel->compute_posterior_probability(rgbPtr[i].toVector3()) : 0.0f;
      int handProbThreshold = 100 - objectProbThreshold;

#if 1
      if(handProb >= handProbThreshold / 100.0f) value = 255;
#else
      // For debugging purposes
      if(handProb >= handProbThreshold / 100.0f) value = (uchar)(handProb * 255);
#endif
    }

    handMask.data[i] = value;
  }

  // If desired, update the hand mask to only contain components over a certain size.
  if(removeSmallHandComponents)
  {
    remove_small_components(handMask, handComponentSizeThreshold);
  }

  // Make the object mask.
  static cv::Mat1b objectMask = cv::Mat1b::zeros(m_view->rgb->noDims.y, m_view->rgb->noDims.x);

  // Set the object mask to the difference between the change mask and the hand mask.
#if WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < pixelCount; ++i)
  {
    objectMask.data[i] = changeMaskPtr[i] && !handMask.data[i] ? 255 : 0;
  }

  // Update the object mask to only contain components over a certain size.
  remove_small_components(objectMask, objectComponentSizeThreshold);

#if DEBUGGING
  // Show the debugging window for the object mask.
  cv::imshow(debugWindowName, objectMask);
  cv::waitKey(10);
#endif

  // Convert the object mask to InfiniTAM format and return it.
  std::copy(objectMask.data, objectMask.data + m_view->rgb->dataSize, m_targetMask->GetData(MEMORYDEVICE_CPU));
  return m_targetMask;
}

ITMUChar4Image_CPtr BackgroundSubtractingObjectSegmenter::train(const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState)
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
  static int depthEdgeThreshold = 3;              // pixels with values above this will be treated as edges in the gradient magnitude image of the depth raycast
  static int lowerCompactnessThreshold = 50;      // small components whose compactness is less than this percentage will be ignored
  static int lowerDiffThresholdMm = 15;           // pixels whose depth difference (in mm) is less than this will be ignored
  static int lowerDiffThresholdNearEdgesMm = 100; // pixels near depth edges whose depth difference (in mm) is less than this will be ignored
  static int maxContourSizeForBox = 1000;         // contours that are at most this size will be subjected to a box test
  static int maxContourSizeForCompactness = 800;  // contours that are at most this size will be subjected to a compactness test
  static int maxIntraClusterDepthDiffMm = 5;      // the maximum difference in depth to allow between pixels within the same cluster
  static int minClusterSize = 250;                // clusters of pixels (by depth) that are less than this size will be ignored
  static int minComponentSize = 150;              // components below this size will be ignored
  static int upperDepthThresholdMm = 1000;        // pixels whose live depth value (in mm) is greater than this will be ignored

#if DEBUGGING
  // Set up the debugging window for the change mask.
  const std::string debugWindowName = "Change Mask";
  static bool initialised = false;
  if(!initialised)
  {
    cv::namedWindow(debugWindowName, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("centreDistThreshold", debugWindowName, &centreDistThreshold, 100);
    cv::createTrackbar("depthEdgeThreshold", debugWindowName, &depthEdgeThreshold, 255);
    cv::createTrackbar("lowerCompactnessThreshold", debugWindowName, &lowerCompactnessThreshold, 100);
    cv::createTrackbar("lowerDiffThresholdMm", debugWindowName, &lowerDiffThresholdMm, 100);
    cv::createTrackbar("lowerDiffThresholdNearEdgesMm", debugWindowName, &lowerDiffThresholdNearEdgesMm, 100);
    cv::createTrackbar("maxContourSizeForBox", debugWindowName, &maxContourSizeForBox, 2000);
    cv::createTrackbar("maxContourSizeForCompactness", debugWindowName, &maxContourSizeForCompactness, 2000);
    cv::createTrackbar("maxIntraClusterDepthDiffMm", debugWindowName, &maxIntraClusterDepthDiffMm, 20);
    cv::createTrackbar("minClusterSize", debugWindowName, &minClusterSize, 1000);
    cv::createTrackbar("minComponentSize", debugWindowName, &minComponentSize, 2000);
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
  cv::Mat gradX, gradY, absGradX, absGradY, grad, depthEdges, dilatedDepthEdges;
  cv::Sobel(cvDepthRaycast, gradX, CV_16S, 1, 0, 3);
  cv::convertScaleAbs(gradX, absGradX);
  cv::Sobel(cvDepthRaycast, gradY, CV_16S, 0, 1, 3);
  cv::convertScaleAbs(gradY, absGradY);
  cv::addWeighted(absGradX, 0.5, absGradY, 0.5, 0, grad);
  cv::threshold(grad, depthEdges, depthEdgeThreshold, 255.0, cv::THRESH_BINARY);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
  cv::dilate(depthEdges, dilatedDepthEdges, kernel);

  // Get the difference between the live depth image and the depth raycast of the scene.
  ITMFloatImage_CPtr diffRawRaycast = m_touchDetector->get_diff_raw_raycast();
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
    if(dilatedDepthEdges.data[i] && diffRawRaycastMm < lowerDiffThresholdNearEdgesMm)
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
    if(componentSize < minComponentSize)
    {
      cvChangeMask.data[i] = 0;
      changeMaskPtr[i] = 0;
    }
  }

  // Find the contours in the change mask.
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(cvChangeMask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  // Divide the contours into three sets:
  // - bad contours (small and not compact)
  // - large contours
  // - small contours (small and compact)
  std::set<int> badContours, largeContours, smallContours;

  int largestContour = -1;
  double largestContourArea = 0.0;

  for(int i = 0, size = static_cast<int>(contours.size()); i < size; ++i)
  {
    // Calculate the compactness of the contour.
    double area = cv::contourArea(contours[i]);
    size_t perimeter = contours[i].size();
    double compactness = 4 * M_PI * area / (perimeter * perimeter);

    if(static_cast<int>(area) <= maxContourSizeForCompactness && static_cast<int>(CLAMP(ROUND(compactness * 100), 0, 100)) < lowerCompactnessThreshold)
    {
      // If the contour is small and not sufficiently compact, add it to the bad contours set.
      badContours.insert(i);
    }
    else
    {
      // Otherwise, add the contour to the large or small contours set based on its size,
      // and update the largest contour and its area as necessary.
      (area >= maxContourSizeForBox ? largeContours : smallContours).insert(i);

      if(area > largestContourArea)
      {
        largestContour = i;
        largestContourArea = area;
      }
    }
  }

  // If there is a largest contour, make sure that it is in the large contours set rather than the small contours one.
  // This has the effect of making sure that the large contours set is never empty.
  if(largestContour != -1)
  {
    largeContours.insert(largestContour);
    smallContours.erase(largestContour);
  }

  // Find any remaining small contours that are not contained within a 200% bounding box around one of the large contours.
  for(std::set<int>::const_iterator it = largeContours.begin(), iend = largeContours.end(); it != iend; ++it)
  {
    // Make a 200% bounding box around the current large contour.
    cv::Rect largeContourRect = cv::boundingRect(contours[*it]);
    largeContourRect.x -= largeContourRect.width / 2;
    largeContourRect.y -= largeContourRect.height / 2;
    largeContourRect.width *= 2;
    largeContourRect.height *= 2;

    // For each remaining small contour:
    for(std::set<int>::const_iterator jt = smallContours.begin(), jend = smallContours.end(); jt != jend; /* no-op */)
    {
      // If the small contour is within the large contour's box, remove it from the small contours set.
      cv::Rect smallContourRect = cv::boundingRect(contours[*jt]);
      if(largeContourRect.contains(smallContourRect.tl()) && largeContourRect.contains(smallContourRect.br()))
      {
        smallContours.erase(jt++);
      }
      else ++jt;
    }
  }

  // Add any remaining small contours to the bad contours set.
  std::copy(smallContours.begin(), smallContours.end(), std::inserter(badContours, badContours.begin()));

  // Make a mask containing all of the bad contours.
  cv::Mat1b badContourMask = cv::Mat1b::zeros(cvChangeMask.size());
  for(std::set<int>::const_iterator it = badContours.begin(), iend = badContours.end(); it != iend; ++it)
  {
    cv::drawContours(badContourMask, contours, *it, cv::Scalar(255), cv::FILLED);
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

  // Cluster the pixels in the change mask by depth, and discard clusters that are below a certain size.
  std::multimap<float,int> depthToPixels;
  for(int i = 0; i < pixelCount; ++i)
  {
    if(changeMaskPtr[i])
    {
      depthToPixels.insert(std::make_pair(thresholdedRawDepthPtr[i], i));
    }
  }

  std::vector<std::vector<int> > clusters;
  boost::optional<float> lastDepth;
  for(std::multimap<float,int>::const_iterator it = depthToPixels.begin(), iend = depthToPixels.end(); it != iend; ++it)
  {
    float depth = it->first;
    if(!lastDepth || static_cast<int>(ROUND((depth - *lastDepth) * 1000)) > maxIntraClusterDepthDiffMm)
    {
      clusters.push_back(std::vector<int>());
    }
    lastDepth = depth;

    clusters.back().push_back(it->second);
  }

  for(size_t i = 0, clusterCount = clusters.size(); i < clusterCount; ++i)
  {
    const std::vector<int>& cluster = clusters[i];
    const size_t clusterSize = cluster.size();

    if(clusterSize < minClusterSize)
    {
      for(size_t j = 0; j < clusterSize; ++j)
      {
        cvChangeMask.data[cluster[j]] = 0;
        changeMaskPtr[cluster[j]] = 0;
      }
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

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

void BackgroundSubtractingObjectSegmenter::remove_small_components(cv::Mat1b& mask, int minimumComponentSize)
{
  // Find the connected components of the mask.
  cv::Mat1i ccsImage, stats;
  cv::Mat1d centroids;
  cv::connectedComponentsWithStats(mask, ccsImage, stats, centroids);

  // Update the mask to only contain components over a certain size.
  const int *ccsData = reinterpret_cast<int*>(ccsImage.data);
  const int pixelCount = mask.rows * mask.cols;

#if WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < pixelCount; ++i)
  {
    int componentSize = stats(ccsData[i], cv::CC_STAT_AREA);
    if(componentSize < minimumComponentSize)
    {
      mask.data[i] = 0;
    }
  }
}

}
