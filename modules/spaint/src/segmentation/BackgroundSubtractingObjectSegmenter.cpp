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

namespace spaint {

//#################### CONSTRUCTORS ####################

BackgroundSubtractingObjectSegmenter::BackgroundSubtractingObjectSegmenter(const View_CPtr& view,
                                                                           const ITMSettings_CPtr& itmSettings,
                                                                           const TouchSettings_Ptr& touchSettings)
: Segmenter(view), m_touchDetector(new TouchDetector(view->depth->noDims, itmSettings, touchSettings))
{
  reset();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void BackgroundSubtractingObjectSegmenter::reset()
{
  m_handAppearanceModel.reset(new ColourAppearanceModel(30, 30));
}

ITMUCharImage_CPtr BackgroundSubtractingObjectSegmenter::segment(const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState) const
{
  // TEMPORARY: Debugging controls.
  static bool initialised = false;
  static int closingSize = 15;
  static int componentSizeThreshold = 1000;
  static int lowerDepthThresholdMm = 100;
  static int objectProbThreshold = 80;
  static int useClosing = 1;
  static int useOnlyLargest = 0;
  static int useOpening = 0;
  const std::string debugWindowName = "Debug";
  if(!initialised)
  {
    cv::namedWindow(debugWindowName, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("closingSize", debugWindowName, &closingSize, 50);
    cv::createTrackbar("componentSizeThreshold", debugWindowName, &componentSizeThreshold, 2000);
    cv::createTrackbar("lowerDepthThresholdMm", debugWindowName, &lowerDepthThresholdMm, 100);
    cv::createTrackbar("objectProbThreshold", debugWindowName, &objectProbThreshold, 100);
    cv::createTrackbar("useClosing", debugWindowName, &useClosing, 1);
    cv::createTrackbar("useOnlyLargest", debugWindowName, &useOnlyLargest, 1);
    cv::createTrackbar("useOpening", debugWindowName, &useOpening, 1);
    initialised = true;
  }

  // If the user has not yet trained a hand appearance model, early out.
  if(!m_handAppearanceModel) return ITMUCharImage_Ptr();

  // Copy the current colour and depth input images across to the CPU.
  ITMUChar4Image_CPtr rgbInput(m_view->rgb, boost::serialization::null_deleter());
  rgbInput->UpdateHostFromDevice();

  ITMFloatImage_CPtr depthInput(m_view->depth, boost::serialization::null_deleter());
  depthInput->UpdateHostFromDevice();

  // Update the lower depth threshold for the touch detector.
  m_touchDetector->set_lower_depth_threshold_mm(lowerDepthThresholdMm);

  // Make the change mask and object mask images.
  ITMUCharImage_CPtr changeMask = make_change_mask(depthInput, pose, renderState);
#if 0
  static cv::Mat1b cvObjectMask = cv::Mat1b::zeros(m_view->rgb->noDims.y, m_view->rgb->noDims.x);

  // For each pixel in the current colour input image:
  const Vector4u *rgbPtr = rgbInput->GetData(MEMORYDEVICE_CPU);
  const uchar *changeMaskPtr = changeMask->GetData(MEMORYDEVICE_CPU);
  for(size_t i = 0, size = rgbInput->dataSize; i < size; ++i)
  {
    // Update the object mask based on whether the pixel is part of the object.
    unsigned char value = 0;
    if(changeMaskPtr[i])
    {
      float objectProb = 1.0f - m_handAppearanceModel->compute_posterior_probability(rgbPtr[i].toVector3());
      if(objectProb >= objectProbThreshold / 100.0f) value = 255;
      //if(objectProb >= objectProbThreshold / 100.0f) value = (uchar)(objectProb * 255);
    }

    cvObjectMask.data[i] = value;
  }

  cv::Mat kernel, temp;

  if(useOpening)
  {
    // Perform a morphological opening operation on the object mask to reduce the noise.
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::erode(cvObjectMask, temp, kernel);  cvObjectMask = temp;
    cv::dilate(cvObjectMask, temp, kernel); cvObjectMask = temp;
  }

  // Find the connected components of the object mask.
  cv::Mat1i ccsImage, stats;
  cv::Mat1d centroids;
  cv::connectedComponentsWithStats(cvObjectMask, ccsImage, stats, centroids);

  // Update the object mask to only contain components over a certain size.
  const int *ccsData = reinterpret_cast<int*>(ccsImage.data);
  for(size_t i = 0, size = rgbInput->dataSize; i < size; ++i)
  {
    int componentSize = stats(ccsData[i], cv::CC_STAT_AREA);
    if(componentSize < componentSizeThreshold)
    {
      cvObjectMask.data[i] = 0;
    }
  }

  if(useClosing)
  {
    // Perform a morphological closing operation on the object mask to fill in holes.
    int k = std::max(closingSize, 3);
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k, k));
    cv::dilate(cvObjectMask, temp, kernel); cvObjectMask = temp;
    cv::erode(cvObjectMask, temp, kernel);  cvObjectMask = temp;
  }

  if(useOnlyLargest)
  {
    // Find the connected components of the object mask again.
    cv::connectedComponentsWithStats(cvObjectMask, ccsImage, stats, centroids);

    // Determine the largest connected component in the object mask and its size.
    int largestComponentIndex = -1;
    int largestComponentSize = INT_MIN;
    for(int componentIndex = 1; componentIndex < stats.rows; ++componentIndex)
    {
      int componentSize = stats(componentIndex, cv::CC_STAT_AREA);
      if(componentSize > largestComponentSize)
      {
        largestComponentIndex = componentIndex;
        largestComponentSize = componentSize;
      }
    }

    // Update the object mask to only contain the largest connected component (if any).
    const int *ccsData = reinterpret_cast<int*>(ccsImage.data);
    for(size_t i = 0, size = rgbInput->dataSize; i < size; ++i)
    {
      if(ccsData[i] != largestComponentIndex)
      {
        cvObjectMask.data[i] = 0;
      }
    }
  }

  cv::imshow(debugWindowName, cvObjectMask);

  // Convert the object mask to InfiniTAM format and return it.
  std::copy(cvObjectMask.data, cvObjectMask.data + m_view->rgb->dataSize, m_targetMask->GetData(MEMORYDEVICE_CPU));
  //return m_targetMask;
#endif
  return changeMask;
}

ITMUChar4Image_Ptr BackgroundSubtractingObjectSegmenter::train(const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState)
{
  // Copy the current colour and depth input images across to the CPU.
  ITMUChar4Image_CPtr rgbInput(m_view->rgb, boost::serialization::null_deleter());
  rgbInput->UpdateHostFromDevice();

  ITMFloatImage_CPtr depthInput(m_view->depth, boost::serialization::null_deleter());
  depthInput->UpdateHostFromDevice();

  // Train a colour appearance model to separate the user's hand from the scene background.
  m_handAppearanceModel->train(rgbInput, make_hand_mask(depthInput, pose, renderState));

  // Generate a segmented image of the user's hand that can be shown to the user to provide them
  // with interactive feedback about the data that is being used to train the appearance model.
  return m_touchDetector->generate_touch_image(m_view);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

ITMUCharImage_CPtr BackgroundSubtractingObjectSegmenter::make_change_mask(const ITMFloatImage_CPtr& depthInput,
                                                                          const ORUtils::SE3Pose& pose,
                                                                          const RenderState_CPtr& renderState) const
{
  rigging::MoveableCamera_CPtr camera(new rigging::SimpleCamera(CameraPoseConverter::pose_to_camera(pose)));
  m_touchDetector->determine_touch_points(camera, depthInput, renderState);

#if 0
  ITMUCharImage_Ptr changeMask = m_touchDetector->get_change_mask();

  ITMFloatImage_CPtr depthRaycast = m_touchDetector->get_depth_raycast();
  depthRaycast->UpdateHostFromDevice();

  uchar *changeMaskPtr = changeMask->GetData(MEMORYDEVICE_CPU);
  const float *depthRaycastPtr = depthRaycast->GetData(MEMORYDEVICE_CPU);
  int pixelCount = static_cast<int>(changeMask->dataSize);

#if WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < pixelCount; ++i)
  {
    if(fabs(depthRaycastPtr[i] - m_touchDetector->invalid_depth_value()) < 1e-3f)
    {
      changeMaskPtr[i] = 0;
    }
  }

  return changeMask;
#else
  // Display the absolute difference between the raw depth image and the depth raycast.
  static ITMFloatImage_Ptr diffRawRaycast;
  diffRawRaycast = m_touchDetector->get_diff_raw_raycast();

  ITMFloatImage_CPtr depthRaycast = m_touchDetector->get_depth_raycast();
  depthRaycast->UpdateHostFromDevice();

  ITMFloatImage_CPtr thresholdedRawDepth = m_touchDetector->get_thresholded_raw_depth();
  thresholdedRawDepth->UpdateHostFromDevice();
  const float *thresholdedRawDepthPtr = thresholdedRawDepth->GetData(MEMORYDEVICE_CPU);

  static ITMUCharImage_Ptr changeMask(new ITMUCharImage(Vector2i(640,480), true, true));
  uchar *changeMaskPtr = changeMask->GetData(MEMORYDEVICE_CPU);
  const float *diffRawRaycastPtr = diffRawRaycast->GetData(MEMORYDEVICE_CPU);
  const float *depthRaycastPtr = depthRaycast->GetData(MEMORYDEVICE_CPU);
  int pixelCount = static_cast<int>(changeMask->dataSize);

  static bool initialised = false;
  static int centreDistThreshold = 70;
  static int componentSizeThreshold = 150;
  static int componentSizeThreshold2 = 800;
  static int gradThreshold = 3;
  static int lowerCompactnessThreshold = 50;
  static int lowerDiffThresholdMm = 15;
  static int upperDepthThresholdMm = 1000;
  if(!initialised)
  {
    cv::namedWindow("Foo", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("centreDistThreshold", "Foo", &centreDistThreshold, 100);
    cv::createTrackbar("componentSizeThreshold", "Foo", &componentSizeThreshold, 2000);
    cv::createTrackbar("componentSizeThreshold2", "Foo", &componentSizeThreshold2, 2000);
    cv::createTrackbar("gradThreshold", "Foo", &gradThreshold, 255);
    cv::createTrackbar("lowerCompactnessThreshold", "Foo", &lowerCompactnessThreshold, 100);
    cv::createTrackbar("lowerDiffThresholdMm", "Foo", &lowerDiffThresholdMm, 100);
    cv::createTrackbar("upperDepthThresholdMm", "Foo", &upperDepthThresholdMm, 2000);
  }

  cv::Mat1b cvDepthRaycast = OpenCVUtil::make_greyscale_image(depthRaycastPtr, 640, 480, OpenCVUtil::ROW_MAJOR, 100.0f);
  cv::Mat gradX, gradY, absGradX, absGradY, grad, thresholdedGrad, dilatedThresholdedGrad;
  cv::Sobel(cvDepthRaycast, gradX, CV_16S, 1, 0, 3);
  cv::convertScaleAbs(gradX, absGradX);
  cv::Sobel(cvDepthRaycast, gradY, CV_16S, 0, 1, 3);
  cv::convertScaleAbs(gradY, absGradY);
  cv::addWeighted(absGradX, 0.5, absGradY, 0.5, 0, grad);
  cv::threshold(grad, thresholdedGrad, gradThreshold, 255.0, cv::THRESH_BINARY);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
  cv::dilate(thresholdedGrad, dilatedThresholdedGrad, kernel);
  cv::imshow("Baz", dilatedThresholdedGrad);

#if 1
  cv::Mat1b cvDepthRaycast2 = OpenCVUtil::make_greyscale_image(thresholdedRawDepthPtr, 640, 480, OpenCVUtil::ROW_MAJOR, 100.0f);
  cv::Mat gradX2, gradY2, absGradX2, absGradY2, grad2, thresholdedGrad2, dilatedThresholdedGrad2;
  cv::Sobel(cvDepthRaycast2, gradX2, CV_16S, 1, 0, 3);
  cv::convertScaleAbs(gradX2, absGradX2);
  cv::Sobel(cvDepthRaycast2, gradY2, CV_16S, 0, 1, 3);
  cv::convertScaleAbs(gradY2, absGradY2);
  cv::addWeighted(absGradX2, 0.5, absGradY2, 0.5, 0, grad2);
  cv::threshold(grad2, thresholdedGrad2, gradThreshold, 255.0, cv::THRESH_BINARY);
  cv::dilate(thresholdedGrad2, dilatedThresholdedGrad2, kernel);
  cv::imshow("Baz2", dilatedThresholdedGrad2);
#endif

#if WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < pixelCount; ++i)
  {
    changeMaskPtr[i] = 255;

    // No live depth
    if(thresholdedRawDepthPtr[i] == -1.0f)
    {
      changeMaskPtr[i] = 0;
      continue;
    }

    // No depth raycast
    if(fabs(depthRaycastPtr[i] - m_touchDetector->invalid_depth_value()) < 1e-3f)
    {
      changeMaskPtr[i] = 0;
      continue;
    }

    // Live depth too far away (unreliable)
    if(thresholdedRawDepthPtr[i] * 1000.0f > upperDepthThresholdMm)
    {
      changeMaskPtr[i] = 0;
      continue;
    }

    // Close to the image corners (unreliable)
    {
      int cx = i % 640, cy = i / 640;
      double xDist = fabs(cx - 320.0), yDist = fabs(cy - 240.0);
      double dist = sqrt((xDist*xDist + yDist*yDist) / (320*320 + 240*240));
      if(static_cast<int>(dist * 100) > centreDistThreshold)
      {
        changeMaskPtr[i] = 0;
        continue;
      }
    }

    // If near a depth raycast edge:
    if(dilatedThresholdedGrad.data[i]/* || dilatedThresholdedGrad2.data[i]*/)
    {
      float value = depthRaycastPtr[i];
      //if(value > thresholdedRawDepthPtr[i])
      {
        int cx = i % 640, cy = i / 640;
        for(int dy = -3; dy <= 3; ++dy)
        {
          int y = cy + dy;
          if(y < 0 || y >= 480) continue;
          for(int dx = -3; dx <= 3; ++dx)
          {
            int x = cx + dx;
            if(x < 0 || x >= 640) continue;
            value = std::min(value, depthRaycastPtr[y * 640 + x]);
          }
        }
      }
      if(diffRawRaycastPtr[i] * 1000.0f < 100)
      //if(diffRawRaycastPtr[i] * 1000.0f < 100 || (thresholdedRawDepthPtr[i] - value) * 1000.0f < 100)
      {
        changeMaskPtr[i] = 0;
        continue;
      }
    }

    // Ignore minor changes (noise)
    if(diffRawRaycastPtr[i] * 1000.0f < lowerDiffThresholdMm)
    {
      changeMaskPtr[i] = 0;
      continue;
    }
  }

#if 1
  static cv::Mat1b cvChangeMask = cv::Mat1b::zeros(m_view->rgb->noDims.y, m_view->rgb->noDims.x);
  for(size_t i = 0, size = changeMask->dataSize; i < size; ++i)
  {
    cvChangeMask.data[i] = changeMaskPtr[i];
  }

  // Find the connected components of the change mask.
  cv::Mat1i ccsImage, stats;
  cv::Mat1d centroids;
  cv::connectedComponentsWithStats(cvChangeMask, ccsImage, stats, centroids);

  // Update the change mask to only contain components over a certain size.
  const int *ccsData = reinterpret_cast<int*>(ccsImage.data);
  for(size_t i = 0, size = changeMask->dataSize; i < size; ++i)
  {
    int componentSize = stats(ccsData[i], cv::CC_STAT_AREA);
    if(componentSize < componentSizeThreshold)
    {
      cvChangeMask.data[i] = 0;
      changeMaskPtr[i] = 0;
    }
  }

  // Update the change mask to only contain components that are reasonably compact.
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(cvChangeMask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  cv::Mat1b badContours = cv::Mat1b::zeros(cvChangeMask.size());
  int largestComponent = -1;
  double largestComponentArea = 0.0;
  for(size_t i = 0, size = contours.size(); i < size; ++i)
  {
    double area = cv::contourArea(contours[i]);
    size_t perimeter = contours[i].size();
    double compactness = 4 * M_PI * area / (perimeter * perimeter);
    if(static_cast<int>(area) < componentSizeThreshold2 && static_cast<int>(compactness * 100 + 0.5) < lowerCompactnessThreshold)
    {
      cv::drawContours(badContours, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
    }
    else if(area > largestComponentArea)
    {
      largestComponent = static_cast<int>(i);
      largestComponentArea = area;
    }
  }

  if(largestComponent != -1)
  {
    // Update the change mask to only contain components that are completely within a 200% bounding box around the largest connected component.
    std::set<int> componentsToRemove;
    cv::Rect largestComponentRect = cv::boundingRect(contours[largestComponent]);
    largestComponentRect.x -= largestComponentRect.width / 2;
    largestComponentRect.y -= largestComponentRect.height / 2;
    largestComponentRect.width *= 2;
    largestComponentRect.height *= 2;
    for(size_t i = 0, size = contours.size(); i < size; ++i)
    {
      if(i == largestComponent) continue;
      cv::Rect componentRect = cv::boundingRect(contours[i]);
      if(!largestComponentRect.contains(componentRect.tl()) || !largestComponentRect.contains(componentRect.br()))
      {
        cv::drawContours(badContours, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
      }
    }
  }

  //cv::imshow("Wibble", badContours);

  for(size_t i = 0, size = changeMask->dataSize; i < size; ++i)
  {
    if(badContours.data[i])
    {
      cvChangeMask.data[i] = 0;
      changeMaskPtr[i] = 0;
    }
  }
#endif

  //OpenCVUtil::show_greyscale_figure("Foo", changeMask->GetData(MEMORYDEVICE_CPU), 640, 480, OpenCVUtil::ROW_MAJOR);
  //cv::waitKey(10);
#endif
  return changeMask;
}

ITMUCharImage_CPtr BackgroundSubtractingObjectSegmenter::make_hand_mask(const ITMFloatImage_CPtr& depthInput,
                                                                        const ORUtils::SE3Pose& pose,
                                                                        const RenderState_CPtr& renderState) const
{
  rigging::MoveableCamera_CPtr camera(new rigging::SimpleCamera(CameraPoseConverter::pose_to_camera(pose)));
  m_touchDetector->determine_touch_points(camera, depthInput, renderState);
  return m_touchDetector->get_touch_mask();
}

}
