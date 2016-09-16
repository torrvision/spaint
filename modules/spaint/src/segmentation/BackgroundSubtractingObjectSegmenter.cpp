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
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void BackgroundSubtractingObjectSegmenter::reset()
{
  m_handAppearanceModel.reset(new ColourAppearanceModel(30, 30));
}

ITMUCharImage_CPtr BackgroundSubtractingObjectSegmenter::segment(const ORUtils::SE3Pose& pose, const RenderState_CPtr& renderState) const
{
  // TEMPORARY: Debugging controls.
  static bool initialised = false;
  static int objectProbThreshold = 80;
  const std::string debugWindowName = "Object Mask";
  if(!initialised)
  {
    cv::namedWindow(debugWindowName, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("objectProbThreshold", debugWindowName, &objectProbThreshold, 100);
    initialised = true;
  }

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

  cv::imshow(debugWindowName, objectMask);

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

ITMUCharImage_CPtr BackgroundSubtractingObjectSegmenter::make_change_mask(const ITMFloatImage_CPtr& depthInput,
                                                                          const ORUtils::SE3Pose& pose,
                                                                          const RenderState_CPtr& renderState) const
{
  // TEMPORARY: Debugging controls.
  static bool initialised = false;
  static int centreDistThreshold = 70;
  static int componentSizeThreshold = 150;
  static int componentSizeThreshold2 = 800;
  static int componentSizeThreshold3 = 1000;
  static int gradThreshold = 3;
  static int lowerCompactnessThreshold = 50;
  static int lowerDiffThresholdMm = 15;
  static int upperDepthThresholdMm = 1000;
  const std::string debugWindowName = "Change Mask";
  if(!initialised)
  {
    cv::namedWindow(debugWindowName, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("centreDistThreshold", debugWindowName, &centreDistThreshold, 100);
    cv::createTrackbar("componentSizeThreshold", debugWindowName, &componentSizeThreshold, 2000);
    cv::createTrackbar("componentSizeThreshold2", debugWindowName, &componentSizeThreshold2, 2000);
    cv::createTrackbar("componentSizeThreshold3", debugWindowName, &componentSizeThreshold3, 2000);
    cv::createTrackbar("gradThreshold", debugWindowName, &gradThreshold, 255);
    cv::createTrackbar("lowerCompactnessThreshold", debugWindowName, &lowerCompactnessThreshold, 100);
    cv::createTrackbar("lowerDiffThresholdMm", debugWindowName, &lowerDiffThresholdMm, 100);
    cv::createTrackbar("upperDepthThresholdMm", debugWindowName, &upperDepthThresholdMm, 2000);
  }

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

  // Get the difference between the live depth image and the depth raycast of the scene.
  ITMFloatImage_Ptr diffRawRaycast = m_touchDetector->get_diff_raw_raycast();
  const float *diffRawRaycastPtr = diffRawRaycast->GetData(MEMORYDEVICE_CPU);

  // Make an initial change mask, starting from the whole image and filtering out pixels based on some simple criteria.
  static ITMUCharImage_Ptr changeMask(new ITMUCharImage(Vector2i(640,480), true, true));
  uchar *changeMaskPtr = changeMask->GetData(MEMORYDEVICE_CPU);
  int pixelCount = static_cast<int>(changeMask->dataSize);

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
    if(dilatedThresholdedGrad.data[i])
    {
      if(diffRawRaycastPtr[i] * 1000.0f < 100)
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
    // Update the change mask to exclude relatively small components that are not completely within a 200% bounding box around the largest connected component.
    std::set<int> componentsToRemove;
    cv::Rect largestComponentRect = cv::boundingRect(contours[largestComponent]);
    largestComponentRect.x -= largestComponentRect.width / 2;
    largestComponentRect.y -= largestComponentRect.height / 2;
    largestComponentRect.width *= 2;
    largestComponentRect.height *= 2;
    for(size_t i = 0, size = contours.size(); i < size; ++i)
    {
      if(i == largestComponent) continue;
      if(cv::contourArea(contours[i]) > componentSizeThreshold3) continue;
      cv::Rect componentRect = cv::boundingRect(contours[i]);
      if(!largestComponentRect.contains(componentRect.tl()) || !largestComponentRect.contains(componentRect.br()))
      {
        cv::drawContours(badContours, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
      }
    }
  }

  for(size_t i = 0, size = changeMask->dataSize; i < size; ++i)
  {
    if(badContours.data[i])
    {
      cvChangeMask.data[i] = 0;
      changeMaskPtr[i] = 0;
    }
  }

  OpenCVUtil::show_greyscale_figure(debugWindowName, changeMask->GetData(MEMORYDEVICE_CPU), 640, 480, OpenCVUtil::ROW_MAJOR);
  cv::waitKey(10);
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
