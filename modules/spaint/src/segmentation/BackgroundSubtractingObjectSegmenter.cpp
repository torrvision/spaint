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
