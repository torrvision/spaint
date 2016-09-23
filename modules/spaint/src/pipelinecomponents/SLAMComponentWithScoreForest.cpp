/**
 * spaint: SLAMComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SLAMComponentWithScoreForest.h"

#include <opencv2/imgproc.hpp>

#include <DatasetRGBDInfiniTAM.hpp>

#include "ocv/OpenCVUtil.h"

using namespace InputSource;
using namespace ITMLib;
using namespace ORUtils;
using namespace RelocLib;

namespace spaint {

//#################### CONSTRUCTORS ####################

SLAMComponentWithScoreForest::SLAMComponentWithScoreForest(const SLAMContext_Ptr& context, const std::string& sceneID, const ImageSourceEngine_Ptr& imageSourceEngine,
                             TrackerType trackerType, const std::vector<std::string>& trackerParams, MappingMode mappingMode, TrackingMode trackingMode)
: SLAMComponent(context, sceneID, imageSourceEngine, trackerType, trackerParams, mappingMode, trackingMode)
{
  m_dataset.reset(new DatasetRGBDInfiniTAM(
      "/home/tcavallari/code/scoreforests/apps/TrainAndTest/SettingsDatasetRGBDInfiniTAMDesk.yml",
      "/media/data/",
      5,
      1.0,
      "DFBP",
      true,
      0,
      false,
      42));

  m_dataset->LoadForest();
}

//#################### DESTRUCTOR ####################
SLAMComponentWithScoreForest::~SLAMComponentWithScoreForest() {}

//#################### PROTECTED MEMBER FUNCTIONS ####################

SLAMComponent::TrackingResult SLAMComponentWithScoreForest::process_relocalisation(TrackingResult trackingResult)
{
  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);
  const ITMShortImage_Ptr& inputRawDepthImage = slamState->get_input_raw_depth_image();
  const ITMUChar4Image_Ptr& inputRGBImage = slamState->get_input_rgb_image();

  if(trackingResult == TrackingResult::TRACKING_FAILED)
  {
    std::cout << "Tracking failed, trying to relocalize..." << std::endl;
    // Create RGBD Mat to use in the forest
    cv::Mat rgb = OpenCVUtil::make_rgb_image(inputRGBImage->GetData(MemoryDeviceType::MEMORYDEVICE_CPU), inputRGBImage->noDims.width, inputRGBImage->noDims.height);
    cv::Mat depth(inputRawDepthImage->noDims.height, inputRawDepthImage->noDims.width, CV_16UC1, inputRawDepthImage->GetData(MemoryDeviceType::MEMORYDEVICE_CPU));

    // convert to float images
    rgb.convertTo(rgb, CV_32F);
    depth.convertTo(depth, CV_32F);

    // Dummy channel to fill the rgbd image
    cv::Mat dummyFiller = cv::Mat::zeros(inputRawDepthImage->noDims.height, inputRawDepthImage->noDims.width, CV_32FC1);

    std::vector<cv::Mat> channels;
    cv::split(rgb, channels);
    // swap r & b
    std::swap(channels[0], channels[2]);

    // insert 2 dummies and the depth
    channels.push_back(dummyFiller);
    channels.push_back(dummyFiller);
    channels.push_back(depth);

    cv::Mat rgbd;
    cv::merge(channels, rgbd);

    // Now compute features
    std::vector<InputOutputData *> featuresBuffer;
    m_dataset->ComputeFeaturesForImage(rgbd, featuresBuffer);

    std::cout << "Computed " << featuresBuffer.size() << " features." << std::endl;

    // cleanup
    for(size_t i = 0; i < featuresBuffer.size(); ++i) delete featuresBuffer[i];
  }

  return TrackingResult::TRACKING_GOOD;
}

}
