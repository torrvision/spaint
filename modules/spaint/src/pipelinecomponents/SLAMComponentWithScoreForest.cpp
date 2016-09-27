/**
 * spaint: SLAMComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SLAMComponentWithScoreForest.h"

#include <tuple>

#include <boost/timer/timer.hpp>
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
  const TrackingState_Ptr& trackingState = slamState->get_tracking_state();

  const VoxelRenderState_Ptr& liveVoxelRenderState = slamState->get_live_voxel_render_state();
  const View_Ptr& view = slamState->get_view();
  const SpaintVoxelScene_Ptr& voxelScene = slamState->get_voxel_scene();

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

    {
      boost::timer::auto_cpu_timer t;
      m_dataset->ComputeFeaturesForImage(rgbd, featuresBuffer);
    }

    std::cout << "Computed " << featuresBuffer.size() << " features." << std::endl;

    std::vector<EnsemblePrediction *> predictions;

    // Evaluate forest
    {
      boost::timer::auto_cpu_timer t;
      m_dataset->EvaluateForest(featuresBuffer, predictions);
    }

    std::cout << "Forest evaluated" << std::endl;

    // Find pose
    std::tuple<Eigen::MatrixXf, std::vector<std::pair<int, int>>, float, int> result;

    {
      boost::timer::auto_cpu_timer t;
      result = m_dataset->PoseFromPredictions(rgbd, featuresBuffer, predictions);
    }

    std::cout << "Pose estimated: " << std::get<0>(result) << "\nwith "<< std::get<1>(result).size() << " inliers." << std::endl;

    Matrix4f invPose;
    Eigen::Map<Eigen::Matrix4f> em(invPose.m);
    em = std::get<0>(result);

    trackingState->pose_d->SetInvM(invPose);

    const bool resetVisibleList = true;
    m_denseVoxelMapper->UpdateVisibleList(view.get(), trackingState.get(), voxelScene.get(), liveVoxelRenderState.get(), resetVisibleList);
    prepare_for_tracking(TRACK_VOXELS);
    m_trackingController->Track(trackingState.get(), view.get());
    trackingResult = trackingState->trackerResult;

//    for (const auto &p : predictions)
//    {
//      auto ep = ToEnsemblePredictionGaussianMean(p);
//      std::cout << "Prediction has " << ep->_modes.size() << " modes." << std::endl;
//      for (auto &m : ep->_modes)
//      {
//        std::cout << "\tMode has " << m.size() << " components.\n";
//        for (auto &c : m)
//        {
//          std::cout << "\t\t" << c->_mean.transpose() << "\n";
//        }
//      }
//      break;
//    }


    // cleanup
    for(size_t i = 0; i < featuresBuffer.size(); ++i) delete featuresBuffer[i];
    for(size_t i = 0; i < predictions.size(); ++i) delete predictions[i];
  }

  return trackingResult;
}

}
