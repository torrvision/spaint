/**
 * spaint: SLAMComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SLAMComponentWithScoreForest.h"

#include <tuple>
#include <random>

#include <boost/timer/timer.hpp>
#include <opencv2/imgproc.hpp>

#include <DatasetRGBDInfiniTAM.hpp>

#include "ocv/OpenCVUtil.h"
#include "randomforest/cuda/GPUForest_CUDA.h"

using namespace InputSource;
using namespace ITMLib;
using namespace ORUtils;
using namespace RelocLib;

#define ENABLE_TIMERS

namespace spaint
{

//#################### CONSTRUCTORS ####################

SLAMComponentWithScoreForest::SLAMComponentWithScoreForest(
    const SLAMContext_Ptr& context, const std::string& sceneID,
    const ImageSourceEngine_Ptr& imageSourceEngine, TrackerType trackerType,
    const std::vector<std::string>& trackerParams, MappingMode mappingMode,
    TrackingMode trackingMode) :
    SLAMComponent(context, sceneID, imageSourceEngine, trackerType,
        trackerParams, mappingMode, trackingMode)
{
  m_dataset.reset(
      new DatasetRGBDInfiniTAM(
          "/home/tcavallari/code/scoreforests/apps/TrainAndTest/SettingsDatasetRGBDInfiniTAMDesk.yml",
          "/media/data/", 5, 1.0, "DFBP", true, 0, false, 42));

  m_dataset->LoadForest();
//  m_dataset->ResetNodeAndLeaves();

  m_featureExtractor =
      FeatureCalculatorFactory::make_rgbd_patch_feature_calculator(
          ITMLib::ITMLibSettings::DEVICE_CUDA);
  m_featureImage.reset(new RGBDPatchFeatureImage(Vector2i(0, 0), true, true)); // Dummy size just to allocate the container
  m_leafImage.reset(new ITMIntImage(Vector2i(0, 0), true, true)); // Dummy size just to allocate the container
  m_gpuForest.reset(new GPUForest_CUDA(*m_dataset->GetForest()));
}

//#################### DESTRUCTOR ####################
SLAMComponentWithScoreForest::~SLAMComponentWithScoreForest()
{
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

SLAMComponent::TrackingResult SLAMComponentWithScoreForest::process_relocalisation(
    TrackingResult trackingResult)
{
  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);
  const ITMShortImage_Ptr& inputRawDepthImage =
      slamState->get_input_raw_depth_image();
  const ITMFloatImage_Ptr inputDepthImage(
      new ITMFloatImage(slamState->get_view()->depth->noDims, true, true));
  inputDepthImage->SetFrom(slamState->get_view()->depth,
      ORUtils::MemoryBlock<float>::CUDA_TO_CUDA);

//  const ITMUChar4Image_Ptr& inputRGBImage = slamState->get_input_rgb_image();
  const ITMUChar4Image_Ptr inputRGBImage(
      new ITMUChar4Image(slamState->get_view()->rgb->noDims, true, true));
  inputRGBImage->SetFrom(slamState->get_view()->rgb,
      ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CUDA);
  inputRGBImage->UpdateHostFromDevice();

  const TrackingState_Ptr& trackingState = slamState->get_tracking_state();

  const VoxelRenderState_Ptr& liveVoxelRenderState =
      slamState->get_live_voxel_render_state();
  const View_Ptr& view = slamState->get_view();
  const SpaintVoxelScene_Ptr& voxelScene = slamState->get_voxel_scene();

  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "computing features on the GPU: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    m_featureExtractor->ComputeFeature(inputRGBImage, inputDepthImage,
        m_featureImage);
  }

  std::cout << "Feature image size: " << m_featureImage->noDims << std::endl;

  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "evaluating forest on the GPU: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    m_gpuForest->evaluate_forest(m_featureImage, m_leafImage);
  }

  std::cout << "Leaf image size: " << m_leafImage->noDims << std::endl;

  m_leafImage->UpdateHostFromDevice();

  Vector2i px(84, 46);
  int linear_px = px.y * m_featureImage->noDims.width + px.x;
  const auto ptr = m_leafImage->GetData(MEMORYDEVICE_CPU);

  std::vector<size_t> leaf_indices;

  std::cout << "Leaves for pixel " << px << ": "; // << ptr[0 * m_leafImage->noDims.width + linear_px] << " "
  for (int treeIdx = 0; treeIdx < m_leafImage->noDims.height; ++treeIdx)
  {
    leaf_indices.push_back(
        ptr[treeIdx * m_leafImage->noDims.width + linear_px]);
    std::cout << leaf_indices.back() << " ";
  }

  std::cout << std::endl;

  // Get the corresponding ensemble prediction
  boost::shared_ptr<EnsemblePrediction> prediction =
      m_dataset->GetForest()->GetPredictionForLeaves(leaf_indices);

  EnsemblePredictionGaussianMean* gm = ToEnsemblePredictionGaussianMean(prediction.get());
  std::cout << "The prediction has " << gm->_modes.size() << " modes.\n";
  for(int mode_idx = 0; mode_idx < gm->_modes.size(); ++mode_idx)
  {
    std::cout << "Mode " << mode_idx << " has " << gm->_modes[mode_idx].size() << " elements:\n";
    for(int i = 0; i < gm->_modes[mode_idx].size(); ++i)
    {
      std::cout << "("<< gm->_modes[mode_idx][i]->_mean.transpose() << ") ";
    }
    std::cout << "\n";
  }

  std::cout << std::endl;

  return trackingResult;

  if (trackingResult == TrackingResult::TRACKING_FAILED)
  {
    std::cout << "Tracking failed, trying to relocalize..." << std::endl;

    cv::Mat rgbd = build_rgbd_image(inputRGBImage, inputRawDepthImage);
//    boost::shared_ptr<EnsemblePredictionGaussianMean> prediction;

//    {
//      boost::timer::auto_cpu_timer t(6, "evaluating pixel: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
//
//      boost::shared_ptr<InputOutputData> feature = m_dataset->ComputeFeaturesForPixel(rgbd, 100, 100);
//      prediction = boost::dynamic_pointer_cast<EnsemblePredictionGaussianMean>(m_dataset->PredictForFeature(feature));
//    }
//
//    return trackingResult;
//
//    std::cout << "Prediction has " << prediction->_modes.size() << " modes." << std::endl;
//
//    for(size_t mode = 0 ; mode < prediction->_modes.size(); ++mode)
//    {
//      std::cout << "Mode has " << prediction->_modes[mode].size() << " elements." << std::endl;
//      for(auto x : prediction->_modes[mode])
//      {
//        std::cout << "Npoints: " << x->_nbPoints << " - mean: " << x->_mean.transpose() << std::endl;
//      }
//    }

//    DatasetRGBD7Scenes::PredictionsCache cache;
//    std::vector<DatasetRGBD7Scenes::PoseCandidate> candidates;
//
//    {
//      boost::timer::auto_cpu_timer t(6, "generating candidates: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
//      m_dataset->GeneratePoseCandidatesFromImage(rgbd, cache, candidates);
//    }
//
//    std::cout << "Cache has " << cache.size() << " entries. computed " << candidates.size() << " candidates." << std::endl;
//
//    std::mt19937 random_engine;
//
//    std::vector<std::pair<int, int>> sampled_pixels;
//
//    {
//      boost::timer::auto_cpu_timer t(6, "sampling candidates for ransac: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
//      std::vector<bool> dummy_mask;
//      m_dataset->SamplePixelCandidatesForPoseUpdate(rgbd, cache, dummy_mask, sampled_pixels, random_engine);
//    }
//
//    std::cout << "Sampled " << sampled_pixels.size() << " pixels for RANSAC" << std::endl;

    DatasetRGBD7Scenes::PoseCandidate pose;

    {
      boost::timer::auto_cpu_timer t(6,
          "estimating pose: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
      pose = m_dataset->EstimatePose(rgbd);
    }

    std::cout << "new impl, pose:\n" << std::get < 0
        > (pose) << "\n" << std::endl;

//    // Now compute features
//    std::vector<boost::shared_ptr<InputOutputData>> featuresBuffer;
//
//    {
//      boost::timer::auto_cpu_timer t(6, "computing features: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
//      m_dataset->ComputeFeaturesForImage(rgbd, featuresBuffer);
//    }
//
//    std::cout << "Computed " << featuresBuffer.size() << " features." << std::endl;
//
//    std::vector<EnsemblePrediction *> predictions;
//
//    // Evaluate forest
//    {
//      boost::timer::auto_cpu_timer t(6, "evaluating forest: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
//      m_dataset->EvaluateForest(featuresBuffer, predictions);
//    }
//
//    std::cout << "Forest evaluated" << std::endl;
//
//    // Find pose
//    std::tuple<Eigen::MatrixXf, std::vector<std::pair<int, int>>, float, int> result;
//
//    {
//      boost::timer::auto_cpu_timer t(6, "estimating pose: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
//      result = m_dataset->PoseFromPredictions(rgbd, featuresBuffer, predictions);
//    }
//
//    std::cout << "Pose estimated: " << std::get<0>(result) << "\nwith "<< std::get<1>(result).size() << " inliers." << std::endl;

    Matrix4f invPose;
    Eigen::Map<Eigen::Matrix4f> em(invPose.m);
    em = std::get < 0 > (pose);

    trackingState->pose_d->SetInvM(invPose);

    const bool resetVisibleList = true;
    m_denseVoxelMapper->UpdateVisibleList(view.get(), trackingState.get(),
        voxelScene.get(), liveVoxelRenderState.get(), resetVisibleList);
    prepare_for_tracking(TRACK_VOXELS);
    m_trackingController->Track(trackingState.get(), view.get());
    trackingResult = trackingState->trackerResult;

//    // cleanup
//    for(size_t i = 0; i < featuresBuffer.size(); ++i) delete featuresBuffer[i];
//    for(size_t i = 0; i < predictions.size(); ++i) delete predictions[i];
  }
//  else if (trackingResult == TrackingResult::TRACKING_GOOD)
//  {
//    cv::Matx44f invPose(trackingState->pose_d->GetInvM().m);
//    cv::Mat cvInvPose(invPose.t()); // Matrix4f is col major
//
//    cv::Mat rgbd = build_rgbd_image(inputRGBImage, inputRawDepthImage);
//
//    {
//      boost::timer::auto_cpu_timer t(6, "integrating new image: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
//      m_dataset->AddImageFeaturesToForest(rgbd, cvInvPose);
//    }
//  }

  return trackingResult;
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

cv::Mat SLAMComponentWithScoreForest::build_rgbd_image(
    const ITMUChar4Image_Ptr &inputRGBImage,
    const ITMShortImage_Ptr &inputRawDepthImage) const
{
#ifdef ENABLE_TIMERS
  boost::timer::auto_cpu_timer t(6,
      "creating rgbd: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif

  // Create RGBD Mat wrappers to use in the forest
  cv::Mat rgb = OpenCVUtil::make_rgb_image(
      inputRGBImage->GetData(MemoryDeviceType::MEMORYDEVICE_CPU),
      inputRGBImage->noDims.width, inputRGBImage->noDims.height);
  cv::Mat depth(inputRawDepthImage->noDims.height,
      inputRawDepthImage->noDims.width, CV_16SC1,
      inputRawDepthImage->GetData(MemoryDeviceType::MEMORYDEVICE_CPU));

  // scoreforests wants rgb data
  cv::cvtColor(rgb, rgb, CV_BGR2RGB);

  // convert to float images
  rgb.convertTo(rgb, CV_32F);
  depth.convertTo(depth, CV_32F);

  // Dummy channel to fill the rgbd image
  cv::Mat dummyFiller = cv::Mat::zeros(inputRawDepthImage->noDims.height,
      inputRawDepthImage->noDims.width, CV_32FC1);

  std::vector<cv::Mat> channels;
  cv::split(rgb, channels);
  // swap r with b
  std::swap(channels[0], channels[2]);

  // insert 2 dummies and the depth
  channels.push_back(dummyFiller);
  channels.push_back(dummyFiller);
  channels.push_back(depth);
  channels.push_back(dummyFiller);
  channels.push_back(dummyFiller);
  channels.push_back(dummyFiller); // 9 channels since the sampling functions use Vec9f...

  cv::Mat rgbd;
  cv::merge(channels, rgbd);

  return rgbd;
}

}
