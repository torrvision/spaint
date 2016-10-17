/**
 * spaint: SLAMComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SLAMComponentWithScoreForest.h"

#include <tuple>
#include <random>

#include <boost/timer/timer.hpp>
#include <opencv2/imgproc.hpp>

#include <omp.h>

#include <DatasetRGBDInfiniTAM.hpp>

#include "ocv/OpenCVUtil.h"
#include "randomforest/cuda/GPUForest_CUDA.h"

#include "Helpers.hpp"

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

  // Set params as in scoreforests
  m_kInitRansac = 1024;
  m_nbPointsForKabschBoostrap = 3;
  m_useAllModesPerLeafInPoseHypothesisGeneration = true;
  m_checkMinDistanceBetweenSampledModes = true;
  m_minDistanceBetweenSampledModes = 0.3f;
  m_checkRigidTransformationConstraint = false; // Speeds up a lot, was true in scoreforests
  m_translationErrorMaxForCorrectPose = 0.05f;
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

  const Vector4f depthIntrinsics =
      view->calib.intrinsics_d.projectionParamsSimple.all;

  if (m_lowLevelEngine->CountValidDepths(inputDepthImage.get())
      < m_nbPointsForKabschBoostrap)
  {
    std::cout
        << "Number of valid depth pixels insufficient to perform relocalization."
        << std::endl;
    return trackingResult;
  }

  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "computing features on the GPU: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    m_featureExtractor->ComputeFeature(inputRGBImage, inputDepthImage,
        depthIntrinsics, m_featureImage);
  }

//  std::cout << "Feature image size: " << m_featureImage->noDims << std::endl;

  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "evaluating forest on the GPU: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    m_gpuForest->evaluate_forest(m_featureImage, m_leafImage);
  }

//  std::cout << "Leaf image size: " << m_leafImage->noDims << std::endl;

  m_featureImage->UpdateHostFromDevice(); // Need the features on the host for now
  m_leafImage->UpdateHostFromDevice();

  // Generate pose candidates with the new implementation
  std::vector<PoseCandidate> poseCandidates;

  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "generating initial candidates: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    generate_pose_candidates(poseCandidates);
  }

  std::cout << "Generated " << poseCandidates.size() << " initial candidates."
      << std::endl;

  return trackingResult;

  // Create ensemble predictions
  std::vector<boost::shared_ptr<EnsemblePrediction>> predictions(
      m_leafImage->noDims.width);

  int max_modes = -1;
  int total_modes = 0;

  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "creating predictions from leaves: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif

    const int *leafData = m_leafImage->GetData(MEMORYDEVICE_CPU);

    // Create vectors of leaves
    std::vector<std::vector<size_t>> leaves_indices(m_leafImage->noDims.width);

    {
#ifdef ENABLE_TIMERS
      boost::timer::auto_cpu_timer t(6,
          "creating leaves array: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
      for (size_t prediction_idx = 0; prediction_idx < leaves_indices.size();
          ++prediction_idx)
      {
        auto &tree_leaves = leaves_indices[prediction_idx];
        tree_leaves.reserve(m_leafImage->noDims.height);
        for (int tree_idx = 0; tree_idx < m_leafImage->noDims.height;
            ++tree_idx)
        {
          tree_leaves.push_back(
              leafData[tree_idx * m_leafImage->noDims.width + prediction_idx]);
        }
      }
    }

#pragma omp parallel for reduction(max:max_modes), reduction(+:total_modes)
    for (size_t prediction_idx = 0; prediction_idx < leaves_indices.size();
        ++prediction_idx)
    {
      predictions[prediction_idx] =
          m_dataset->GetForest()->GetPredictionForLeaves(
              leaves_indices[prediction_idx]);

      if (predictions[prediction_idx])
      {
        int nbModes = ToEnsemblePredictionGaussianMean(
            predictions[prediction_idx].get())->_modes.size();

        if (nbModes > max_modes)
        {
          max_modes = nbModes;
        }

        total_modes += nbModes;
      }
    }
  }

  std::cout << "Max number of modes: " << max_modes << std::endl;
  std::cout << "Total number of modes: " << total_modes << std::endl;

//  Vector2i px(84, 46);
//  int linear_px = px.y * m_featureImage->noDims.width + px.x;
//
//  std::vector<size_t> leaf_indices;
//
//  std::cout << "Leaves for pixel " << px << ": "; // << leafData[0 * m_leafImage->noDims.width + linear_px] << " "
//  for (int treeIdx = 0; treeIdx < m_leafImage->noDims.height; ++treeIdx)
//  {
//    leaf_indices.push_back(
//        leafData[treeIdx * m_leafImage->noDims.width + linear_px]);
//    std::cout << leaf_indices.back() << " ";
//  }
//
//  std::cout << std::endl;
//
//  // Get the corresponding ensemble prediction
//  boost::shared_ptr<EnsemblePrediction> prediction =
//      m_dataset->GetForest()->GetPredictionForLeaves(leaf_indices);

//  EnsemblePredictionGaussianMean* gm = ToEnsemblePredictionGaussianMean(
//      prediction.get());
//  std::cout << "The prediction has " << gm->_modes.size() << " modes.\n";
//  for (int mode_idx = 0; mode_idx < gm->_modes.size(); ++mode_idx)
//  {
//    std::cout << "Mode " << mode_idx << " has " << gm->_modes[mode_idx].size()
//        << " elements:\n";
//    for (int i = 0; i < gm->_modes[mode_idx].size(); ++i)
//    {
//      std::cout << "(" << gm->_modes[mode_idx][i]->_mean.transpose() << ") ";
//    }
//    std::cout << "\n";
//  }
//
//  std::cout << std::endl;

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

void SLAMComponentWithScoreForest::generate_pose_candidates(
    std::vector<PoseCandidate> &poseCandidates)
{
  poseCandidates.reserve(m_kInitRansac);

  // Clear old predictions and prepare array for new results
  m_featurePredictions.clear();
  m_featurePredictions.resize(m_featureImage->dataSize);

  const int nbThreads = 12;

  std::vector<std::mt19937> engs(nbThreads);
  for (int i = 0; i < nbThreads; ++i)
  {
    engs[i].seed(static_cast<unsigned int>(i + 1));
  }

  omp_set_num_threads(nbThreads);

//  std::cout << "Generating pose candidates Kabsch" << std::endl;
#pragma omp parallel for
  for (size_t i = 0; i < m_kInitRansac; ++i)
  {
    int threadId = omp_get_thread_num();
    PoseCandidate candidate;

    if (hypothesize_pose(candidate, engs[threadId]))
    {
      if (!std::get < 1 > (candidate).empty()) // Has some inliers
      {
        std::get < 3 > (candidate) = i;

#pragma omp critical
        poseCandidates.emplace_back(std::move(candidate));
      }
    }
  }
}

bool SLAMComponentWithScoreForest::hypothesize_pose(PoseCandidate &res,
    std::mt19937 &eng)
{
  Eigen::MatrixXf worldPoints(3, m_nbPointsForKabschBoostrap);
  Eigen::MatrixXf localPoints(3, m_nbPointsForKabschBoostrap);

  std::uniform_int_distribution<int> col_index_generator(0,
      m_featureImage->noDims.width - 1);
  std::uniform_int_distribution<int> row_index_generator(0,
      m_featureImage->noDims.height - 1);

  const RGBDPatchFeature *patchFeaturesData = m_featureImage->GetData(
      MEMORYDEVICE_CPU);
  const int *leafData = m_leafImage->GetData(MEMORYDEVICE_CPU);

  bool foundIsometricMapping = false;
  const int maxIterationsOuter = 20;
  int iterationsOuter = 0;

  while (!foundIsometricMapping && iterationsOuter < maxIterationsOuter)
  {
    ++iterationsOuter;
    std::vector<std::tuple<int, int, int>> selectedPixelsAndModes;

    const int maxIterationsInner = 6000;
    int iterationsInner = 0;
    while (selectedPixelsAndModes.size() != m_nbPointsForKabschBoostrap
        && iterationsInner < maxIterationsInner)
    {
      const int x = col_index_generator(eng);
      const int y = row_index_generator(eng);
      const int linearFeatureIdx = y * m_featureImage->noDims.width + x;
      const RGBDPatchFeature &selectedFeature =
          patchFeaturesData[linearFeatureIdx];

//      const int cache_key = y * rgbd_img.cols + x;
//      const float depth_mm = rgbd_img.at < Vec9f > (y, x).val[5];

      if (selectedFeature.position.w < 0.f) // Invalid feature
        continue;

      boost::shared_ptr<EnsemblePredictionGaussianMean> selectedPrediction;

//#pragma omp critical
      selectedPrediction = m_featurePredictions[linearFeatureIdx];

      if (!selectedPrediction)
      {
        // Get predicted leaves
        std::vector<size_t> featureLeaves(m_leafImage->noDims.height);

        for (int tree_idx = 0; tree_idx < m_leafImage->noDims.height;
            ++tree_idx)
        {
          featureLeaves[tree_idx] = leafData[tree_idx
              * m_leafImage->noDims.width + linearFeatureIdx];
        }

        auto p = m_dataset->GetForest()->GetPredictionForLeaves(featureLeaves);
        selectedPrediction = boost::dynamic_pointer_cast<
            EnsemblePredictionGaussianMean>(p);

        // Store prediction in the vector for future use
#pragma omp critical
        m_featurePredictions[linearFeatureIdx] = selectedPrediction;
      }

      // The prediction might be null if there are no modes (TODO: improve GetPredictionForLeaves somehow)
      if(!selectedPrediction)
        continue;

      ++iterationsInner;

      // Apparently the scoreforests code uses only modes from the first tree to generate hypotheses...
      // TODO: investigate
      int nbModesInFirstTree = -1;
      //int nbModesInFirstTree = selectedPrediction->_modes.size() // Alternatively, use all modes

      {
        // Apparently the modes are sorted with descending number of inliers (PER TREE)
        // so when the number of points for a mode is greater than the previous
        // it means we are seeing a mode of the next tree
        int prevNbPointsPerMode = INT_MAX;
        bool found = false;

        for (int i = 0; i < selectedPrediction->_modes.size(); ++i)
        {
          if (selectedPrediction->_modes[i][0]->_nbPoints > prevNbPointsPerMode)
          {
            nbModesInFirstTree = i;
            found = true;
            break;
          }

          prevNbPointsPerMode = selectedPrediction->_modes[i][0]->_nbPoints;
        }

        if (!found) // There must be only a single tree...
        {
          nbModesInFirstTree = selectedPrediction->_modes.size();
        }
      }

      int selectedModeIdx = 0;
      if (m_useAllModesPerLeafInPoseHypothesisGeneration)
      {
        std::uniform_int_distribution<int> mode_generator(0,
            nbModesInFirstTree - 1);
        selectedModeIdx = mode_generator(eng);
      }

      // This is the first pixel, check that the pixel colour corresponds with the selected mode
      if (selectedPixelsAndModes.empty())
      {
        bool consistentColour = true;

        for (int c = 0; c < 3; ++c)
        {
          if (std::abs(
              selectedFeature.colour.v[c]
                  - selectedPrediction->_modes[selectedModeIdx][1]->_mean(c))
              > 30)
          {
            consistentColour = false;
            break;
          }
        }

        if (!consistentColour)
          continue;
      }

      // if (false)
      if (m_checkMinDistanceBetweenSampledModes)
      {
        const Eigen::VectorXd worldPt =
            selectedPrediction->_modes[selectedModeIdx][0]->_mean;

        // Check that this mode is far enough from the other modes
        bool farEnough = true;

        for (int idxOther = 0; idxOther < selectedPixelsAndModes.size();
            ++idxOther)
        {
          int xOther, yOther, modeIdxOther;
          std::tie(xOther, yOther, modeIdxOther) =
              selectedPixelsAndModes[idxOther];

          const int linearIdxOther = yOther * m_featureImage->noDims.width
              + xOther;

          boost::shared_ptr<EnsemblePredictionGaussianMean> predOther;

          // Assumption is that since it's already in selectedPixelsAndModes it must be valid
//#pragma omp critical
          predOther = m_featurePredictions[linearIdxOther];

          Eigen::VectorXd worldPtOther =
              predOther->_modes[modeIdxOther][0]->_mean;

          float distOther = (worldPtOther - worldPt).norm();
          if (distOther < m_minDistanceBetweenSampledModes)
          {
            farEnough = false;
            break;
          }
        }

        if (!farEnough)
          continue;
      }

      // isometry?
      // if (false)
      // if (true)
      if (m_checkRigidTransformationConstraint)
      {
        bool violatesConditions = false;

        for (int m = 0;
            m < selectedPixelsAndModes.size() && !violatesConditions; ++m)
        {
          int xFirst, yFirst, modeIdxFirst;
          std::tie(xFirst, yFirst, modeIdxFirst) = selectedPixelsAndModes[m];

          const int linearIdxOther = yFirst * m_featureImage->noDims.width
              + xFirst;
          boost::shared_ptr<EnsemblePredictionGaussianMean> predFirst;

          // Assumption is that since it's already in selectedPixelsAndModes it must be valid
//#pragma omp critical
          predFirst = m_featurePredictions[linearIdxOther];

          Eigen::VectorXd worldPtFirst =
              predFirst->_modes[modeIdxFirst][0]->_mean;
          Eigen::VectorXd worldPtCur =
              selectedPrediction->_modes[selectedModeIdx][0]->_mean;

          float distWorld = (worldPtFirst - worldPtCur).norm();

          Eigen::VectorXf localPred = Eigen::Map<const Eigen::Vector3f>(
              patchFeaturesData[linearIdxOther].position.v);
          Eigen::VectorXf localCur = Eigen::Map<const Eigen::Vector3f>(
              selectedFeature.position.v);

          float distLocal = (localPred - localCur).norm();

          if (distLocal < m_minDistanceBetweenSampledModes)
            violatesConditions = true;

          if (std::abs(distLocal - distWorld)
              > 0.5f * m_translationErrorMaxForCorrectPose)
          {
            violatesConditions = true;
          }
        }

        if (violatesConditions)
          continue;
      }

      selectedPixelsAndModes.push_back(
          std::tuple<int, int, int>(x, y, selectedModeIdx));
//      iterationsInner = 0;
    }

//    std::cout << "Inner iterations: " << iterationsInner << std::endl;

    // Reached limit of iterations
    if (selectedPixelsAndModes.size() != m_nbPointsForKabschBoostrap)
      return false;

    std::vector<std::pair<int, int>> tmpInliers;
    for (int s = 0; s < selectedPixelsAndModes.size(); ++s)
    {
      int x, y, modeIdx;
      std::tie(x, y, modeIdx) = selectedPixelsAndModes[s];
      const int linearIdx = y * m_featureImage->noDims.width + x;

      Eigen::VectorXf localPt = Eigen::Map<const Eigen::Vector3f>(
          patchFeaturesData[linearIdx].position.v);

      boost::shared_ptr<EnsemblePredictionGaussianMean> pred;
//#pragma omp critical
      pred = m_featurePredictions[linearIdx];

      Eigen::VectorXd worldPt = pred->_modes[modeIdx][0]->_mean;

      for (int idx = 0; idx < 3; ++idx)
      {
        localPoints(idx, s) = localPt(idx);
        worldPoints(idx, s) = static_cast<float>(worldPt(idx));
      }

      tmpInliers.push_back(std::pair<int, int>(linearIdx, modeIdx));
    }

    Eigen::Matrix4f tmpCameraModel;

    {
//#ifdef ENABLE_TIMERS
//      boost::timer::auto_cpu_timer t(6,
//          "kabsch: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
//#endif
      tmpCameraModel = Helpers::Kabsch(localPoints, worldPoints);
    }

    foundIsometricMapping = true;
    res = std::make_tuple(tmpCameraModel, tmpInliers, 0.0f, -1);
  }

  if (iterationsOuter < maxIterationsOuter)
    return true;

  return false;
}

}
