/**
 * itmx: ICPRefiningRelocaliser.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ICPRefiningRelocaliser.h"

#include <iostream>
#include <stdexcept>

#include <ITMLib/Core/ITMTrackingController.h>
#include <ITMLib/Engines/Visualisation/ITMVisualisationEngineFactory.h>
#include <ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>
#include <ITMLib/Trackers/ITMTrackerFactory.h>

#include <orx/base/ORImagePtrTypes.h>

#include <tvgutil/filesystem/PathFinder.h>
#include <tvgutil/misc/SettingsContainer.h>
#include <tvgutil/timing/TimeUtil.h>

#include "../persistence/PosePersister.h"
#include "../visualisation/DepthVisualisationUtil.tpp"
#include "../visualisation/DepthVisualiserFactory.h"

#ifdef WITH_OPENCV
#include "../ocv/OpenCVUtil.h"
#endif

namespace itmx {

//#################### CONSTRUCTORS ####################

template <typename VoxelType, typename IndexType>
ICPRefiningRelocaliser<VoxelType,IndexType>::ICPRefiningRelocaliser(const orx::Relocaliser_Ptr& innerRelocaliser, const Tracker_Ptr& tracker,
                                                                    const Vector2i& rgbImageSize, const Vector2i& depthImageSize,
                                                                    const ITMLib::ITMRGBDCalib& calib, const Scene_Ptr& scene,
                                                                    const DenseMapper_Ptr& denseVoxelMapper, const Settings_CPtr& settings)
: RefiningRelocaliser(innerRelocaliser),
  m_denseVoxelMapper(denseVoxelMapper),
  m_depthVisualiser(DepthVisualiserFactory::make_depth_visualiser(settings->deviceType)),
  m_scene(scene),
  m_settings(settings),
  m_timerInitialRelocalisation("Initial Relocalisation"),
  m_timerRefinement("ICP Refinement"),
  m_timerRelocalisation("Relocalisation"),
  m_timerTraining("Training"),
  m_timerUpdate("Update"),
  m_tracker(tracker),
  m_visualisationEngine(ITMVisualisationEngineFactory::MakeVisualisationEngine<VoxelType,IndexType>(settings->deviceType))
{
  // Construct the tracking controller, tracking state, view and render state.
  m_trackingController.reset(new ITMLib::ITMTrackingController(m_tracker.get(), m_settings.get()));
  m_trackingState.reset(new ITMLib::ITMTrackingState(depthImageSize, m_settings->GetMemoryType()));
  m_view.reset(new ITMLib::ITMView(calib, rgbImageSize, depthImageSize, m_settings->deviceType == DEVICE_CUDA));
  m_voxelRenderState.reset(ITMLib::ITMRenderStateFactory<IndexType>::CreateRenderState(
    m_trackingController->GetTrackedImageSize(rgbImageSize, depthImageSize),
    m_scene->sceneParams,
    m_settings->GetMemoryType()
  ));

  // Configure the relocaliser based on the settings that have been passed in.
  const static std::string settingsNamespace = "ICPRefiningRelocaliser.";
  m_chooseBestResult = m_settings->get_first_value<bool>(settingsNamespace + "chooseBestResult", false);
  m_saveImages = m_settings->get_first_value<bool>(settingsNamespace + "saveRelocalisationImages", false);
  m_savePoses = m_settings->get_first_value<bool>(settingsNamespace + "saveRelocalisationPoses", false);
  m_saveTimes = m_settings->get_first_value<bool>(settingsNamespace + "saveRelocalisationTimes", false);
  m_timersEnabled = m_settings->get_first_value<bool>(settingsNamespace + "timersEnabled", false);

  // Get the (global) experiment tag.
  const std::string experimentTag = m_settings->get_first_value<std::string>("experimentTag", tvgutil::TimeUtil::get_iso_timestamp());

  if(m_savePoses)
  {
    // Determine the directory to which to save the poses and make sure that it exists.
    m_posePathGenerator.reset(tvgutil::SequentialPathGenerator(tvgutil::find_subdir_from_executable("reloc_poses") / experimentTag));
    boost::filesystem::create_directories(m_posePathGenerator->get_base_dir());

    // Output the directory we're using (for debugging purposes).
    std::cout << "Saving relocalisation poses in: " << m_posePathGenerator->get_base_dir() << '\n';
  }

  if(m_saveImages)
  {
    // Determine the directory to which to save the images and make sure that it exists.
    m_imagePathGenerator.reset(tvgutil::SequentialPathGenerator(tvgutil::find_subdir_from_executable("reloc_images") / experimentTag));
    boost::filesystem::create_directories(m_imagePathGenerator->get_base_dir());

    std::vector<std::string> sequenceSpecifiers = m_settings->get_values<std::string>("sequenceSpecifiers");
    if(sequenceSpecifiers.size() < 2)
    {
      throw std::runtime_error("Error: saveRelocalisationImages requires at least two sequence specifiers (one for the training and one for the testing sequence).");
    }

    std::cout << "Reading ground truth poses from: " << sequenceSpecifiers[1] << "\n";
    m_gtPathGenerator.reset(tvgutil::SequentialPathGenerator(sequenceSpecifiers[1]));

    // Output the directory we're using (for debugging purposes).
    std::cout << "Saving relocalisation images in: " << m_imagePathGenerator->get_base_dir() << '\n';
  }

  if(m_saveTimes)
  {
    // Enable the timers.
    m_timersEnabled = true;

    // Ensure that the directory in which we want to save the relocalisation times exists.
    boost::filesystem::path timersOutputFolder(tvgutil::find_subdir_from_executable("reloc_times"));
    boost::filesystem::create_directories(timersOutputFolder);

    // Construct the output filename.
    m_timersOutputFile = (timersOutputFolder / (experimentTag + ".txt")).string();
  }
}

//#################### DESTRUCTOR ####################

template <typename VoxelType, typename IndexType>
ICPRefiningRelocaliser<VoxelType,IndexType>::~ICPRefiningRelocaliser()
{
  if(m_timersEnabled)
  {
    std::cout << "Training calls: " << m_timerTraining.count() << ", average duration: " << m_timerTraining.average_duration() << '\n';
    std::cout << "Update calls: " << m_timerUpdate.count() << ", average duration: " << m_timerUpdate.average_duration() << '\n';
    std::cout << "Initial Relocalisation calls: " << m_timerInitialRelocalisation.count() << ", average duration: " << m_timerInitialRelocalisation.average_duration() << '\n';
    std::cout << "ICP Refinement calls: " << m_timerRefinement.count() << ", average duration: " << m_timerRefinement.average_duration() << '\n';
    std::cout << "Total Relocalisation calls: " << m_timerRelocalisation.count() << ", average duration: " << m_timerRelocalisation.average_duration() << '\n';
  }

  if(m_saveTimes)
  {
    std::cout << "Saving average relocalisation times in: " << m_timersOutputFile << '\n';

    std::ofstream fs(m_timersOutputFile.c_str());

    // Output the average durations.
    fs << m_timerTraining.average_duration().count() << ' '
       << m_timerUpdate.average_duration().count() << ' '
       << m_timerInitialRelocalisation.average_duration().count() << ' '
       << m_timerRefinement.average_duration().count() << ' '
       << m_timerRelocalisation.average_duration().count() << '\n';
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::finish_training()
{
  m_innerRelocaliser->finish_training();
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::load_from_disk(const std::string& inputFolder)
{
  m_innerRelocaliser->load_from_disk(inputFolder);
}

template <typename VoxelType, typename IndexType>
std::vector<orx::Relocaliser::Result>
ICPRefiningRelocaliser<VoxelType, IndexType>::relocalise(const ORUChar4Image *colourImage, const ORFloatImage *depthImage,
                                                         const Vector4f& depthIntrinsics) const
{
  std::vector<ORUtils::SE3Pose> initialPoses;
  return relocalise(colourImage, depthImage, depthIntrinsics, initialPoses);
}

template <typename VoxelType, typename IndexType>
std::vector<orx::Relocaliser::Result>
ICPRefiningRelocaliser<VoxelType, IndexType>::relocalise(const ORUChar4Image *colourImage, const ORFloatImage *depthImage,
                                                         const Vector4f& depthIntrinsics, std::vector<ORUtils::SE3Pose>& initialPoses) const
{
#if DEBUGGING
  static int frameIdx = -1;
  ++frameIdx;
  std::cout << "---\nFrame Index: " << frameIdx << std::endl;
#endif

  // Reset the initial poses.
  initialPoses.clear();

  // Run the inner relocaliser. If it fails, save dummy poses and early out.
  start_timer_sync(m_timerRelocalisation);
  start_timer_nosync(m_timerInitialRelocalisation); // No need to synchronize the GPU again.
  std::vector<Result> initialResults = m_innerRelocaliser->relocalise(colourImage, depthImage, depthIntrinsics);
  stop_timer_sync(m_timerInitialRelocalisation);

  if(initialResults.empty())
  {
    Matrix4f invalidPose;
    invalidPose.setValues(std::numeric_limits<float>::quiet_NaN());
    stop_timer_nosync(m_timerRelocalisation); // No need to synchronize the GPU again.
    save_poses(invalidPose, invalidPose);
    if(m_imagePathGenerator) m_imagePathGenerator->increment_index();
    if(m_gtPathGenerator) m_gtPathGenerator->increment_index();
    return std::vector<Relocaliser::Result>();
  }

  std::vector<Relocaliser::Result> refinedResults;
  float bestScore = static_cast<float>(INT_MAX);

  start_timer_nosync(m_timerRefinement); // No need to synchronize the GPU again.

  // Reset the render state before raycasting (we do this once for each relocalisation attempt).
  // FIXME: It would be nicer to simply create the render state once and then reuse it, but unfortunately this leads
  //        to the program randomly crashing after a while. The crash may be occurring because we don't use this render
  //        state to integrate frames into the scene, but we haven't been able to pin this down yet. As a result, we
  //        currently reset the render state each time as a workaround. A mildly less costly alternative might
  //        be to pass in a render state that is being used elsewhere and reuse it here, but that feels messier.
  m_voxelRenderState->Reset();

  // For each initial result from the inner relocaliser:
  for(size_t resultIdx = 0; resultIdx < initialResults.size(); ++resultIdx)
  {
    // Get the suggested pose.
    const ORUtils::SE3Pose initialPose = initialResults[resultIdx].pose;

    // Copy the depth and RGB images into the view.
    m_view->depth->SetFrom(depthImage, m_settings->deviceType == DEVICE_CUDA ? ORFloatImage::CUDA_TO_CUDA : ORFloatImage::CPU_TO_CPU);
    m_view->rgb->SetFrom(colourImage, m_settings->deviceType == DEVICE_CUDA ? ORUChar4Image::CUDA_TO_CUDA : ORUChar4Image::CPU_TO_CPU);

    // Set up the tracking state using the initial pose.
    m_trackingState->pose_d->SetFrom(&initialPose);

    // Update the list of visible blocks.
    const bool resetVisibleList = true;
    m_denseVoxelMapper->UpdateVisibleList(m_view.get(), m_trackingState.get(), m_scene.get(), m_voxelRenderState.get(), resetVisibleList);

    // Raycast from the initial pose to prepare for tracking.
    m_trackingController->Prepare(m_trackingState.get(), m_scene.get(), m_view.get(), m_visualisationEngine.get(), m_voxelRenderState.get());

    // Run the tracker to refine the initial pose.
    m_trackingController->Track(m_trackingState.get(), m_view.get());

    // If tracking succeeded:
    if(m_trackingState->trackerResult != ITMLib::ITMTrackingState::TRACKING_FAILED)
    {
      // Set up the refined result.
      Result refinedResult;
      refinedResult.pose.SetFrom(m_trackingState->pose_d);
      refinedResult.quality = m_trackingState->trackerResult == ITMLib::ITMTrackingState::TRACKING_GOOD ? RELOCALISATION_GOOD : RELOCALISATION_POOR;
      refinedResult.score = m_trackingState->trackerScore;

      // If the inner relocaliser produced multiple initial results, and we're trying to choose the best one after refinement:
      if(initialResults.size() > 1 && m_chooseBestResult)
      {
        // Score the refined result.
        refinedResult.score = score_pose(refinedResult.pose);

#if DEBUGGING
        std::cout << resultIdx << ": " << refinedResult.score << '\n';
#endif

        // If the score is better than the current best score, update the current best score and result.
        if(refinedResult.score < bestScore)
        {
          bestScore = refinedResult.score;
          initialPoses.clear();
          initialPoses.push_back(initialPose);
          refinedResults.clear();
          refinedResults.push_back(refinedResult);
        }
      }
      else
      {
        // If the inner relocaliser only produced one initial result, or we're not trying to choose the best one,
        // simply store the initial pose and refined result without any scoring.
        initialPoses.push_back(initialPose);
        refinedResults.push_back(refinedResult);
      }
    }
  }

  stop_timer_sync(m_timerRefinement);
  stop_timer_nosync(m_timerRelocalisation); // No need to synchronize the GPU again.

  // Save the best initial and refined poses if needed.
  if(m_savePoses)
  {
    // The best initial pose is the best one returned by the inner relocaliser.
    const Matrix4f initialPose = initialResults[0].pose.GetInvM();

    // The best refined pose is the pose (if any) whose score is lowest after refinement. Note that the
    // best refined pose is not necessarily the result of refining the best initial pose, since refinement
    // is not monotonic. If refinement failed for all initial poses, the refined pose is set to NaNs.
    Matrix4f refinedPose;
    if(!refinedResults.empty())
    {
      refinedPose = refinedResults[0].pose.GetInvM();
    }
    else
    {
      refinedPose.setValues(std::numeric_limits<float>::quiet_NaN());
    }

    // Actually save the poses.
    save_poses(initialPose, refinedPose);

    // Since we are saving the poses (i.e. we are running in evaluation mode), we force the quality of
    // every refined result to POOR to prevent fusion whilst evaluating the testing sequence.
    for(size_t i = 0; i < refinedResults.size(); ++i)
    {
      refinedResults[i].quality = RELOCALISATION_POOR;
    }
  }

  // If requested, render images from both the ground truth pose, and the best initial and refined poses, and save them to disk. Also save the input depth image.
  if(m_saveImages)
  {
#if WITH_OPENCV
    const cv::Size imgSize(m_view->depth->noDims.width, m_view->depth->noDims.height);
    ORFloatImage_Ptr synthDepthF(new ORFloatImage(m_view->depth->noDims, true, true));
    ORUChar4Image_Ptr synthDepthU(new ORUChar4Image(m_view->depth->noDims, true, true));

    // Step 1: Read in the ground truth pose (stored as a matrix in column-major order).
    std::ifstream poseFile(m_gtPathGenerator->make_path("frame-%06i.pose.txt").string().c_str());

    Matrix4f invPose;
    poseFile >> invPose.m00 >> invPose.m10 >> invPose.m20 >> invPose.m30
             >> invPose.m01 >> invPose.m11 >> invPose.m21 >> invPose.m31
             >> invPose.m02 >> invPose.m12 >> invPose.m22 >> invPose.m32
             >> invPose.m03 >> invPose.m13 >> invPose.m23 >> invPose.m33;

    ORUtils::SE3Pose gtPose;
    gtPose.SetInvM(invPose);

    // Step 2: Render a synthetic depth image of the scene from the ground truth pose, and save it to disk.
    DepthVisualisationUtil<VoxelType,IndexType>::generate_depth_from_voxels(
      synthDepthF, m_scene, gtPose, m_view->calib.intrinsics_d, m_voxelRenderState,
      DepthVisualiser::DT_ORTHOGRAPHIC, m_visualisationEngine, m_depthVisualiser, m_settings
    );

    save_colourised_depth(synthDepthF.get(), synthDepthU, "image-%06i.gt.png");

    // Step 3: Copy the input depth image to the CPU and save it to disk.
    m_view->depth->UpdateHostFromDevice();
    save_colourised_depth(m_view->depth, synthDepthU, "image-%06i.depth.png");

    // Step 4: Render a synthetic depth image of the scene from the initial relocalised pose (which is always valid if we got here), and save it to disk.
    DepthVisualisationUtil<VoxelType,IndexType>::generate_depth_from_voxels(
      synthDepthF, m_scene, initialResults[0].pose, m_view->calib.intrinsics_d, m_voxelRenderState,
      DepthVisualiser::DT_ORTHOGRAPHIC, m_visualisationEngine, m_depthVisualiser, m_settings
    );

    save_colourised_depth(synthDepthF.get(), synthDepthU, "image-%06i.reloc.png");

    // Step 5: Compute the difference between the input depth image and the rendering from the ground truth pose, and save it to disk.
    cv::Mat inputDepthF = cv::Mat(imgSize, CV_32FC1, m_view->depth->GetData(MEMORYDEVICE_CPU)).clone();
    cv::Mat gtDepthF = cv::Mat(imgSize, CV_32FC1, synthDepthF->GetData(MEMORYDEVICE_CPU)).clone();
    compute_and_save_diff(inputDepthF, gtDepthF, "image-%06i.gtDiff.png");

    // Step 6: Compute the "score" for the ground truth pose and save it to disk.
    {
      std::ofstream fs(m_imagePathGenerator->make_path("image-%06i.gtScore.txt").string().c_str());
      fs << score_pose(gtPose) << "\n";
    }

    // Step 7: Compute the difference between the input depth image and the rendering from the initial relocalised pose, and save it to disk.
    cv::Mat initialDepthF = cv::Mat(imgSize, CV_32FC1, synthDepthF->GetData(MEMORYDEVICE_CPU)).clone();
    compute_and_save_diff(inputDepthF, initialDepthF, "image-%06i.relocDiff.png");

    // Step 8: Compute the "score" for the initial relocalised pose and save it to disk.
    {
      std::ofstream fs(m_imagePathGenerator->make_path("image-%06i.relocScore.txt").string().c_str());
      fs << score_pose(initialResults[0].pose) << "\n";
    }

    // If there is a refined pose:
    if(!refinedResults.empty())
    {
      // Step 9: Render a synthetic depth image of the scene from the refined pose (which is always valid if we got here), and save it to disk.
      DepthVisualisationUtil<VoxelType,IndexType>::generate_depth_from_voxels(
        synthDepthF, m_scene, refinedResults[0].pose, m_view->calib.intrinsics_d, m_voxelRenderState,
        DepthVisualiser::DT_ORTHOGRAPHIC, m_visualisationEngine, m_depthVisualiser, m_settings
      );

      save_colourised_depth(synthDepthF.get(), synthDepthU, "image-%06i.icp.png");

      // Step 10: Compute the difference between the input depth image and the rendering from the refined pose, and save it to disk.
      cv::Mat refinedDepthF = cv::Mat(imgSize, CV_32FC1, synthDepthF->GetData(MEMORYDEVICE_CPU)).clone();
      compute_and_save_diff(inputDepthF, refinedDepthF, "image-%06i.icpDiff.png");

      // Step 11: Compute the "score" for the refined pose and save it to disk.
      {
        std::ofstream fs(m_imagePathGenerator->make_path("image-%06i.icpScore.txt").string().c_str());
        fs << score_pose(refinedResults[0].pose) << "\n";
      }
    }
#endif

    m_imagePathGenerator->increment_index();
    m_gtPathGenerator->increment_index();
  }

  return refinedResults;
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::reset()
{
  m_innerRelocaliser->reset();
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::save_to_disk(const std::string& outputFolder) const
{
  m_innerRelocaliser->save_to_disk(outputFolder);
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::train(const ORUChar4Image *colourImage, const ORFloatImage *depthImage,
                                                        const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose)
{
  start_timer_sync(m_timerTraining);
  m_innerRelocaliser->train(colourImage, depthImage, depthIntrinsics, cameraPose);
  stop_timer_sync(m_timerTraining);
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::update()
{
  start_timer_sync(m_timerUpdate);
  m_innerRelocaliser->update();
  stop_timer_sync(m_timerUpdate);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

#ifdef WITH_OPENCV
template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::compute_and_save_diff(const cv::Mat& depthImage1, const cv::Mat& depthImage2, const std::string& pattern) const
{
  cv::Mat diff;
  cv::absdiff(depthImage1, depthImage2, diff);
  diff.convertTo(diff, CV_8U, 255 / 0.3);
  cv::applyColorMap(diff, diff, cv::COLORMAP_JET);
  cv::imwrite(m_imagePathGenerator->make_path(pattern).string().c_str(), diff);
}
#endif

#ifdef WITH_OPENCV
template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::save_colourised_depth(const ORFloatImage *depthF, const ORUChar4Image_Ptr& depthU, const std::string& pattern) const
{
  m_visualisationEngine->DepthToUchar4(depthU.get(), depthF);
  cv::Size imgSize(depthF->noDims.width, depthF->noDims.height);
  cv::Mat cvDepthU = cv::Mat(imgSize, CV_8UC4, depthU->GetData(MEMORYDEVICE_CPU));
  cv::Mat cvOutput;
  cv::cvtColor(cvDepthU, cvOutput, cv::COLOR_RGBA2BGR);
  cv::imwrite(m_imagePathGenerator->make_path(pattern).string().c_str(), cvOutput);
}
#endif

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::save_poses(const Matrix4f& relocalisedPose, const Matrix4f& refinedPose) const
{
  if(!m_savePoses) return;

  PosePersister::save_pose_on_thread(relocalisedPose, m_posePathGenerator->make_path("pose-%06i.reloc.txt"));
  PosePersister::save_pose_on_thread(refinedPose, m_posePathGenerator->make_path("pose-%06i.icp.txt"));
  m_posePathGenerator->increment_index();
}

template <typename VoxelType, typename IndexType>
float ICPRefiningRelocaliser<VoxelType,IndexType>::score_pose(const ORUtils::SE3Pose& pose) const
{
#ifdef WITH_OPENCV
  // Make an OpenCV wrapper of the current depth image.
  m_view->depth->UpdateHostFromDevice();
  cv::Mat cvRealDepth(m_view->depth->noDims.y, m_view->depth->noDims.x, CV_32FC1, m_view->depth->GetData(MEMORYDEVICE_CPU));

  // Render a synthetic depth image of the scene from the suggested pose.
  ORFloatImage_Ptr synthDepth(new ORFloatImage(m_view->depth->noDims, true, true));
  DepthVisualisationUtil<VoxelType,IndexType>::generate_depth_from_voxels(
    synthDepth, m_scene, pose, m_view->calib.intrinsics_d, m_voxelRenderState,
    DepthVisualiser::DT_ORTHOGRAPHIC, m_visualisationEngine, m_depthVisualiser, m_settings
  );

  // Make an OpenCV wrapper of the synthetic depth image.
  cv::Mat cvSynthDepth(synthDepth->noDims.y, synthDepth->noDims.x, CV_32FC1, synthDepth->GetData(MEMORYDEVICE_CPU));

#if DEBUGGING
  // If we're debugging, show the real and synthetic depth images to the user (note that we need to convert them to unsigned chars for visualisation).
  // We don't use the OpenCV normalize function because we want consistent visualisations for different frames (even though there might be clamping).
  const float scaleFactor = 100.0f;
  cv::Mat cvRealDepthVis, cvSynthDepthVis;
  cvRealDepth.convertTo(cvRealDepthVis, CV_8U, scaleFactor);
  cvSynthDepth.convertTo(cvSynthDepthVis, CV_8U, scaleFactor);

  cv::imshow("Real Depth", cvRealDepthVis);
  cv::imshow("Synthetic Depth", cvSynthDepthVis);
#endif

  // Compute a binary mask showing which pixels are valid in both the real and synthetic depth images.
  cv::Mat cvRealMask = cvRealDepth > 0;
  cv::Mat cvSynthMask = cvSynthDepth > 0;
  cv::Mat cvCombinedMask = cvRealMask & cvSynthMask;

  // Compute the difference between the real and synthetic depth images, and mask it using the combined mask.
  cv::Mat cvDepthDiff, cvMaskedDepthDiff;
  cv::absdiff(cvRealDepth, cvSynthDepth, cvDepthDiff);
  cvDepthDiff.copyTo(cvMaskedDepthDiff, cvCombinedMask);

#if DEBUGGING
  // We need to convert the image for visualisation.
  cv::Mat cvMaskedDepthDiffVis;
  cvMaskedDepthDiff.convertTo(cvMaskedDepthDiffVis, CV_8U, scaleFactor);

  cv::imshow("Masked Depth Difference", cvMaskedDepthDiff);
  cv::waitKey(1);
#endif

  // Determine the mean depth difference for valid pixels in the real and synthetic depth images.
  cv::Scalar meanDepthDiff = cv::mean(cvMaskedDepthDiff, cvCombinedMask);

#if DEBUGGING
  std::cout << "\nMean Depth Difference: " << meanDepthDiff << std::endl;
#endif

  // Compute the fraction of the synthetic depth image that is valid.
  float validFraction = static_cast<float>(cv::countNonZero(cvSynthMask)) / (cvSynthMask.size().area());

#if DEBUGGING
  std::cout << "Valid Synthetic Depth Pixels: " << cv::countNonZero(cvSynthMask) << std::endl;
#endif

  return validFraction >= 0.1f ? static_cast<float>(meanDepthDiff(0)) : static_cast<float>(INT_MAX);
#else
  return 0.0f;
#endif
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::start_timer_nosync(AverageTimer& timer) const
{
  if(m_timersEnabled) timer.start_nosync();
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::start_timer_sync(AverageTimer& timer) const
{
  if(m_timersEnabled) timer.start_sync();
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::stop_timer_nosync(AverageTimer& timer) const
{
  if(m_timersEnabled) timer.stop_nosync();
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::stop_timer_sync(AverageTimer& timer) const
{
  if(m_timersEnabled) timer.stop_sync();
}

}
