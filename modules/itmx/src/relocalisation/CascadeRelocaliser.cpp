/**
 * itmx: CascadeRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "relocalisation/CascadeRelocaliser.h"

#include <iostream>
#include <stdexcept>

#include <tvgutil/filesystem/PathFinder.h>
#include <tvgutil/misc/SettingsContainer.h>
#include <tvgutil/timing/TimeUtil.h>

#include "persistence/PosePersister.h"

#define DEBUGGING 1

namespace itmx {

//#################### CONSTRUCTORS ####################

CascadeRelocaliser::CascadeRelocaliser(const std::vector<orx::Relocaliser_Ptr>& innerRelocalisers, const Settings_CPtr& settings)
: m_innerRelocalisers(innerRelocalisers),
  m_timerInitialRelocalisation("Initial Relocalisation"),
  m_timerRefinement("ICP Refinement"),
  m_timerRelocalisation("Relocalisation"),
  m_timerTraining("Training"),
  m_timerUpdate("Update")
{
  // Check that the cascade contains at least one relocaliser.
  if(innerRelocalisers.empty())
  {
    throw std::runtime_error("Error: Cannot create an empty cascade relocaliser");
  }

  // Configure the cascade relocaliser based on the settings that have been passed in.
  const static std::string settingsNamespace = "CascadeRelocaliser.";
  m_savePoses = settings->get_first_value<bool>(settingsNamespace + "saveRelocalisationPoses", false);
  m_saveTimes = settings->get_first_value<bool>(settingsNamespace + "saveRelocalisationTimes", false);
  m_timersEnabled = settings->get_first_value<bool>(settingsNamespace + "timersEnabled", false);

  for(size_t i = 0, size = m_innerRelocalisers.size(); i < size - 1; ++i)
  {
    m_fallbackThresholds.push_back(settings->get_first_value<float>(settingsNamespace + "fallbackThreshold" + boost::lexical_cast<std::string>(i)));
  }

  // Get the (global) experiment tag.
  const std::string experimentTag = settings->get_first_value<std::string>("experimentTag", tvgutil::TimeUtil::get_iso_timestamp());

  if(m_savePoses)
  {
    // Determine the directory to which to save the poses and make sure that it exists.
    m_posePathGenerator.reset(tvgutil::SequentialPathGenerator(tvgutil::find_subdir_from_executable("reloc_poses") / experimentTag));
    boost::filesystem::create_directories(m_posePathGenerator->get_base_dir());

    // Output the directory we're using (for debugging purposes).
    std::cout << "Saving relocalisation poses in: " << m_posePathGenerator->get_base_dir() << '\n';
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

CascadeRelocaliser::~CascadeRelocaliser()
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
    std::ofstream out(m_timersOutputFile.c_str());

    // Output the average durations.
    out << m_timerTraining.average_duration().count() << ' '
        << m_timerUpdate.average_duration().count() << ' '
        << m_timerInitialRelocalisation.average_duration().count() << ' '
        << m_timerRefinement.average_duration().count() << ' '
        << m_timerRelocalisation.average_duration().count() << '\n';
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void CascadeRelocaliser::finish_training()
{
  for(size_t i = 0, size = m_innerRelocalisers.size(); i < size; ++i)
  {
    m_innerRelocalisers[i]->finish_training();
  }
}

void CascadeRelocaliser::load_from_disk(const std::string& inputFolder)
{
  for(size_t i = 0, size = m_innerRelocalisers.size(); i < size; ++i)
  {
    m_innerRelocalisers[i]->load_from_disk(inputFolder);
  }
}

std::vector<orx::Relocaliser::Result>
CascadeRelocaliser::relocalise(const ORUChar4Image *colourImage, const ORFloatImage *depthImage, const Vector4f& depthIntrinsics) const
{
#if DEBUGGING && 0
  static int frameIdx = -1;
  ++frameIdx;
  std::cout << "---\nFrame Index: " << frameIdx << std::endl;
#endif

  start_timer_sync(m_timerRelocalisation);
  start_timer_nosync(m_timerInitialRelocalisation); // No need to synchronize the GPU again.

  // Try to relocalise using the first relocaliser in the cascade.
  std::vector<Result> initialRelocalisationResults = m_innerRelocalisers[0]->relocalise(colourImage, depthImage, depthIntrinsics);
  std::vector<Result> relocalisationResults = initialRelocalisationResults;

  stop_timer_sync(m_timerInitialRelocalisation);
  start_timer_nosync(m_timerRefinement); // No need to synchronize the GPU again.

#if DEBUGGING
  static std::vector<int> relocalisationCounts(m_innerRelocalisers.size());
#endif

  // For each other relocaliser in the cascade:
  for(size_t i = 1, size = m_innerRelocalisers.size(); i < size; ++i)
  {
    // If either there is no current best relocalisation result or it's not good enough:
    if(relocalisationResults.empty() || relocalisationResults[0].score > m_fallbackThresholds[i-1])
    {
#if DEBUGGING
      std::cout << "Using inner relocaliser " << i << " to relocalise: " << relocalisationCounts[i]++ << ".\n";
#endif

      // Try to relocalise using the new relocaliser.
      relocalisationResults = m_innerRelocalisers[i]->relocalise(colourImage, depthImage, depthIntrinsics);
    }
  }

  stop_timer_sync(m_timerRefinement);
  stop_timer_nosync(m_timerRelocalisation); // No need to synchronize the GPU again.

  // Save the best initial and refined poses if needed.
  if(m_savePoses)
  {     
    // Determine the best initial pose, namely the best pose (if any) returned by the first relocaliser in the cascade.
    Matrix4f initialPose;
    if(!initialRelocalisationResults.empty()) initialPose = initialRelocalisationResults[0].pose.GetInvM();
    else initialPose.setValues(std::numeric_limits<float>::quiet_NaN());

    // Determine the best refined pose, namely the pose (if any) whose score is lowest after running the entire cascade.
    Matrix4f refinedPose;
    if(!relocalisationResults.empty()) refinedPose = relocalisationResults[0].pose.GetInvM();
    else refinedPose.setValues(std::numeric_limits<float>::quiet_NaN());

    // Save both poses to disk.
    save_poses(initialPose, refinedPose);

    // Since we are saving the poses (i.e. we are running in evaluation mode), we force the quality of
    // every refined result to poor to prevent fusion whilst evaluating the testing sequence.
    for(size_t i = 0; i < relocalisationResults.size(); ++i)
    {
      relocalisationResults[i].quality = RELOCALISATION_POOR;
    }
  }

  return relocalisationResults;
}

void CascadeRelocaliser::reset()
{
  for(size_t i = 0, size = m_innerRelocalisers.size(); i < size; ++i)
  {
    m_innerRelocalisers[i]->reset();
  }
}

void CascadeRelocaliser::save_to_disk(const std::string& outputFolder) const
{
  for(size_t i = 0, size = m_innerRelocalisers.size(); i < size; ++i)
  {
    m_innerRelocalisers[i]->save_to_disk(outputFolder);
  }
}

void CascadeRelocaliser::train(const ORUChar4Image *colourImage, const ORFloatImage *depthImage,
                               const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose)
{
  start_timer_sync(m_timerTraining);

  for(size_t i = 0, size = m_innerRelocalisers.size(); i < size; ++i)
  {
    m_innerRelocalisers[i]->train(colourImage, depthImage, depthIntrinsics, cameraPose);
  }

  stop_timer_sync(m_timerTraining);
}

void CascadeRelocaliser::update()
{
  start_timer_sync(m_timerUpdate);

  for(size_t i = 0, size = m_innerRelocalisers.size(); i < size; ++i)
  {
    m_innerRelocalisers[i]->update();
  }

  stop_timer_sync(m_timerUpdate);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void CascadeRelocaliser::save_poses(const Matrix4f& relocalisedPose, const Matrix4f& refinedPose) const
{
  if(!m_savePoses) return;

  PosePersister::save_pose_on_thread(relocalisedPose, m_posePathGenerator->make_path("pose-%06i.reloc.txt"));
  PosePersister::save_pose_on_thread(refinedPose, m_posePathGenerator->make_path("pose-%06i.icp.txt"));
  m_posePathGenerator->increment_index();
}

}
