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

namespace itmx {

//#################### CONSTRUCTORS ####################

CascadeRelocaliser::CascadeRelocaliser(const orx::Relocaliser_Ptr& innerRelocaliser_Fast, const orx::Relocaliser_Ptr& innerRelocaliser_Intermediate,
                                       const orx::Relocaliser_Ptr& innerRelocaliser_Full, const Settings_CPtr& settings)
: m_innerRelocaliser_Fast(innerRelocaliser_Fast),
  m_innerRelocaliser_Intermediate(innerRelocaliser_Intermediate),
  m_innerRelocaliser_Full(innerRelocaliser_Full),
  m_settings(settings),
  m_timerInitialRelocalisation("Initial Relocalisation"),
  m_timerRefinement("ICP Refinement"),
  m_timerRelocalisation("Relocalisation"),
  m_timerTraining("Training"),
  m_timerUpdate("Update")
{
  // Configure the relocaliser based on the settings that have been passed in.
  const static std::string settingsNamespace = "CascadeRelocaliser.";
  m_savePoses = m_settings->get_first_value<bool>(settingsNamespace + "saveRelocalisationPoses", false);
  m_saveTimes = m_settings->get_first_value<bool>(settingsNamespace + "saveRelocalisationTimes", false);
  m_relocaliserThresholdScore_Fast = settings->get_first_value<float>(settingsNamespace + "relocaliserThresholdScore_Fast", 0.05f);
  m_relocaliserThresholdScore_Intermediate = settings->get_first_value<float>(settingsNamespace + "relocaliserThresholdScore_Intermediate", 0.05f);
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

  if(m_saveTimes)
  {
    // Forcefully enable timers.
    m_timersEnabled = true;

    // Make sure the directory where we want to save the relocalisation times exists.
    boost::filesystem::path timersOutputFolder(tvgutil::find_subdir_from_executable("reloc_times"));
    boost::filesystem::create_directories(timersOutputFolder);

    // Prepare the output filename.
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
    std::cout << "Saving relocalisation average times in: " << m_timersOutputFile << "\n";
    std::ofstream out(m_timersOutputFile.c_str());

    // Output the average durations.
    out << m_timerTraining.average_duration().count() << " "
        << m_timerUpdate.average_duration().count() << " "
        << m_timerInitialRelocalisation.average_duration().count() << " "
        << m_timerRefinement.average_duration().count() << " "
        << m_timerRelocalisation.average_duration().count() << "\n";
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void CascadeRelocaliser::finish_training()
{
  m_innerRelocaliser_Full->finish_training();
}

void CascadeRelocaliser::load_from_disk(const std::string& inputFolder)
{
  m_innerRelocaliser_Full->load_from_disk(inputFolder);
}

std::vector<orx::Relocaliser::Result>
CascadeRelocaliser::relocalise(const ORUChar4Image *colourImage, const ORFloatImage *depthImage, const Vector4f& depthIntrinsics) const
{
#if DEBUGGING
  static int frameIdx = -1;
  ++frameIdx;
  std::cout << "---\nFrame Index: " << frameIdx << std::endl;
#endif

  std::vector<Result> relocalisationResults_Fast;
  std::vector<Result> relocalisationResults;

  // Main timing section.
  start_timer(m_timerRelocalisation);

  // 1. Run the fast inner relocaliser.
  start_timer(m_timerInitialRelocalisation, false); // No need to synchronize the GPU again.
  relocalisationResults = relocalisationResults_Fast = m_innerRelocaliser_Fast->relocalise(colourImage, depthImage, depthIntrinsics);
  stop_timer(m_timerInitialRelocalisation);

  // We time the following as "refinement".
  start_timer(m_timerRefinement, false); // No need to synchronize the GPU again.

  // If the fast relocaliser failed to relocalise or returned a bad relocalisation, then use the intermediate relocaliser (if available).
  if(m_innerRelocaliser_Intermediate && (relocalisationResults.empty() || relocalisationResults[0].score > m_relocaliserThresholdScore_Fast))
  {
#if DEBUGGING
    static int intermediateRelocalisationsCount = 0;
    std::cout << "Using intermediate relocaliser to relocalise: " << intermediateRelocalisationsCount++ << ".\n";
#endif

    relocalisationResults = m_innerRelocaliser_Intermediate->relocalise(colourImage, depthImage, depthIntrinsics);
  }

  // Finally, run the normal relocaliser if all else failed.
  if(m_innerRelocaliser_Full && (relocalisationResults.empty() || relocalisationResults[0].score > m_relocaliserThresholdScore_Intermediate))
  {
#if DEBUGGING
    static int fullRelocalisationsCount = 0;
    std::cout << "Using full relocaliser to relocalise: " << fullRelocalisationsCount++ << ".\n";
#endif

    relocalisationResults = m_innerRelocaliser_Full->relocalise(colourImage, depthImage, depthIntrinsics);
  }

  // Done.
  stop_timer(m_timerRefinement);
  stop_timer(m_timerRelocalisation, false); // No need to synchronize the GPU again.

  // Save the best initial and refined poses if needed.
  if(m_savePoses)
  {     
    // The initial pose is the best one returned by the fast relocaliser (if any).
    Matrix4f initialPose;
    if(!relocalisationResults_Fast.empty())
    {
      initialPose = relocalisationResults_Fast[0].pose.GetInvM();
    }
    else
    {
      initialPose.setValues(std::numeric_limits<float>::quiet_NaN());
    }

    // The best refined pose is the pose (if any) whose score is lowest after running all the cascade.
    Matrix4f refinedPose;
    if(!relocalisationResults.empty())
    {
      refinedPose = relocalisationResults[0].pose.GetInvM();
    }
    else
    {
      refinedPose.setValues(std::numeric_limits<float>::quiet_NaN());
    }

    // Actually save the poses.
    save_poses(initialPose, refinedPose);

    // Since we are saving the poses (i.e. we are running in evaluation mode), we force the quality of
    // every refined result to POOR to prevent fusion whilst evaluating the testing sequence.
    for(size_t i = 0; i < relocalisationResults.size(); ++i)
    {
      relocalisationResults[i].quality = RELOCALISATION_POOR;
    }
  }

  return relocalisationResults;
}

void CascadeRelocaliser::reset()
{
  m_innerRelocaliser_Full->reset();
}

void CascadeRelocaliser::save_to_disk(const std::string& outputFolder) const
{
  m_innerRelocaliser_Full->save_to_disk(outputFolder);
}

void CascadeRelocaliser::train(const ORUChar4Image *colourImage, const ORFloatImage *depthImage,
                               const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose)
{
  start_timer(m_timerTraining);
  m_innerRelocaliser_Full->train(colourImage, depthImage, depthIntrinsics, cameraPose);
  stop_timer(m_timerTraining);
}

void CascadeRelocaliser::update()
{
  start_timer(m_timerUpdate);
  m_innerRelocaliser_Full->update();
  stop_timer(m_timerUpdate);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void CascadeRelocaliser::save_poses(const Matrix4f& relocalisedPose, const Matrix4f& refinedPose) const
{
  if(!m_savePoses) return;

  PosePersister::save_pose_on_thread(relocalisedPose, m_posePathGenerator->make_path("pose-%06i.reloc.txt"));
  PosePersister::save_pose_on_thread(refinedPose, m_posePathGenerator->make_path("pose-%06i.icp.txt"));
  m_posePathGenerator->increment_index();
}

void CascadeRelocaliser::start_timer(AverageTimer& timer, bool cudaSynchronize) const
{
  if(!m_timersEnabled) return;

#ifdef WITH_CUDA
  if(cudaSynchronize)
  {
    ORcudaSafeCall(cudaDeviceSynchronize());
  }
#endif

  timer.start();
}

void CascadeRelocaliser::stop_timer(AverageTimer& timer, bool cudaSynchronize) const
{
  if(!m_timersEnabled) return;

#ifdef WITH_CUDA
  if(cudaSynchronize)
  {
    ORcudaSafeCall(cudaDeviceSynchronize());
  }
#endif

  timer.stop();
}

}
