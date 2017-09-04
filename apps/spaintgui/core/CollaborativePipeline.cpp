/**
 * spaintgui: CollaborativePipeline.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "CollaborativePipeline.h"
using namespace itmx;
using namespace spaint;

//#################### CONSTRUCTORS ####################

CollaborativePipeline::CollaborativePipeline(const Settings_Ptr& settings, const std::string& resourcesDir,
                                             const std::vector<CompositeImageSourceEngine_Ptr>& imageSourceEngines,
                                             const std::vector<std::string>& trackerConfigs,
                                             const std::vector<SLAMComponent::MappingMode>& mappingModes,
                                             const std::vector<SLAMComponent::TrackingMode>& trackingModes,
                                             const FiducialDetector_CPtr& fiducialDetector, bool detectFiducials)
  // Note: A minimum of 2 labels is required (background and foreground).
: MultiScenePipeline("collaborative", settings, resourcesDir, 2)
{
  for(size_t i = 0, size = imageSourceEngines.size(); i < size; ++i)
  {
    const std::string sceneID = i == 0 ? Model::get_world_scene_id() : "Agent" + boost::lexical_cast<std::string>(i);
    m_slamComponents[sceneID].reset(
      new SLAMComponent(m_model, sceneID, imageSourceEngines[i], trackerConfigs[i], mappingModes[i], trackingModes[i], fiducialDetector, detectFiducials)
    );
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool CollaborativePipeline::run_main_section()
{
  if(!MultiScenePipeline::run_main_section()) return false;

  static std::map<std::pair<size_t,size_t>,ORUtils::SE3Pose> relativePoses;
  static bool done = false;
  if(!done)
  {
    for(size_t i = 0, size = m_slamComponents.size(); i < size - 1; ++i)
    {
      for(size_t j = i + 1; j < size; ++j)
      {
        Matrix4f m;
        m.setIdentity();
        relativePoses[std::make_pair(i,j)].SetM(m);
      }
    }
    done = true;
  }

  static int count = 0;
  if(count > 0 && count % 10 == 0)
  {
    // Pick a relative pose to optimise.
    // TODO
    size_t i = 0, j = 1;

    // Try to relocalise the current frame of each against the other.
    std::vector<std::string> sceneIDs;
    for(std::map<std::string,SLAMComponent_Ptr>::const_iterator it = m_slamComponents.begin(), iend = m_slamComponents.end(); it != iend; ++it)
    {
      sceneIDs.push_back(it->first);
    }

    Relocaliser_CPtr relocaliserI = m_model->get_relocaliser(sceneIDs[i]);
    Relocaliser_CPtr relocaliserJ = m_model->get_relocaliser(sceneIDs[j]);
    View_CPtr viewI = m_model->get_slam_state(sceneIDs[i])->get_view();
    View_CPtr viewJ = m_model->get_slam_state(sceneIDs[j])->get_view();
    ORUtils::SE3Pose localPoseI = m_model->get_slam_state(sceneIDs[i])->get_pose();
    ORUtils::SE3Pose localPoseJ = m_model->get_slam_state(sceneIDs[j])->get_pose();

    std::cout << "Attempting to relocalise " << j << " against " << i << '\n';
    boost::optional<Relocaliser::Result> resultIJ = relocaliserI->relocalise(viewJ->rgb, viewJ->depth, viewJ->calib.intrinsics_d.projectionParamsSimple.all);
    if(resultIJ && resultIJ->quality == Relocaliser::RELOCALISATION_GOOD)
    {
      ORUtils::SE3Pose relativePoseIJ(resultIJ->pose.GetM() * localPoseJ.GetInvM());
      std::cout << "Succeeded!\n";
      std::cout << relativePoseIJ.GetM() << '\n' << relativePoseIJ.GetInvM() << '\n';
    }

    std::cout << "Attempting to relocalise " << i << " against " << j << '\n';
    boost::optional<Relocaliser::Result> resultJI = relocaliserJ->relocalise(viewI->rgb, viewI->depth, viewI->calib.intrinsics_d.projectionParamsSimple.all);
    if(resultJI && resultJI->quality == Relocaliser::RELOCALISATION_GOOD)
    {
      ORUtils::SE3Pose relativePoseJI(resultJI->pose.GetM() * localPoseI.GetInvM());
      std::cout << "Succeeded!\n";
      std::cout << relativePoseJI.GetM() << '\n' << relativePoseJI.GetInvM() << '\n';
    }
  }
  ++count;

  return true;
}

void CollaborativePipeline::set_mode(Mode mode)
{
  // The only supported mode.
  m_mode = MODE_NORMAL;
}
