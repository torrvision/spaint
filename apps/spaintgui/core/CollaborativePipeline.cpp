/**
 * spaintgui: CollaborativePipeline.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "CollaborativePipeline.h"
using namespace spaint;

#include <itmx/geometry/GeometryUtil.h>
using namespace itmx;

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
  bool result = MultiScenePipeline::run_main_section();

  static std::map<std::pair<size_t,size_t>,std::vector<ORUtils::SE3Pose> > relativePoses;
  static bool done = false;
  if(!done)
  {
    for(size_t i = 0, size = m_slamComponents.size(); i < size - 1; ++i)
    {
      for(size_t j = i + 1; j < size; ++j)
      {
        relativePoses[std::make_pair(i, j)] = std::vector<ORUtils::SE3Pose>();
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

    std::vector<ORUtils::SE3Pose>& relativePosesIJ = relativePoses[std::make_pair(i, j)];

    std::cout << "Attempting to relocalise " << j << " against " << i << '\n';
    boost::optional<Relocaliser::Result> resultIJ = relocaliserI->relocalise(viewJ->rgb, viewJ->depth, viewJ->calib.intrinsics_d.projectionParamsSimple.all);
    ORUtils::SE3Pose relativePoseIJ;
    if(resultIJ && resultIJ->quality == Relocaliser::RELOCALISATION_GOOD)
    {
      //relativePoseIJ = ORUtils::SE3Pose(resultIJ->pose.GetM() * localPoseJ.GetInvM());
      relativePoseIJ = ORUtils::SE3Pose(localPoseJ.GetInvM() * resultIJ->pose.GetM());
      std::cout << "Succeeded!\n";
      std::cout << relativePoseIJ.GetM() << '\n' << relativePoseIJ.GetInvM() << '\n';
    }

    std::cout << "Attempting to relocalise " << i << " against " << j << '\n';
    boost::optional<Relocaliser::Result> resultJI = relocaliserJ->relocalise(viewI->rgb, viewI->depth, viewI->calib.intrinsics_d.projectionParamsSimple.all);
    ORUtils::SE3Pose relativePoseJI;
    if(resultJI && resultJI->quality == Relocaliser::RELOCALISATION_GOOD)
    {
      //relativePoseJI = ORUtils::SE3Pose(resultJI->pose.GetM() * localPoseI.GetInvM());
      relativePoseJI = ORUtils::SE3Pose(localPoseI.GetInvM() * resultJI->pose.GetM());
      std::cout << "Succeeded!\n";
      std::cout << relativePoseJI.GetM() << '\n' << relativePoseJI.GetInvM() << '\n';
    }

    if(resultIJ && resultIJ->quality == Relocaliser::RELOCALISATION_GOOD && resultJI && resultJI->quality == Relocaliser::RELOCALISATION_GOOD &&
       GeometryUtil::poses_are_similar(relativePoseIJ, ORUtils::SE3Pose(relativePoseJI.GetInvM())))
    {
      std::cout << "Similar poses\n";
      relativePosesIJ.push_back(relativePoseIJ);
      relativePosesIJ.push_back(ORUtils::SE3Pose(relativePoseJI.GetInvM()));
    }

    if(!relativePosesIJ.empty())
    {
      std::cout << "Blended:\n";
      ORUtils::SE3Pose blended = GeometryUtil::blend_poses(relativePosesIJ);
      std::cout << blended.GetM() << '\n' << blended.GetInvM() << '\n';
    }
  }
  ++count;

  return result;
}

void CollaborativePipeline::set_mode(Mode mode)
{
  // The only supported mode.
  m_mode = MODE_NORMAL;
}
