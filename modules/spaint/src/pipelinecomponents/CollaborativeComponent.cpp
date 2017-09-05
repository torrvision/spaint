/**
 * spaint: CollaborativeComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "pipelinecomponents/CollaborativeComponent.h"

#include <itmx/geometry/GeometryUtil.h>
#include <itmx/relocalisation/Relocaliser.h>
using namespace itmx;

namespace spaint {

//#################### CONSTRUCTORS ####################

CollaborativeComponent::CollaborativeComponent(const CollaborativeContext_Ptr& context)
: m_context(context)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void CollaborativeComponent::run_collaborative_pose_estimation()
{
  std::vector<std::string> sceneIDs = m_context->get_scene_ids();

  static std::map<std::pair<size_t,size_t>,std::vector<ORUtils::SE3Pose> > relativePoses;
  static bool done = false;
  if(!done)
  {
    for(size_t i = 0, size = sceneIDs.size(); i < size - 1; ++i)
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
    Relocaliser_CPtr relocaliserI = m_context->get_relocaliser(sceneIDs[i]);
    Relocaliser_CPtr relocaliserJ = m_context->get_relocaliser(sceneIDs[j]);
    View_CPtr viewI = m_context->get_slam_state(sceneIDs[i])->get_view();
    View_CPtr viewJ = m_context->get_slam_state(sceneIDs[j])->get_view();
    ORUtils::SE3Pose localPoseI = m_context->get_slam_state(sceneIDs[i])->get_pose();
    ORUtils::SE3Pose localPoseJ = m_context->get_slam_state(sceneIDs[j])->get_pose();

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
}

}
