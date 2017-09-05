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

#if 0
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
#endif

  static int count = 0;
  if(count > 0 && count % 10 == 0)
  {
    // Pick a relative pose to optimise.
    // TODO
#if 0
    size_t i = 0, j = 1;
#endif
    const std::string sceneI = sceneIDs[0];
    const std::string sceneJ = sceneIDs[1];

    // Try to relocalise the current frame of each against the other.
    Relocaliser_CPtr relocaliserI = m_context->get_relocaliser(sceneI);
    Relocaliser_CPtr relocaliserJ = m_context->get_relocaliser(sceneJ);
    View_CPtr viewI = m_context->get_slam_state(sceneI)->get_view();
    View_CPtr viewJ = m_context->get_slam_state(sceneJ)->get_view();
    ORUtils::SE3Pose localPoseI = m_context->get_slam_state(sceneI)->get_pose();
    ORUtils::SE3Pose localPoseJ = m_context->get_slam_state(sceneJ)->get_pose();

#if 0
    std::vector<ORUtils::SE3Pose>& relativePosesIJ = relativePoses[std::make_pair(i, j)];
#endif

    std::cout << "Attempting to relocalise " << sceneJ << " against " << sceneI << '\n';
    boost::optional<Relocaliser::Result> resultIJ = relocaliserI->relocalise(viewJ->rgb, viewJ->depth, viewJ->calib.intrinsics_d.projectionParamsSimple.all);
    ORUtils::SE3Pose relativePoseIJ;
    if(resultIJ && resultIJ->quality == Relocaliser::RELOCALISATION_GOOD)
    {
      //relativePoseIJ = ORUtils::SE3Pose(localPoseJ.GetInvM() * resultIJ->pose.GetM());
      relativePoseIJ = ORUtils::SE3Pose(resultIJ->pose.GetInvM() * localPoseJ.GetM());
      std::cout << "Succeeded!\n";
      std::cout << relativePoseIJ.GetM() << '\n';// << relativePoseIJ.GetInvM() << '\n';
    }

    std::cout << "Attempting to relocalise " << sceneI << " against " << sceneJ << '\n';
    boost::optional<Relocaliser::Result> resultJI = relocaliserJ->relocalise(viewI->rgb, viewI->depth, viewI->calib.intrinsics_d.projectionParamsSimple.all);
    ORUtils::SE3Pose relativePoseJI;
    if(resultJI && resultJI->quality == Relocaliser::RELOCALISATION_GOOD)
    {
      //relativePoseJI = ORUtils::SE3Pose(localPoseI.GetInvM() * resultJI->pose.GetM());
      relativePoseJI = ORUtils::SE3Pose(resultJI->pose.GetInvM() * localPoseI.GetM());
      std::cout << "Succeeded!\n";
      std::cout << relativePoseJI.GetM() << '\n';// << relativePoseJI.GetInvM() << '\n';
    }

    if(resultIJ && resultIJ->quality == Relocaliser::RELOCALISATION_GOOD && resultJI && resultJI->quality == Relocaliser::RELOCALISATION_GOOD &&
       GeometryUtil::poses_are_similar(relativePoseIJ, ORUtils::SE3Pose(relativePoseJI.GetInvM())))
    {
      std::cout << "Similar poses\n";
      m_context->add_relative_transform_sample(sceneI, sceneJ, relativePoseIJ);
      m_context->add_relative_transform_sample(sceneJ, sceneI, relativePoseJI);
#if 0
      relativePosesIJ.push_back(relativePoseIJ);
      relativePosesIJ.push_back(ORUtils::SE3Pose(relativePoseJI.GetInvM()));
#endif
    }

#if 0
    if(!relativePosesIJ.empty())
    {
      std::cout << "Blended:\n";
      ORUtils::SE3Pose blended = GeometryUtil::blend_poses(relativePosesIJ);
      std::cout << blended.GetM() << '\n' << blended.GetInvM() << '\n';
    }
#endif
  }
  ++count;
}

}
