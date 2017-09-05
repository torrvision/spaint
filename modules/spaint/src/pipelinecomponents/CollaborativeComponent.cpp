/**
 * spaint: CollaborativeComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "pipelinecomponents/CollaborativeComponent.h"

#include <itmx/geometry/GeometryUtil.h>
#include <itmx/relocalisation/Relocaliser.h>
using namespace itmx;

#include <tvgutil/numbers/RandomNumberGenerator.h>
using namespace tvgutil;

namespace spaint {

//#################### CONSTRUCTORS ####################

CollaborativeComponent::CollaborativeComponent(const CollaborativeContext_Ptr& context)
: m_context(context)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void CollaborativeComponent::run_collaborative_pose_estimation()
{
  std::vector<std::string> sceneIDs = m_context->get_scene_ids();
  int sceneCount = static_cast<int>(sceneIDs.size());

  static RandomNumberGenerator rng(12345);

  static int count = 0;
  if(count > 0 && count % 10 == 0)
  {
    // Pick a pair of scenes to relocalise with respect to each other.
    int i = rng.generate_int_from_uniform(0, sceneCount - 1);
    int j = (i + rng.generate_int_from_uniform(1, sceneCount - 1)) % sceneCount;
    const std::string sceneI = sceneIDs[i];
    const std::string sceneJ = sceneIDs[j];

    // Try to relocalise the current frame of each against the other.
    Relocaliser_CPtr relocaliserI = m_context->get_relocaliser(sceneI);
    Relocaliser_CPtr relocaliserJ = m_context->get_relocaliser(sceneJ);
    View_CPtr viewI = m_context->get_slam_state(sceneI)->get_view();
    View_CPtr viewJ = m_context->get_slam_state(sceneJ)->get_view();
    ORUtils::SE3Pose localPoseI = m_context->get_slam_state(sceneI)->get_pose();
    ORUtils::SE3Pose localPoseJ = m_context->get_slam_state(sceneJ)->get_pose();

    std::cout << "Attempting to relocalise " << sceneJ << " against " << sceneI << '\n';
    boost::optional<Relocaliser::Result> resultIJ = relocaliserI->relocalise(viewJ->rgb, viewJ->depth, viewJ->calib.intrinsics_d.projectionParamsSimple.all);
    ORUtils::SE3Pose relativePoseIJ;
    if(resultIJ && resultIJ->quality == Relocaliser::RELOCALISATION_GOOD)
    {
      relativePoseIJ = ORUtils::SE3Pose(resultIJ->pose.GetInvM() * localPoseJ.GetM());
      std::cout << "Succeeded!\n";
      std::cout << relativePoseIJ.GetM() << '\n';// << relativePoseIJ.GetInvM() << '\n';
    }

    std::cout << "Attempting to relocalise " << sceneI << " against " << sceneJ << '\n';
    boost::optional<Relocaliser::Result> resultJI = relocaliserJ->relocalise(viewI->rgb, viewI->depth, viewI->calib.intrinsics_d.projectionParamsSimple.all);
    ORUtils::SE3Pose relativePoseJI;
    if(resultJI && resultJI->quality == Relocaliser::RELOCALISATION_GOOD)
    {
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
    }
  }
  ++count;
}

}
