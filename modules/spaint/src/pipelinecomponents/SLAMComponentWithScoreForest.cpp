/**
 * spaint: SLAMComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SLAMComponentWithScoreForest.h"

using namespace InputSource;
using namespace ITMLib;
using namespace ORUtils;
using namespace RelocLib;

namespace spaint {

//#################### CONSTRUCTORS ####################

SLAMComponentWithScoreForest::SLAMComponentWithScoreForest(const SLAMContext_Ptr& context, const std::string& sceneID, const ImageSourceEngine_Ptr& imageSourceEngine,
                             TrackerType trackerType, const std::vector<std::string>& trackerParams, MappingMode mappingMode, TrackingMode trackingMode)
: SLAMComponent(context, sceneID, imageSourceEngine, trackerType, trackerParams, mappingMode, trackingMode)
{
}

//#################### DESTRUCTOR ####################
SLAMComponentWithScoreForest::~SLAMComponentWithScoreForest() {}

//#################### PROTECTED MEMBER FUNCTIONS ####################

SLAMComponent::TrackingResult SLAMComponentWithScoreForest::process_relocalisation(TrackingResult trackingResult)
{
//  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);
  return TrackingResult::TRACKING_GOOD;
}

}
