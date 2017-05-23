/**
 * spaint: SLAMComponentWithScoreForest.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SLAMComponentWithScoreForest.h"

#include "ITMLib/Trackers/ITMTrackerFactory.h"
#include "ITMLib/Utils/ITMProjectionUtils.h"

#include <itmx/relocalisation/ICPRefiningRelocaliser.h>
#include <itmx/relocalisation/RelocaliserFactory.h>

#include "tvgutil/filesystem/PathFinder.h"
#include "tvgutil/misc/GlobalParameters.h"
#include "tvgutil/timing/TimeUtil.h"

#include <boost/lexical_cast.hpp>

#include <grove/relocalisation/ScoreRelocaliserFactory.h>

// Whether to enable VERBOSE timers, printing the time spent in each relocalisation phase, for each frame.
//#define ENABLE_VERBOSE_TIMERS

// Whether or not to save the modes associated to a certain set of forest leaves after the adaptation.
//#define SAVE_LEAF_MODES

// Whether or not to show the correspondences used to perform P-RANSAC.
//#define SHOW_RANSAC_CORRESPONDENCES

#ifdef SHOW_RANSAC_CORRESPONDENCES
#include <itmx/MemoryBlockFactory.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace bf = boost::filesystem;
using namespace InputSource;
using namespace ITMLib;
using namespace itmx;
using namespace ORUtils;
using namespace grove;
using namespace tvgutil;

namespace spaint {

//#################### CONSTRUCTORS ####################

SLAMComponentWithScoreForest::SLAMComponentWithScoreForest(const SLAMContext_Ptr &context,
                                                           const std::string &sceneID,
                                                           const ImageSourceEngine_Ptr &imageSourceEngine,
                                                           const std::string &trackerConfig,
                                                           MappingMode mappingMode,
                                                           TrackingMode trackingMode)
  : SLAMComponent(context, sceneID, imageSourceEngine, trackerConfig, mappingMode, trackingMode)
{
}

//#################### DESTRUCTOR ####################
SLAMComponentWithScoreForest::~SLAMComponentWithScoreForest() {}

} // namespace spaint
