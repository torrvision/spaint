/**
 * itmx: TrackerFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "trackers/TrackerFactory.h"

#include <boost/serialization/extended_type_info.hpp>
#include <boost/serialization/singleton.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include <ITMLib/Trackers/ITMTrackerFactory.h>
using namespace ITMLib;

#include <tvgutil/filesystem/PathFinder.h>
#include <tvgutil/persistence/PropertyUtil.h>
using namespace tvgutil;

#include "trackers/RemoteTracker.h"

#ifdef WITH_OVR
#include "trackers/RiftTracker.h"
#endif

#ifdef WITH_VICON
#include "trackers/RobustViconTracker.h"
#include "trackers/ViconTracker.h"
#endif

namespace itmx {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

Tracker_Ptr TrackerFactory::make_tracker_from_file(const std::string& trackerConfigFilename, bool trackSurfels,
                                                   const Vector2i& rgbImageSize, const Vector2i& depthImageSize,
                                                   const LowLevelEngine_CPtr& lowLevelEngine, const IMUCalibrator_Ptr& imuCalibrator,
                                                   const Settings_CPtr& settings, FallibleTracker*& fallibleTracker,
                                                   const MappingServer_Ptr& mappingServer, NestingFlag nestingFlag)
{
  Tree tree = PropertyUtil::load_properties_from_xml_file(trackerConfigFilename);
  Tree trackerTree = tree.get_child("tracker");
  return make_tracker(trackerTree, trackSurfels, rgbImageSize, depthImageSize, lowLevelEngine, imuCalibrator, settings, fallibleTracker, mappingServer, nestingFlag);
}

Tracker_Ptr TrackerFactory::make_tracker_from_string(const std::string& trackerConfig, bool trackSurfels,
                                                     const Vector2i& rgbImageSize, const Vector2i& depthImageSize,
                                                     const LowLevelEngine_CPtr& lowLevelEngine, const IMUCalibrator_Ptr& imuCalibrator,
                                                     const Settings_CPtr& settings, FallibleTracker*& fallibleTracker,
                                                     const MappingServer_Ptr& mappingServer, NestingFlag nestingFlag)
{
  Tree tree = PropertyUtil::load_properties_from_xml_string(trackerConfig);
  Tree trackerTree = tree.get_child("tracker");
  return make_tracker(trackerTree, trackSurfels, rgbImageSize, depthImageSize, lowLevelEngine, imuCalibrator, settings, fallibleTracker, mappingServer, nestingFlag);
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

Tracker_Ptr TrackerFactory::make_simple_tracker(std::string trackerType, std::string trackerParams, bool trackSurfels,
                                                const Vector2i& rgbImageSize, const Vector2i& depthImageSize,
                                                const LowLevelEngine_CPtr& lowLevelEngine, const IMUCalibrator_Ptr& imuCalibrator,
                                                const Settings_CPtr& settings, FallibleTracker*& fallibleTracker,
                                                const MappingServer_Ptr& mappingServer, NestingFlag nestingFlag)
{
  ITMTracker *tracker = NULL;

  // If the user wants to construct a non-InfiniTAM tracker (e.g. a Rift or Vicon tracker),
  // try to construct it, falling back on the InfiniTAM tracker if it doesn't work.
  if(trackerType == "remote")
  {
    tracker = new RemoteTracker(mappingServer, boost::lexical_cast<int>(trackerParams));
  }
  else if(trackerType == "rift")
  {
#ifdef WITH_OVR
    // If the Rift isn't available, make sure that we're not trying to use the Rift tracker.
    if(ovrHmd_Detect() == 0)
    {
      trackerType = "infinitam";
      trackerParams = "";
    }
    else
    {
      tracker = new RiftTracker;
    }
#else
    // If we haven't built with Rift support, make sure that we're not trying to use the Rift tracker.
    trackerType = "infinitam";
    trackerParams = "";
#endif
  }
  else if(trackerType == "robustvicon")
  {
#ifdef WITH_VICON
    fallibleTracker = new RobustViconTracker(trackerParams, "kinect", rgbImageSize, depthImageSize, settings, lowLevelEngine);
    tracker = fallibleTracker;
#else
    // If we haven't built with Vicon support, make sure that we're not trying to use the Vicon tracker.
    trackerType = "infinitam";
    trackerParams = "";
#endif
  }
  else if(trackerType == "vicon")
  {
#ifdef WITH_VICON
    fallibleTracker = new ViconTracker(trackerParams, "kinect");
    tracker = fallibleTracker;
#else
    // If we haven't built with Vicon support, make sure that we're not trying to use the Vicon tracker.
    trackerType = "infinitam";
    trackerParams = "";
#endif
  }

  // If we reach this point, either we were already trying to use the InfiniTAM tracker, or construction
  // of a different tracker failed. In either case, we will now use the InfiniTAM tracker.
  if(trackerType == "infinitam")
  {
    // If no parameters were specified for the InfiniTAM tracker, use the default ones. The defaults
    // differ depending on whether we're tracking against the voxel or the surfel scene.
    if(trackerParams == "")
    {
      if(trackSurfels) trackerParams = "type=extended,levels=rrbb,minstep=1e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=20,tukeyCutOff=8,framesToSkip=0,framesToWeight=1,failureDec=20.0";
      else trackerParams = "type=extended,levels=rrbb,minstep=1e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=20,tukeyCutOff=8,framesToSkip=20,framesToWeight=50,failureDec=20.0";
    }

    // We use the InfiniTAM tracker factory to actually construct the tracker.
    tracker = ITMTrackerFactory::Instance().Make(
      settings->deviceType, trackerParams.c_str(), rgbImageSize, depthImageSize,
      lowLevelEngine.get(), imuCalibrator.get(), &settings->sceneParams
    );
  }

  // Finally, return the constructed tracker, making sure to prevent double deletion if it will be added to a composite.
  return nestingFlag == NESTED ? Tracker_Ptr(tracker, boost::serialization::null_deleter()) : Tracker_Ptr(tracker);
}

Tracker_Ptr TrackerFactory::make_tracker(const Tree& trackerTree, bool trackSurfels,
                                         const Vector2i& rgbImageSize, const Vector2i& depthImageSize,
                                         const LowLevelEngine_CPtr& lowLevelEngine, const IMUCalibrator_Ptr& imuCalibrator,
                                         const Settings_CPtr& settings, FallibleTracker*& fallibleTracker,
                                         const MappingServer_Ptr& mappingServer, NestingFlag nestingFlag)
{
  std::string trackerType, trackerParams;
  PropertyUtil::get_required_property(trackerTree, "<xmlattr>.type", trackerType);
  PropertyUtil::get_optional_property(trackerTree, "params", trackerParams);

  if(trackerType == "composite")
  {
    // Determine the policy to use for the composite tracker.
    std::string trackerPolicy;
    PropertyUtil::get_optional_property(trackerTree, "<xmlattr>.policy", trackerPolicy);

    ITMCompositeTracker::Policy policy = ITMCompositeTracker::POLICY_REFINE;
    if(trackerPolicy == "sequential") policy = ITMCompositeTracker::POLICY_SEQUENTIAL;
    else if(trackerPolicy == "stoponfirstsuccess") policy = ITMCompositeTracker::POLICY_STOP_ON_FIRST_SUCCESS;

    // Construct all of the subsidiary trackers and add them to the composite tracker.
    ITMCompositeTracker *compositeTracker = new ITMCompositeTracker(policy);
    for(boost::property_tree::ptree::const_iterator it = trackerTree.begin(), iend = trackerTree.end(); it != iend; ++it)
    {
      if(it->first != "tracker") continue;

      Tracker_Ptr nestedTracker = make_tracker(
        it->second, trackSurfels, rgbImageSize, depthImageSize,
        lowLevelEngine, imuCalibrator, settings, fallibleTracker,
        mappingServer, NESTED
      );

      compositeTracker->AddTracker(nestedTracker.get());
    }

    // Return the composite tracker, making sure to prevent double deletion if it will itself be added to another composite.
    return nestingFlag == NESTED ? Tracker_Ptr(compositeTracker, boost::serialization::null_deleter()) : Tracker_Ptr(compositeTracker);
  }
  else if(trackerType == "import")
  {
    // Use the tracker parameters to determine the file from which to import the tracker configuration.
    // The parameters can either be of the form "builtin:<Name>", or be an absolute path.
    std::string trackerConfigFilename;
    const std::string builtinLabel = "builtin:";
    if(trackerParams.length() >= builtinLabel.length() && trackerParams.substr(0, builtinLabel.length()) == builtinLabel)
    {
      // If we're loading a built-in tracker configuration, determine its path within the resources/trackerconfigs subdirectory of the calling application.
      // FIXME: It might be better to pass the trackerconfigs directory in as a parameter here.
      const std::string builtinName = trackerParams.substr(builtinLabel.length());
      boost::filesystem::path builtinPath = find_subdir_from_executable("resources") / "trackerconfigs" / (builtinName + ".xml");
      trackerConfigFilename = builtinPath.string();
    }
    else
    {
      // If we're not loading a built-in, the parameters specify the relevant filename directly.
      trackerConfigFilename = trackerParams;
    }

    // Import the tracker configuration from the specified file.
    return make_tracker_from_file(
      trackerConfigFilename, trackSurfels, rgbImageSize, depthImageSize, lowLevelEngine,
      imuCalibrator, settings, fallibleTracker, mappingServer, nestingFlag
    );
  }
  else
  {
    return make_simple_tracker(
      trackerType, trackerParams, trackSurfels, rgbImageSize, depthImageSize, lowLevelEngine,
      imuCalibrator, settings, fallibleTracker, mappingServer, nestingFlag
    );
  }
}

}
