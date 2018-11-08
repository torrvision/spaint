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

#include <orx/geometry/GeometryUtil.h>
using namespace orx;

#include <tvgutil/filesystem/PathFinder.h>
#include <tvgutil/persistence/PropertyUtil.h>
using namespace tvgutil;

#include "trackers/GlobalTracker.h"
#include "trackers/RemoteTracker.h"

#ifdef WITH_OVR
#include "trackers/RiftTracker.h"
#endif

#ifdef WITH_VICON
#include "trackers/ViconTracker.h"
#endif

#ifdef WITH_ZED
#include "trackers/ZedTracker.h"
#endif

namespace itmx {

//#################### PUBLIC MEMBER FUNCTIONS ####################

Tracker_Ptr TrackerFactory::make_tracker_from_file(const std::string& trackerConfigFilename, bool trackSurfels,
                                                   const Vector2i& rgbImageSize, const Vector2i& depthImageSize,
                                                   const LowLevelEngine_CPtr& lowLevelEngine, const IMUCalibrator_Ptr& imuCalibrator,
                                                   const Settings_CPtr& settings, FallibleTracker*& fallibleTracker,
                                                   const MappingServer_Ptr& mappingServer, NestingFlag nestingFlag) const
{
  Tree tree = PropertyUtil::load_properties_from_xml_file(trackerConfigFilename);
  Tree trackerTree = tree.get_child("tracker");
  return make_tracker(trackerTree, trackSurfels, rgbImageSize, depthImageSize, lowLevelEngine, imuCalibrator, settings, fallibleTracker, mappingServer, nestingFlag);
}

Tracker_Ptr TrackerFactory::make_tracker_from_string(const std::string& trackerConfig, bool trackSurfels,
                                                     const Vector2i& rgbImageSize, const Vector2i& depthImageSize,
                                                     const LowLevelEngine_CPtr& lowLevelEngine, const IMUCalibrator_Ptr& imuCalibrator,
                                                     const Settings_CPtr& settings, FallibleTracker*& fallibleTracker,
                                                     const MappingServer_Ptr& mappingServer, NestingFlag nestingFlag) const
{
  Tree tree = PropertyUtil::load_properties_from_xml_string(trackerConfig);
  Tree trackerTree = tree.get_child("tracker");
  return make_tracker(trackerTree, trackSurfels, rgbImageSize, depthImageSize, lowLevelEngine, imuCalibrator, settings, fallibleTracker, mappingServer, nestingFlag);
}

#ifdef WITH_VICON
void TrackerFactory::set_vicon(const ViconInterface_CPtr& vicon)
{
  m_vicon = vicon;
}
#endif

//#################### PRIVATE MEMBER FUNCTIONS ####################

Tracker_Ptr TrackerFactory::make_simple_tracker(std::string trackerType, std::string trackerParams, bool trackSurfels,
                                                const Vector2i& rgbImageSize, const Vector2i& depthImageSize,
                                                const LowLevelEngine_CPtr& lowLevelEngine, const IMUCalibrator_Ptr& imuCalibrator,
                                                const Settings_CPtr& settings, FallibleTracker*& fallibleTracker,
                                                const MappingServer_Ptr& mappingServer, NestingFlag nestingFlag) const
{
  ITMTracker *tracker = NULL;

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
  else if(trackerType == "remote")
  {
    tracker = new RemoteTracker(mappingServer, boost::lexical_cast<int>(trackerParams));
  }
  else if(trackerType == "rift")
  {
#ifdef WITH_OVR
    if(ovrHmd_Detect() != 0) tracker = new RiftTracker;
    else throw std::runtime_error("Error: The Rift is not currently available. Did you install the driver and plug it in?");
#else
    throw std::runtime_error("Error: Rift support not currently available. Reconfigure in CMake with the WITH_OVR option set to on.");
#endif
  }
  else if(trackerType == "vicon")
  {
#ifdef WITH_VICON
    if(m_vicon)
    {
      ViconTracker::TrackingMode trackingMode = trackerParams == "absolute" ? ViconTracker::TM_ABSOLUTE : ViconTracker::TM_RELATIVE;
      fallibleTracker = new ViconTracker(m_vicon, "kinect", trackingMode);
      tracker = fallibleTracker;
    }
    else throw std::runtime_error("Error: The Vicon interface is null (try adding --useVicon to the command line).");
#else
    throw std::runtime_error("Error: Vicon support not currently available. Reconfigure in CMake with the WITH_VICON option set to on.");
#endif
  }
  else if(trackerType == "zed")
  {
#ifdef WITH_ZED
    tracker = new ZedTracker(ZedCamera::instance());
#else
    throw std::runtime_error("Error: Zed support not currently available. Reconfigure in CMake with the WITH_ZED option set to on.");
#endif
  }

  // Finally, return the constructed tracker, making sure to prevent double deletion if it will be added to a composite.
  return nestingFlag == NESTED ? Tracker_Ptr(tracker, boost::serialization::null_deleter()) : Tracker_Ptr(tracker);
}

Tracker_Ptr TrackerFactory::make_tracker(const Tree& trackerTree, bool trackSurfels,
                                         const Vector2i& rgbImageSize, const Vector2i& depthImageSize,
                                         const LowLevelEngine_CPtr& lowLevelEngine, const IMUCalibrator_Ptr& imuCalibrator,
                                         const Settings_CPtr& settings, FallibleTracker*& fallibleTracker,
                                         const MappingServer_Ptr& mappingServer, NestingFlag nestingFlag) const
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
  else if(trackerType == "global")
  {
    // Construct the nested tracker.
    Tree nestedTrackerTree = trackerTree.get_child("tracker");
    Tracker_Ptr nestedTracker = make_tracker(
      nestedTrackerTree, trackSurfels, rgbImageSize, depthImageSize,
      lowLevelEngine, imuCalibrator, settings, fallibleTracker,
      mappingServer, NESTED
    );

    // Construct the global tracker.
    ORUtils::SE3Pose initialPose = GeometryUtil::dual_quat_to_pose(boost::lexical_cast<DualQuatd>(trackerParams));
    GlobalTracker *globalTracker = new GlobalTracker(nestedTracker, initialPose);

    // Return the global tracker, making sure to prevent double deletion if it will be added to a composite.
    return nestingFlag == NESTED ? Tracker_Ptr(globalTracker, boost::serialization::null_deleter()) : Tracker_Ptr(globalTracker);
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
