/**
 * combineglobalposes: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include <fstream>
#include <iostream>

#include <boost/lexical_cast.hpp>

#include <itmx/geometry/GeometryUtil.h>
using namespace itmx;

#include <spaint/collaboration/CollaborativePoseOptimiser.h>
using namespace spaint;

#include <tvgutil/filesystem/PathFinder.h>
using namespace tvgutil;

namespace bf = boost::filesystem;

int main(int argc, char *argv[])
{
  // This application can be used to combine a set of global pose files produced by our collaborative mapping approach.
  std::vector<std::string> args(argv, argv + argc);
  if(args.size() < 4)
  {
    std::cout << "Usage: combineglobalposes <primary scene ID> <specifier 1> <specifier 2> [<specifier 3> ...]\n";
    return EXIT_FAILURE;
  }

  // Start a collaborative pose optimiser that aims to find the poses of all scenes relative to the specified primary scene.
  const std::string primarySceneID = args[1];
  CollaborativePoseOptimiser_Ptr poseOptimiser(new CollaborativePoseOptimiser(primarySceneID));
  poseOptimiser->start("Output");

  // For each global pose file specified:
  for(int i = 2; i < argc; ++i)
  {
    // Determine the path to the file. If the file doesn't exist, print a warning and skip it.
    bf::path p = find_subdir_from_executable("..") / "spaintgui" / "global_poses" / (args[i] + ".txt");
    if(!bf::exists(p))
    {
      std::cout << "Warning: " << p.string() << " not found\n";
      continue;
    }

    // Otherwise, read in all of the poses from the file, which are specified relative to the file's primary scene.
    std::cout << "Processing " << p << "\n\n";

    std::string filePrimarySceneID;
    std::vector<std::pair<std::string,DualQuatd> > fileRelativePoses;

    std::ifstream fs(p.string().c_str());
    std::string id;
    DualQuatd dq;
    while(fs >> id >> dq)
    {
      std::cout << "Read " << id << ' ' << dq << '\n';
      fileRelativePoses.push_back(std::make_pair(id, dq));

      // If this scene's pose relative to the file's primary scene is the identity, this is almost certainly the file's primary scene, so record that fact.
      if(DualQuatd::close(dq, DualQuatd::identity()))
      {
        std::cout << "Found file's primary scene: " << id << '\n';
        filePrimarySceneID = id;
      }
    }

    std::cout << '\n';

    // If we were unable to find the file's primary scene, print a warning and continue to the next global pose file.
    if(filePrimarySceneID == "")
    {
      std::cout << "Warning: Could not find file's primary scene, skipping poses in file\n";
      continue;
    }

    // Otherwise, add to the pose optimiser a relative transform from the coordinate system of the file's primary scene to that of each non-primary scene in the file.
    // FIXME: Currently, we are forced to add each relative transform multiple times, since the pose optimiser only works with confident edges.
    for(size_t i = 0, size = fileRelativePoses.size(); i < size; ++i)
    {
      if(fileRelativePoses[i].first == filePrimarySceneID) continue;

      for(int j = 0; j < CollaborativePoseOptimiser::confidence_threshold(); ++j)
      {
        poseOptimiser->add_relative_transform_sample(
          fileRelativePoses[i].first, filePrimarySceneID, GeometryUtil::dual_quat_to_pose(fileRelativePoses[i].second), CollaborationMode::CM_BATCH
        );
      }
    }
  }

  // Make sure the pose optimiser finishes running before we exit. The computed poses will be saved to disk automatically when the pose optimiser's destructor runs.
  boost::this_thread::sleep_for(boost::chrono::seconds(2));

  return 0;
}
