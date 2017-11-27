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
  std::vector<std::string> args(argv, argv + argc);
  if(args.size() < 4)
  {
    std::cout << "Usage: combineglobalposes <primary scene ID> <specifier 1> <specifier 2> [<specifier 3> ...]\n";
    return EXIT_FAILURE;
  }

  const std::string primarySceneID = args[1];
  CollaborativePoseOptimiser_Ptr poseOptimiser(new CollaborativePoseOptimiser(primarySceneID));
  poseOptimiser->start("Output");

  for(int i = 2; i < argc; ++i)
  {
    // TODO: Comment here.
    bf::path p = find_subdir_from_executable("..") / "spaintgui" / "global_poses" / (args[i] + ".txt");
    if(!bf::exists(p))
    {
      std::cout << "Warning: " << p.string() << " not found\n";
      continue;
    }

    std::cout << "Processing " << p << "\n\n";

    std::vector<std::pair<std::string,DualQuatd> > fileRelativePoses;
    std::string filePrimary;

    std::ifstream fs(p.string().c_str());
    std::string id;
    DualQuatd dq;
    while(fs >> id >> dq)
    {
      std::cout << "Read " << id << ' ' << dq << '\n';

      fileRelativePoses.push_back(std::make_pair(id, dq));
      if(DualQuatd::close(dq, DualQuatd::identity()))
      {
        std::cout << "Found file primary: " << id << '\n';
        filePrimary = id;
      }
    }

    std::cout << '\n';

    // TODO: Comment here.
    if(filePrimary == "")
    {
      std::cout << "Warning: Could not find file primary, skipping poses in file\n";
      continue;
    }

    // TODO: Comment here.
    for(size_t i = 0, size = fileRelativePoses.size(); i < size; ++i)
    {
      if(fileRelativePoses[i].first != filePrimary)
      {
        for(int j = 0; j < CollaborativePoseOptimiser::confidence_threshold(); ++j)
        {
          poseOptimiser->add_relative_transform_sample(fileRelativePoses[i].first, filePrimary, GeometryUtil::dual_quat_to_pose(fileRelativePoses[i].second), CollaborationMode::CM_BATCH);
        }
      }
    }
  }

  boost::this_thread::sleep_for(boost::chrono::seconds(2));

  return 0;
}
