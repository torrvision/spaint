/**
 * combineglobalposes: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include <fstream>
#include <iostream>

#include <boost/lexical_cast.hpp>

#include <itmx/geometry/GeometryUtil.h>
using namespace itmx;

#include <tvgutil/filesystem/PathFinder.h>
using namespace tvgutil;

namespace bf = boost::filesystem;

int main(int argc, char *argv[])
{
  std::vector<std::string> args(argv, argv + argc);
  if(args.size() < 4)
  {
    std::cout << "Usage: combineglobalposes <primary scene> <specifier 1> <specifier 2> [<specifier 3> ...]\n";
    return EXIT_FAILURE;
  }

  for(int i = 2; i < argc; ++i)
  {
    bf::path p = find_subdir_from_executable("..") / "spaintgui" / "global_poses" / (args[i] + ".txt");
    if(!bf::exists(p))
    {
      std::cout << "Warning: " << p.string() << " not found\n";
      continue;
    }

    std::cout << "Processing " << p << "\n\n";

    std::vector<std::pair<std::string,DualQuatd> > localRelativePoses;
    std::string localPrimary;

    std::ifstream fs(p.string().c_str());
    std::string id;
    DualQuatd dq;
    while(fs >> id >> dq)
    {
      std::cout << "Read " << id << ' ' << dq << '\n';

      localRelativePoses.push_back(std::make_pair(id, dq));
      if(DualQuatd::close(dq, DualQuatd::identity()))
      {
        std::cout << "Found Local Primary: " << id << '\n';
        localPrimary = id;
      }
    }

    std::cout << '\n';
  }

  return 0;
}
