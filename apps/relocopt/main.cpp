/**
 * relocopt: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include <fstream>
#include <limits>

#include <boost/bind.hpp>

#include <evaluation/util/CoordinateDescentParameterOptimiser.h>
using namespace evaluation;

#include <tvgutil/filesystem/PathFinder.h>
using namespace tvgutil;

namespace bf = boost::filesystem;

//#################### HELPER FUNCTIONS ####################

float grove_cost_fn(const std::string& scriptSpecifier, const std::string& iniSpecifier, const std::string& outputSpecifier, const ParamSet& params)
{
  // Write the parameters to the specified .ini file.
  const bf::path dir = find_executable().parent_path();
  const bf::path iniPath = dir / (iniSpecifier + ".ini");

  {
    std::ofstream fs(iniPath.string().c_str());
    for(ParamSet::const_iterator it = params.begin(), iend = params.end(); it != iend; ++it)
    {
      fs << it->first << "=" << it->second << '\n';
    }
  }

  // Run the specified script.
  const bf::path outputPath = dir / (scriptSpecifier + ".txt");
  const bf::path scriptPath = dir / (scriptSpecifier + ".sh");
  const std::string command = scriptPath.string() + " " + iniPath.string() + " " + outputPath.string();
  system(command.c_str());

  // Read the cost back in from the output file.
  float cost = std::numeric_limits<float>::max();

  {
    std::ifstream fs(outputPath.string().c_str());
    fs >> cost;
  }

  // Delete the .ini file and the output file again.
  bf::remove(iniPath);
  bf::remove(outputPath);

  return cost;
}

//#################### FUNCTIONS ####################

int main()
{
  // Set up the optimiser.
  const std::string scriptSpecifier = "evalgrove";
  const std::string iniSpecifier = "temp";
  const std::string outputSpecifier = "temp";
  const size_t epochCount = 10;
  const unsigned seed = 12345;
  CoordinateDescentParameterOptimiser optimiser(boost::bind(grove_cost_fn, scriptSpecifier, iniSpecifier, outputSpecifier, _1), epochCount, seed);
  // TODO

  // Use the optimiser to choose a set of parameters.
  float cost;
  ParamSet params = optimiser.optimise_for_parameters(&cost);

  // Output the chosen parameters.
  for(std::map<std::string,std::string>::const_iterator it = params.begin(), iend = params.end(); it != iend; ++it)
  {
    std::cout << it->first << ": " << it->second << '\n';
  }

  return 0;
}
