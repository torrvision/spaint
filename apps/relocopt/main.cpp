/**
 * relocopt: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include <boost/program_options.hpp>
using boost::assign::list_of;

#include <evaluation/util/CoordinateDescentParameterOptimiser.h>
using namespace evaluation;

#include <tvgutil/filesystem/PathFinder.h>
using namespace tvgutil;

//#################### NAMESPACE ALIASES ####################

namespace bf = boost::filesystem;
namespace po = boost::program_options;

//#################### TYPES ####################

struct Arguments
{
  std::string datasetDir;
  bf::path dir;
  std::string iniSpecifier;
  std::string outputSpecifier;
  bf::path scriptPath;
  std::string scriptSpecifier;

  Arguments()
  : dir(find_executable().parent_path()),
    iniSpecifier("temp"),
    outputSpecifier("temp")
  {}
};

//#################### FUNCTIONS ####################

float grove_cost_fn(const Arguments& args, const ParamSet& params)
{
  // Write the parameters to the specified .ini file.
  const bf::path iniPath = args.dir / (args.iniSpecifier + ".ini");

  {
    std::ofstream fs(iniPath.string().c_str());
    for(ParamSet::const_iterator it = params.begin(), iend = params.end(); it != iend; ++it)
    {
      fs << it->first << "=" << it->second << '\n';
    }
  }

  // Attempt to run the specified script. If it doesn't exist, throw.
  const bf::path outputPath = args.dir / (args.outputSpecifier + ".txt");
  const std::string command = "\"\"" + args.scriptPath.string() + "\" \"" + iniPath.string() + "\" \"" + outputPath.string() + "\" \"" + bf::path(args.datasetDir).string() + "\"\"";
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

bool parse_command_line(int argc, char *argv[], Arguments& args)
{
  // Specify the possible options.
  po::options_description options;
  options.add_options()
    ("help", "produce help message")
    ("datasetDir,d", po::value<std::string>(&args.datasetDir)->default_value(""), "the dataset directory")
    ("scriptSpecifier,s", po::value<std::string>(&args.scriptSpecifier)->default_value(""), "the script specifier")
  ;

  // Actually parse the command line.
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, options), vm);
  po::notify(vm);

  // If the user specifies the --help flag, print a help message.
  if(vm.count("help"))
  {
    std::cout << options << '\n';
    return false;
  }

  // Attempt to find the specified script file.
#if _MSC_VER
  args.scriptPath = args.dir / (args.scriptSpecifier + ".bat");
#else
  args.scriptPath = args.dir / (args.scriptSpecifier + ".sh");
#endif

  if(!bf::exists(args.scriptPath))
  {
    throw std::runtime_error("The script file was not specified or does not exist");
  }

  // Attempt to find the dataset directory.
  if(!bf::exists(bf::path(args.datasetDir)))
  {
    throw std::runtime_error("The dataset directory was not specified or does not exist");
  }

  return true;
}

int main(int argc, char *argv[])
try
{
  // Parse the command-line arguments.
  Arguments args;
  if(!parse_command_line(argc, argv, args))
  {
    return EXIT_FAILURE;
  }

  // Set up the optimiser.
  const size_t epochCount = 10;
  const unsigned seed = 12345;
  CoordinateDescentParameterOptimiser optimiser(boost::bind(grove_cost_fn, args, _1), epochCount, seed);
  optimiser.add_param("modeCount", list_of<int>(5)(10)(20)(30)(40)(50))
           .add_param("verifiedCandidateCount", list_of<int>(1)(2)(4)(8));

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
catch(std::exception& e)
{
  std::cerr << e.what() << '\n';
  return EXIT_FAILURE;
}
