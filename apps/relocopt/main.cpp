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
#include <boost/chrono.hpp>
#include <boost/program_options.hpp>
#include <boost/timer/timer.hpp>
using boost::assign::list_of;

#include <evaluation/util/CoordinateDescentParameterOptimiser.h>
using namespace evaluation;

#include <tvgutil/filesystem/PathFinder.h>
using namespace tvgutil;

//#define COST_IS_TIME

//#################### NAMESPACE ALIASES ####################

namespace bc = boost::chrono;
namespace bf = boost::filesystem;
namespace bt = boost::timer;
namespace po = boost::program_options;

//#################### TYPES ####################

struct Arguments
{
  std::string datasetDir;
  bf::path dir;
  std::string iniSpecifier;
  bf::path logPath;
  std::string logSpecifier;
  std::string outputSpecifier;
  bf::path scriptPath;
  std::string scriptSpecifier;

  Arguments()
  : dir(find_subdir_from_executable("resources")),
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

  // Run the specified script.
  const bf::path outputPath = args.dir / (args.outputSpecifier + ".txt");
  const std::string command = "\"" + args.scriptPath.string() + "\" \"" + iniPath.string() + "\" \"" + outputPath.string() + "\" \"" + bf::path(args.datasetDir).string() + "\"";

  // Wrap the system call with a timer.
  bt::cpu_timer timer;
  int exitCode = system(command.c_str());
  timer.stop();

  float elapsedSeconds = bc::duration_cast<bc::seconds>(bc::nanoseconds(timer.elapsed().system + timer.elapsed().user)).count();

  if(exitCode)
  {
    throw std::runtime_error("System call failed. Terminating evaluation.");
  }

  // Read the results back in from the output file.
  float cost = std::numeric_limits<float>::max();
  float relocWeightedAverage, icpWeightedAverage, trainingMicroseconds, relocalisationMicroseconds, updateMicroseconds;

  {
    std::ifstream fs(outputPath.string().c_str());
    fs >> relocWeightedAverage >> icpWeightedAverage >> trainingMicroseconds >> relocalisationMicroseconds >> updateMicroseconds;

    // If the reads were successful, compute the cost.
    if(fs)
    {
      // The maximum times depend on the GPU (the following values assume a Titan X is used).
      static const float maxTrainingTime = 10000; // 10ms
      static const float maxRelocalisationTime = 200000; // 200ms
      static const float maxUpdateTime = 10000; // 10ms (we don't really care too much about it).

//      static const bool useRelocAverage = true; // We want to evaluate the relocalisation results BEFORE ICP.
      static const bool useRelocAverage = false; // We want to evaluate the relocalisation results AFTER ICP.

      // We need to negate the values because the optimiser tries to *minimise* the cost.
      cost = useRelocAverage ? -relocWeightedAverage : -icpWeightedAverage;

      // If we ran past the computation budget, penalize the cost.
      // Normally the cost would range [-100,0], this change makes the cost for a "slow" variant of the algorithm range [0,100].
      if(trainingMicroseconds > maxTrainingTime || relocalisationMicroseconds > maxRelocalisationTime || updateMicroseconds > maxUpdateTime)
      {
        cost += 100.0f;
      }
    }
  }

  std::ofstream logStream(args.logPath.c_str(), std::ios::app);
  logStream << cost << ';'
            << elapsedSeconds << ';'
            << relocWeightedAverage << ';'
            << icpWeightedAverage << ';'
            << trainingMicroseconds << ';'
            << relocalisationMicroseconds << ';'
            << updateMicroseconds << ';'
            << ParamSetUtil::param_set_to_string(params) << '\n';

  // Delete the .ini file and the output file again.
  bf::remove(iniPath);
  bf::remove(outputPath);

#ifdef COST_IS_TIME
  return elapsedSeconds;
#else
  return cost;
#endif
}

bool parse_command_line(int argc, char *argv[], Arguments& args)
{
  // Specify the possible options.
  po::options_description options;
  options.add_options()
    ("help", "produce help message")
    ("datasetDir,d", po::value<std::string>(&args.datasetDir)->default_value(""), "the dataset directory")
    ("logSpecifier,l", po::value<std::string>(&args.logSpecifier)->default_value("relocopt.log"), "the log specifier")
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

  // Prepare the log path.
  args.logPath = args.dir / args.logSpecifier;

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

  // Set up the log file.
  {
    std::ofstream(args.logPath.c_str(), std::ios::trunc);
  }

  // Set up the optimiser.
  const size_t epochCount = 5;
  const unsigned seed = 12345;
  CoordinateDescentParameterOptimiser optimiser(boost::bind(grove_cost_fn, args, _1), epochCount, seed);

  // Preemptive Ransac parameters.
  optimiser.add_param("PreemptiveRansac.maxCandidateGenerationIterations", list_of<int>(50)(250)(500)(1000)(6000));
  optimiser.add_param("PreemptiveRansac.maxPoseCandidates", list_of<int>(256)(512)(768)(1024)(2048));
  optimiser.add_param("PreemptiveRansac.maxPoseCandidatesAfterCull", list_of<int>(32)(64)(128)(256));
  optimiser.add_param("PreemptiveRansac.maxTranslationErrorForCorrectPose", list_of<float>(0.05f)(0.1f)(1000.0f)); // Last value basically disables the check.
  optimiser.add_param("PreemptiveRansac.minSquaredDistanceBetweenSampledModes", list_of<float>(0.0f)(0.15f * 0.15f)(0.3f * 0.3f)(0.6f * 0.6f)); // First value disables the check.
  optimiser.add_param("PreemptiveRansac.poseUpdate", list_of<bool>(false)(true));
  optimiser.add_param("PreemptiveRansac.ransacInliersPerIteration", list_of<int>(256)(512)(1024));
  optimiser.add_param("PreemptiveRansac.usePredictionCovarianceForPoseOptimization", list_of<bool>(false)(true));

  // Relocaliser parameters.
  optimiser.add_param("ScoreRelocaliser.clustererSigma", list_of<float>(0.05f)(0.1f)(0.2f));
  optimiser.add_param("ScoreRelocaliser.clustererTau", list_of<float>(0.05f)(0.1f)(0.2f));
  optimiser.add_param("ScoreRelocaliser.maxClusterCount", list_of<int>(5)(10)(25)(50));
  optimiser.add_param("ScoreRelocaliser.minClusterSize", list_of<int>(5)(20)(50)(100));
  optimiser.add_param("ScoreRelocaliser.reservoirCapacity", list_of<int>(512)(1024)(2048));

  // Print header in the log file.
  {
    std::ofstream logStream(args.logPath.c_str());
    logStream << "Cost;TotalTime;RelocAvg;ICPAvg;TrainingTime;RelocalisationTime;UpdateTime;Params\n";
  }

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
