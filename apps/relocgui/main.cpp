/**
 * relocgui: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include <boost/program_options.hpp>

#include <tvgutil/misc/SettingsContainer.h>
using namespace tvgutil;

#include "RelocaliserApplication.h"

//#################### NAMESPACE ALIASES ####################

namespace po = boost::program_options;

//#################### TYPES ####################

/**
 * \brief This struct holds user-specifiable arguments.
 */
struct CommandLineArguments
{
  std::string calibrationFilename;
  std::string experimentTag;
  std::string testFolder;
  std::string trainFolder;
};

//#################### FUNCTIONS ####################

/**
 * \brief Adds all options in a set of parsed options to a settings container object.
 *
 * \param parsedOptions The set of parsed options.
 * \param settings      The settings container object.
 */
void add_options_to_settings(const po::parsed_options &parsedOptions, const SettingsContainer_Ptr &settings)
{
  for(size_t i = 0, optionCount = parsedOptions.options.size(); i < optionCount; ++i)
  {
    const po::basic_option<char> &option = parsedOptions.options[i];

    // Add all the specified values for the option in the correct order.
    for(size_t j = 0, valueCount = option.value.size(); j < valueCount; ++j)
    {
      settings->add_value(option.string_key, option.value[j]);
    }
  }
}

/**
 * \brief Parses any command-line arguments passed in by the user and adds them to the application settings.
 *
 * \param argc      The command-line argument count.
 * \param argv      The raw command-line arguments.
 * \param args      The parsed command-line arguments.
 * \param settings  The application settings.
 * \return          true, if the program should continue after parsing the command-line arguments, or false otherwise.
 */
bool parse_command_line(int argc, char *argv[], CommandLineArguments &args, const SettingsContainer_Ptr &settings)
{
  // Specify the possible options.
  po::options_description genericOptions("Generic options");
  genericOptions.add_options()
      ("help", "produce help message")
      ("configFile,f", po::value<std::string>(), "additional parameters filename")
      ("experimentTag", po::value<std::string>(&args.experimentTag)->default_value(""), "experiment tag")
      ;

  po::options_description diskSequenceOptions("Disk sequence options");
  diskSequenceOptions.add_options()
      ("calib,c", po::value<std::string>(&args.calibrationFilename)->required(), "calibration filename")
      ("test", po::value<std::string>(&args.testFolder)->required(), "path to the folder containing the training sequence")
      ("train", po::value<std::string>(&args.trainFolder)->required(), "path to the folder containing the training sequence")
      ;

  po::options_description options;
  options.add(genericOptions);
  options.add(diskSequenceOptions);

  // Parse the command line.
  po::parsed_options parsedCommandLineOptions = po::parse_command_line(argc, argv, options);

  // Add all options to the settings.
  add_options_to_settings(parsedCommandLineOptions, settings);

  // Also store them in the variable map.
  po::variables_map vm;
  po::store(parsedCommandLineOptions, vm);

  // If a configuration file was specified:
  if(vm.count("configFile"))
  {
    // Parse additional options from the configuration file and add any registered options to the variables map.
    // These will be post-processed (if necessary) and added to the settings later. Unregistered options are
    // also allowed: we add these directly to the settings without post-processing.
    po::parsed_options parsedConfigFileOptions =
        po::parse_config_file<char>(vm["configFile"].as<std::string>().c_str(), options, true);

    // Store registered options in the variables map.
    po::store(parsedConfigFileOptions, vm);

    // Add all remaining options to the settings.
    add_options_to_settings(parsedConfigFileOptions, settings);
  }

  po::notify(vm);

  std::cout << "Global settings:\n" << *settings << '\n';

  // If the user specifies the --help flag, print a help message.
  if(vm.count("help"))
  {
    std::cout << options << '\n';
    return false;
  }

  return true;
}

int main(int argc, char *argv[]) try
{
  // Construct the settings object for the application.
  SettingsContainer_Ptr settings(new SettingsContainer);

  // Parse the command-line arguments.
  CommandLineArguments args;
  if(!parse_command_line(argc, argv, args, settings))
  {
    return 0;
  }

  relocgui::RelocaliserApplication app(args.calibrationFilename, args.trainFolder, args.testFolder, settings);
  app.run();

  return EXIT_SUCCESS;
}
catch(std::exception &e)
{
  std::cerr << e.what() << '\n';
  return EXIT_FAILURE;
}
