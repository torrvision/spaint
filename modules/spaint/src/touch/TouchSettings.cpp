/**
 * spaint: TouchSettings.cpp
 */

#include "touch/TouchSettings.h"

#include <tvgutil/DirectoryUtil.h>
#include <tvgutil/MapUtil.h>
#include <tvgutil/PropertyUtil.h>
#include <tvgutil/SerializationUtil.h>
using namespace tvgutil;

namespace spaint {

//#################### CONSTRUCTORS ####################

TouchSettings::TouchSettings(const boost::filesystem::path& touchSettingsFile)
{
  // Load in the settings.
  boost::property_tree::ptree tree = PropertyUtil::load_properties_from_xml(touchSettingsFile.string());
  std::map<std::string,std::string> properties = PropertyUtil::make_property_map(tree);

  std::string forestPath;

  #define GET_SETTING(param) tvgutil::MapUtil::typed_lookup(properties, #param, param)
    GET_SETTING(forestPath);
    GET_SETTING(lowerDepthThresholdMm);
    GET_SETTING(minCandidateFraction);
    GET_SETTING(minTouchAreaFraction);
    GET_SETTING(maxCandidateFraction);
    GET_SETTING(morphKernelSize);
    GET_SETTING(saveCandidateComponents);
    GET_SETTING(saveCandidateComponentsPath);
  #undef GET_SETTING

  // Determine the full path to the file containing the random forest, and check that it exists.
  fullForestPath = touchSettingsFile.branch_path() / forestPath;
  if(!boost::filesystem::exists(fullForestPath))
  {
    throw std::runtime_error("Touch detection random forest not found: " + fullForestPath.string());
  }

  // If we're saving candidate components, check that the target path is valid and does not contain any existing files.
  if(saveCandidateComponents)
  {
    if(!boost::filesystem::is_directory(saveCandidateComponentsPath))
    {
      throw std::runtime_error("Save candidate components path not found: " + saveCandidateComponentsPath + ". Please add a valid path to the touch settings.");
    }

    static size_t fileCount = tvgutil::DirectoryUtil::get_file_count(saveCandidateComponentsPath);
    if(fileCount > 0)
    {
      throw std::runtime_error("Error: " + saveCandidateComponentsPath + " already contains " + boost::lexical_cast<std::string>(fileCount) + " files");
    }
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const std::string& TouchSettings::get_save_candidate_components_path() const
{
  return saveCandidateComponentsPath;
}

TouchSettings::RF_Ptr TouchSettings::load_forest() const
{
  // Register the relevant decision function generators with the factory.
  rafl::DecisionFunctionGeneratorFactory<Label>::instance().register_rafl_makers();

  // Load the forest.
  RF_Ptr forest = SerializationUtil::load_text(fullForestPath.string(), forest);

  return forest;
}

bool TouchSettings::should_save_candidate_components() const
{
  return saveCandidateComponents;
}

}
