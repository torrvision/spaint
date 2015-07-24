/**
 * spaint: TouchSettings.h
 */

#ifndef H_SPAINT_TOUCHSETTINGS
#define H_SPAINT_TOUCHSETTINGS

#include <boost/filesystem.hpp>

#include <rafl/core/RandomForest.h>

namespace spaint {

/**
 * \brief An instance of this class can be used to provide the settings needed to configure a touch detector.
 */
class TouchSettings
{
  //#################### TYPEDEFS ####################
private:
  typedef int Label;
  typedef rafl::RandomForest<Label> RF;
  typedef boost::shared_ptr<RF> RF_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The path to the XML file containing the touch settings. */
  boost::filesystem::path m_touchSettingsFile;

  //#################### PUBLIC VARIABLES ####################
public:
  /** The path to the file containing the random forest used to filter touch regions. */
  std::string forestPath;

  /** The threshold (in mm) below which the raw and raycasted depths are assumed to be equal. */
  int lowerDepthThresholdMm;

  /** The maximum fraction of the image that a connected change component can have if it is to be considered as a candidate touch interaction. */
  float maxCandidateFraction;

  /** The minimum fraction of the image that a connected change component can have if it is to be considered as a candidate touch interaction. */
  float minCandidateFraction;

  /** The minimum fraction of the image that the part of the candidate touch interaction which is touching the scene can have if it is to be considered as a valid touch. */
  float minTouchAreaFraction;

  /** The side length of the morphological opening kernel that is applied to the change mask to reduce noise. */
  int morphKernelSize;

  /** A flag indicating whether to save images of the candidate connected components. */
  bool saveCandidateComponents;

  /** The path to use when saving the candidate components. */
  std::string saveCandidateComponentsPath;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Attempts to load settings from the specified XML file.
   *
   * This will throw if the settings cannot be successfully loaded.
   *
   * \param touchSettingsFile The path to the XML file containing the touch settings.
   */
  explicit TouchSettings(const boost::filesystem::path& touchSettingsFile);

  //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
  /** Deliberately private and unimplemented. */
  TouchSettings(const TouchSettings&);
  TouchSettings& operator=(const TouchSettings&);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  RF_Ptr load_forest() const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  boost::filesystem::path determine_full_forest_path() const;

  /**
   * \brief Load settings from a property map.
   *
   * \param properties  The property map.
   */
  void initialise(const std::map<std::string,std::string>& properties);
};

}

#endif
