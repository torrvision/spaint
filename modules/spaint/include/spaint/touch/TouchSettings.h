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

  //#################### PUBLIC VARIABLES ####################
public:
  /** The full path to the file containing the random forest used to filter touch regions. */
  boost::filesystem::path fullForestPath;

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

  /** A flag indicating whether or not to save images of the candidate connected components. */
  bool saveCandidateComponents;

  /** The path to use when saving the candidate components. */
  std::string saveCandidateComponentsPath;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Attempts to load touch settings from the specified XML file.
   *
   * This will throw if the settings cannot be successfully loaded.
   *
   * \param touchSettingsFile The path to the XML file containing the touch settings.
   */
  explicit TouchSettings(const boost::filesystem::path& touchSettingsFile);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Loads a random forest from the file specified by the forest path.
   *
   * The loading is done in TouchSettings rather than TouchDetector to work around a weird compiler bug.
   *
   * \return  The random forest that has been loaded.
   */
  RF_Ptr load_forest() const;
};

}

#endif
