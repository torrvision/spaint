/**
 * relocgui: RelocaliserApplication.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_RELOCGUI_RELOCALISERAPPLICATION
#define H_RELOCGUI_RELOCALISERAPPLICATION

#include <string>

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

#include <opencv2/opencv.hpp>

#include <ITMLib/Objects/Camera/ITMRGBDCalib.h>

#include <ORUtils/SE3Pose.h>

#include <itmx/base/ITMObjectPtrTypes.h>
#include <itmx/relocalisation/Relocaliser.h>

#include <orx/base/ORImagePtrTypes.h>

#include <tvgutil/misc/SettingsContainer.h>

#include <tvgutil/filesystem/SequentialPathGenerator.h>

namespace relocgui {

/**
 * \brief An instance of this class allows the evaluation of the grove relocaliser on a 7-scenes-like RGB-D sequence.
 *
 * The application expects to be provided with the path to a camera calibration file, and the paths to the training and
 * testing splits of the RGB-D sequence. Each split has the following layout on disk:
 *
 * frame-000000.color.png // Colour image.
 * frame-000000.depth.png // Depth image (each pixel representing a depth encoded as an unsigned short value).
 * frame-000000.pose.txt  // 4x4 camera-to-world transformation matrix.
 * frame-000001.color.png // ...
 * .
 * .
 * .
 * frame-NNNNNN.pose.txt  // ...
 *
 * By default the application computes the percentage of testing frames relocalised correctly against the ground truth
 * pose (according to Shotton et al's 5cm/5deg metric). The application also saves the relocalised pose for each frame
 * of the testing sequence in the reloc_poses/experimentTag subfolder of the current executable using the following
 * filename pattern: pose-%06i.reloc.txt
 */
class RelocaliserApplication
{
  //#################### NESTED TYPES ####################
private:
  struct RelocalisationExample
  {
    /** Ground truth camera pose associated to the example (world-to-camera). */
    ORUtils::SE3Pose cameraPose;

    /** Depth image (single channel, uint16_t, can be converted to metric values according to the camera calibration
     * file). */
    cv::Mat depthImage;

    /** Colour image in uint8_t RGBA format. */
    cv::Mat rgbImage;
  };

  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<tvgutil::SequentialPathGenerator> SequentialPathGenerator_Ptr;

public:
  //#################### CONSTRUCTOR ####################
  /**
   * \brief Constructs an instance of the RelocaliserApplication.
   *
   * \param calibrationPath Path to a camera calibration file in the InfiniTAM format.
   * \param trainingPath    Path to a training split of a 7-scenes-like sequence.
   * \param testingPath     Path to a testing split of a 7-scenes-like sequence.
   * \param settings        A container of settings for the relocaliser.
   *
   * \throws std::invalid_argument if the paths are incorrect.
   */
  RelocaliserApplication(const std::string &calibrationPath,
                         const std::string &trainingPath,
                         const std::string &testingPath,
                         const tvgutil::SettingsContainer_CPtr &settings);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Run the evaluation, first training the relocaliser using the training split then relocalising each frame of
   *        the testing split.
   */
  void run();

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Tries to read a RGB-D example from the 7-scenes-like sequence contained in the folder specified by the
   * pathGenerator.
   *
   * \param pathGenerator The path generator used to obtain the current file names for RGB, depth and pose files.
   *
   * \return An example if found, boost::none if the index contained in the pathGenerator is greater than the number of
   * examples in the folder.
   */
  boost::optional<RelocalisationExample> read_example(const SequentialPathGenerator_Ptr &pathGenerator) const;

  /**
   * \brief Convert the images contained in the example to ORUtil's image format and copy them on the GPU.
   *
   * \param example The example to convert.
   */
  void prepare_example_images(const RelocalisationExample &example);

  /**
   * \brief Visualise the RGB-D frame for an example.
   *
   * \param example The example to visualise in a window.
   * \param uiText  Optional text to superimpose to the example.
   */
  void show_example(const RelocalisationExample &example, const std::string &uiText = "") const;

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** The path to a camera calibration file. */
  boost::filesystem::path m_calibrationFilePath;

  /** The camera calibration informations. */
  ITMLib::ITMRGBDCalib m_cameraCalibration;

  /** The (printf-like) mask used to generate the depth image filenames. */
  std::string m_depthImageMask;

  /** The path to a folder wherein to save the relocalised poses. */
  boost::filesystem::path m_outputPosesPath;

  /** A path generator used to generate the filenames for the relocalised poses. */
  SequentialPathGenerator_Ptr m_outputPosesPathGenerator;

  /** The (printf-like) mask used to generate the pose filenames. */
  std::string m_poseFileMask;

  /** The relocaliser being tested. */
  itmx::Relocaliser_Ptr m_relocaliser;

  /** The (printf-like) mask used to generate the RGB image filenames. */
  std::string m_rgbImageMask;

  /** Whether or not to save the relocalised poses on disk. */
  bool m_saveRelocalisedPoses;

  /** The container storing the relocaliser's settings (and miscellaneous data such as the experimentTag). */
  tvgutil::SettingsContainer_CPtr m_settingsContainer;

  /** A path generator used to generate testing example filenames. */
  SequentialPathGenerator_Ptr m_testingSequencePathGenerator;

  /** The path to a testing split of a 7-scenes-like sequence. */
  boost::filesystem::path m_testSequencePath;

  /** A path generator used to generate training example filenames. */
  SequentialPathGenerator_Ptr m_trainingSequencePathGenerator;

  /** The path to a training split of a 7-scenes-like sequence. */
  boost::filesystem::path m_trainSequencePath;

  /** A viewbuilder used to convert the depth from short ints to metric values. */
  ViewBuilder_Ptr m_viewBuilder;

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  //
  // The variables below are overwritten for each example. Kept as members to avoid reallocating them every time, since
  // it's expensive on the GPU.
  //
  /** The current depth image in metres. */
  ORFloatImage_Ptr m_currentDepthImage;

  /** The current depth image in short format. */
  ORShortImage_Ptr m_currentRawDepthImage;

  /** The current colour image. */
  ORUChar4Image_Ptr m_currentRgbImage;
};

}

#endif
