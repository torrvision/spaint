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

#include <itmx/base/ITMImagePtrTypes.h>
#include <itmx/base/ITMObjectPtrTypes.h>
#include <itmx/relocalisation/Relocaliser.h>

#include <ORUtils/SE3Pose.h>

#include <tvgutil/misc/SettingsContainer.h>

#include <tvgutil/filesystem/SequentialPathGenerator.h>

namespace relocgui {

class RelocaliserApplication
{
  //#################### NESTED TYPES ####################
private:
  struct RelocalisationExample
  {
    cv::Mat depthImage;
    cv::Mat rgbImage;
    ORUtils::SE3Pose cameraPose;
  };

  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<tvgutil::SequentialPathGenerator> SequentialPathGenerator_Ptr;

public:
  //#################### CONSTRUCTOR ####################
  RelocaliserApplication(const std::string &calibrationPath,
                         const std::string &trainingPath,
                         const std::string &testingPath,
                         const tvgutil::SettingsContainer_CPtr &settings);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  void set_pose_file_mask(const std::string &poseMask);
  void set_rgb_image_mask(const std::string &rgbMask);
  void set_depth_image_mask(const std::string &depthMask);

  void run();

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  boost::optional<RelocalisationExample> read_example(const SequentialPathGenerator_Ptr& pathGenerator) const;

  void prepare_example_images(const RelocalisationExample& example);

  void show_example(const RelocalisationExample& example, const std::string& uiText = "") const;

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  std::string m_poseFileMask;
  std::string m_rgbImageMask;
  std::string m_depthImageMask;

  boost::filesystem::path m_calibrationFilePath;
  boost::filesystem::path m_trainSequencePath;
  boost::filesystem::path m_testSequencePath;
  boost::filesystem::path m_outputPosesPath;

  SequentialPathGenerator_Ptr m_trainingSequencePathGenerator;
  SequentialPathGenerator_Ptr m_testingSequencePathGenerator;
  SequentialPathGenerator_Ptr m_outputPosesPathGenerator;

  bool m_saveRelocalisedPoses;

  ITMLib::ITMRGBDCalib m_cameraCalibration;

  tvgutil::SettingsContainer_CPtr m_settingsContainer;

  itmx::Relocaliser_Ptr m_relocaliser;

  ViewBuilder_Ptr m_viewBuilder;

  ITMShortImage_Ptr m_currentRawDepthImage;
  ITMFloatImage_Ptr m_currentDepthImage;
  ITMUChar4Image_Ptr m_currentRgbImage;
};
}

#endif
