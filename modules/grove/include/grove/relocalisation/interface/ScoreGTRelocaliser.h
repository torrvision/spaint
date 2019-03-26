/**
 * grove: ScoreGTRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_GROVE_SCOREGTRELOCALISER
#define H_GROVE_SCOREGTRELOCALISER

#include "ScoreRelocaliser.h"

namespace grove {

/**
 * \brief An instance of a class deriving from this one can be used to relocalise a camera in a 3D scene using known ground truth correspondences.
 */
class ScoreGTRelocaliser : public ScoreRelocaliser
{
  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a ground truth SCoRe relocaliser.
   *
   * \param settings          The settings used to configure the relocaliser.
   * \param settingsNamespace The namespace associated with the settings that are specific to the relocaliser.
   * \param deviceType        The device on which the relocaliser should operate.
   */
  ScoreGTRelocaliser(const tvgutil::SettingsContainer_CPtr& settings, const std::string& settingsNamespace, ORUtils::DeviceType deviceType);

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /** Override */
  virtual void make_predictions(const ORUChar4Image *colourImage) const;

  /** Override */
  virtual void train_sub(const ORUChar4Image *colourImage, const ORFloatImage *depthImage, const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose);
};

}

#endif
