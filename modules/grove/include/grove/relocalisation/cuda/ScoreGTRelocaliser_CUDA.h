/**
 * grove: ScoreGTRelocaliser_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_GROVE_SCOREGTRELOCALISER_CUDA
#define H_GROVE_SCOREGTRELOCALISER_CUDA

#include "../interface/ScoreGTRelocaliser.h"

namespace grove {

/**
 * \brief An instance of this class can be used to relocalise a camera in a 3D scene on the GPU, using known ground truth correspondences.
 */
class ScoreGTRelocaliser_CUDA : public ScoreGTRelocaliser
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a ground truth SCoRe relocaliser on the GPU.
   *
   * \param settings          The settings used to configure the relocaliser.
   * \param settingsNamespace The namespace associated with the settings that are specific to the relocaliser.
   */
  ScoreGTRelocaliser_CUDA(const tvgutil::SettingsContainer_CPtr& settings, const std::string& settingsNamespace);

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /** Override */
  virtual void set_ground_truth_predictions_for_keypoints(const Keypoint3DColourImage_CPtr& keypointsImage, const Matrix4f& cameraToWorld,
                                                          ScorePredictionsImage_Ptr& outputPredictions) const;
};

}

#endif
