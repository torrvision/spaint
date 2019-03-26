/**
 * grove: ScoreNetRelocaliser_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_GROVE_SCORENETRELOCALISER_CUDA
#define H_GROVE_SCORENETRELOCALISER_CUDA

#include "../interface/ScoreNetRelocaliser.h"

namespace grove {

/**
 * \brief An instance of this class can be used to relocalise a camera in a 3D scene on the GPU, by adapting the predictions of a
 *        pre-trained scene coordinate regression network to the scene of interest.
 */
class ScoreNetRelocaliser_CUDA : public ScoreNetRelocaliser
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a network-based SCoRe relocaliser on the GPU.
   *
   * \param settings          The settings used to configure the relocaliser.
   * \param settingsNamespace The namespace associated with the settings that are specific to the relocaliser.
   *
   * \throws std::runtime_error If the relocaliser cannot be constructed.
   */
  ScoreNetRelocaliser_CUDA(const tvgutil::SettingsContainer_CPtr& settings, const std::string& settingsNamespace);

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /** Override */
  virtual void set_bucket_predictions_for_keypoints(const BucketIndicesImage_CPtr& bucketIndices, ScorePredictionsImage_Ptr& outputPredictions) const;

  /** Override */
  virtual void set_net_predictions_for_keypoints(const Keypoint3DColourImage_CPtr& keypointsImage, const ScoreNetOutput_CPtr& scoreNetOutput,
                                                 ScorePredictionsImage_Ptr& outputPredictions) const;
};

}

#endif
