/**
 * grove: ScoreForestRelocaliser_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCOREFORESTRELOCALISER_CUDA
#define H_GROVE_SCOREFORESTRELOCALISER_CUDA

#include "../interface/ScoreForestRelocaliser.h"

namespace grove {

/**
 * \brief An instance of this class can be used to relocalise a camera in a 3D scene on the GPU, using the approach described
 *        in "On-the-Fly Adaptation of Regression Forests for Online Camera Relocalisation" (Cavallari et al., 2017).
 */
class ScoreForestRelocaliser_CUDA : public ScoreForestRelocaliser
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a GPU-based SCoRe forest relocaliser.
   *
   * \param settings          The settings used to configure the relocaliser.
   * \param settingsNamespace The namespace associated with the settings that are specific to the relocaliser.
   *
   * \throws std::runtime_error If the relocaliser cannot be constructed.
   */
  ScoreForestRelocaliser_CUDA(const tvgutil::SettingsContainer_CPtr& settings, const std::string& settingsNamespace);

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /** Override */
  virtual void merge_predictions_for_keypoints(const LeafIndicesImage_CPtr& leafIndices, ScorePredictionsImage_Ptr& outputPredictions) const;
};

}

#endif
