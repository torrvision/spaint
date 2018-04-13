/**
 * grove: ScoreRelocaliser_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCORERELOCALISERCUDA
#define H_GROVE_SCORERELOCALISERCUDA

#include "../interface/ScoreRelocaliser.h"

namespace grove {

/**
 * \brief An instance of this class allows the relocalisation of camera pose used to acquire RGB-D
 *        image pairs, on the GPU, according to the method described in:
 *
 *        "On-the-Fly Adaptation of Regression Forests for Online Camera Relocalisation" by
 *        Tommaso Cavallari, Stuart Golodetz*, Nicholas A. Lord*,
 *        Julien Valentin, Luigi Di Stefano and Philip H. S. Torr
 */
class ScoreRelocaliser_CUDA : public ScoreRelocaliser
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an instance of ScoreRelocaliser_CUDA, loading a pretrained forest from a file.
   *
   * \param settings           Pointer to an instance of SettingsContainer used to configure the relocaliser.
   * \param settingsNamespace  The namespace used to read settings from the SettingsContainer.
   * \param forestFilename     The path to the pretrained forest file.
   *
   * \throws std::runtime_error if the forest cannot be loaded.
   */
  ScoreRelocaliser_CUDA(const tvgutil::SettingsContainer_CPtr& settings, const std::string& settingsNamespace, const std::string& forestFilename);

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /** Override */
  virtual void get_predictions_for_leaves(const LeafIndicesImage_CPtr& leafIndices, const ScorePredictionsMemoryBlock_CPtr& leafPredictions,
                                          ScorePredictionsImage_Ptr& outputPredictions) const;
};

}

#endif
