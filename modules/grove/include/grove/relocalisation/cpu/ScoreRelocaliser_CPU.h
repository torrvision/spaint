/**
 * grove: ScoreRelocaliser_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCORERELOCALISER_CPU
#define H_GROVE_SCORERELOCALISER_CPU

#include "../interface/ScoreRelocaliser.h"

namespace grove {

/**
 * \brief An instance of this class can be used to relocalise a camera in a 3D scene on the CPU, using the approach described
 *        in "On-the-Fly Adaptation of Regression Forests for Online Camera Relocalisation" (Cavallari et al., 2017).
 */
class ScoreRelocaliser_CPU : public ScoreRelocaliser
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a CPU-based SCoRe relocaliser by loading a pre-trained forest from a file.
   *
   * \param forestFilename  The name of the file from which to load the pre-trained forest.
   * \param settings        The settings used to configure the relocaliser.
   *
   * \throws std::runtime_error If the forest cannot be loaded.
   */
  ScoreRelocaliser_CPU(const std::string& forestFilename, const tvgutil::SettingsContainer_CPtr& settings);

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /** Override */
  virtual void merge_predictions_for_keypoints(const LeafIndicesImage_CPtr& leafIndices, const ScorePredictionsMemoryBlock_CPtr& leafPredictions,
                                               ScorePredictionsImage_Ptr& outputPredictions) const;
};

}

#endif
