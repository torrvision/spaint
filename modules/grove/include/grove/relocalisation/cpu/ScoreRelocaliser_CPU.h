/**
 * grove: ScoreRelocaliser_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCORERELOCALISERCPU
#define H_GROVE_SCORERELOCALISERCPU

#include "../interface/ScoreRelocaliser.h"

namespace grove {

/**
 * \brief An instance of this class allows the relocalisation of camera pose used to acquire RGB-D
 *        image pairs, on the CPU, according to the method described in:
 *
 *        "On-the-Fly Adaptation of Regression Forests for Online Camera Relocalisation" by
 *        Tommaso Cavallari, Stuart Golodetz*, Nicholas A. Lord*,
 *        Julien Valentin, Luigi Di Stefano and Philip H. S. Torr
 */
class ScoreRelocaliser_CPU : public ScoreRelocaliser
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an instance of ScoreRelocaliser_CPU, loading a pretrained forest from a file.
   *
   * \param forestFilename The path to the pretrained forest file.
   *
   * \throws std::runtime_error if the forest cannot be loaded.
   */
  ScoreRelocaliser_CPU(const tvgutil::SettingsContainer_CPtr& settings, const std::string& forestFilename);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual ScorePrediction get_raw_prediction(uint32_t treeIdx, uint32_t leafIdx) const;

  /** Override */
  virtual std::vector<Keypoint3DColour> get_reservoir_contents(uint32_t treeIdx, uint32_t leafIdx) const;

  /** Override */
  virtual void reset();

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /** Override */
  virtual void get_predictions_for_leaves(const LeafIndicesImage_CPtr& leafIndices, const ScorePredictionsMemoryBlock_CPtr& leafPredictions,
                                          ScorePredictionsImage_Ptr& outputPredictions) const;
};

}

#endif
