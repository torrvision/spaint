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
  ScoreRelocaliser_CPU(const std::string &forestFilename);

  //#################### PROTECTED VIRTUAL MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Each keypoint/descriptor pair extracted from the input RGB-D image pairs determines a leaf in a tree of the
   *        forest. This function merges the 3D modal clusters associated to multiple leaves coming from different trees
   *        in the forest in a single prediction for each keypoint/descriptor pair.
   *
   * \param leafIndices       Indices of the forest leafs predicted from a keypoint/descriptor pair.
   * \param leafPredictions   A memory block containing all the 3D modal clusters associated to the forest.
   * \param outputPredictions An image wherein each element represent the modal clsters associated to the predicted
   *                          leaves.
   */
  virtual void get_predictions_for_leaves(const LeafIndicesImage_CPtr &leafIndices,
                                          const ScorePredictionsBlock_CPtr &leafPredictions,
                                          ScorePredictionsImage_Ptr &outputPredictions) const;
};

} // namespace grove

#endif // H_GROVE_SCORERELOCALISERCPU
