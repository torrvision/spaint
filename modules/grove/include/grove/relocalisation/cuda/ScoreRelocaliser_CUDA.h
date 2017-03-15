/**
 * grove: ScoreRelocaliser_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCORERELOCALISERCUDA
#define H_GROVE_SCORERELOCALISERCUDA

#include "../interface/ScoreRelocaliser.h"

namespace grove {

class ScoreRelocaliser_CUDA : public ScoreRelocaliser
{
public:
  ScoreRelocaliser_CUDA(const std::string &forestFilename);

protected:
  virtual void get_predictions_for_leaves(const LeafIndicesImage_CPtr &leafIndices, const ScorePredictionsBlock_CPtr &leafPredictions, ScorePredictionsImage_Ptr &outputPredictions) const;
};

}

#endif
