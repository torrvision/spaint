/**
 * grove: ScoreRelocaliserState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCORERELOCALISERSTATE
#define H_GROVE_SCORERELOCALISERSTATE

#include <boost/shared_ptr.hpp>

#include "../../keypoints/Keypoint3DColour.h"
#include "../../reservoirs/interface/ExampleReservoirs.h"
#include "../../scoreforests/ScorePrediction.h"

namespace grove {

/**
 * \brief An instance of this struct holds all data required to perform training and relocalisation with a Score-based method:
 *        - example reservoirs, used when training the relocaliser.
 *        - predictions block, containing the 3D Modal clusters used for the actual camera relocalisation.
 */
struct ScoreRelocaliserState
{
  //#################### TYPEDEFS ####################
  typedef Keypoint3DColour ExampleType;
  typedef ScorePrediction PredictionType;

  typedef ExampleReservoirs<ExampleType> Reservoirs;
  typedef boost::shared_ptr<Reservoirs> Reservoirs_Ptr;

  //#################### CONSTRUCTORS ####################
  ScoreRelocaliserState() : lastFeaturesAddedStartIdx(0), reservoirUpdateStartIdx(0) {}

  //#################### MEMBER VARIABLES ####################
  /** The example reservoirs associated to every leaf in the forest. */
  Reservoirs_Ptr exampleReservoirs;

  /** A block of memory storing the 3D modal clusters associated to each leaf in the forest. */
  ScorePredictionsBlock_Ptr predictionsBlock;

  // Update-related data
  /** The index of the reservoir that had been updated when the integration function has been called. */
  uint32_t lastFeaturesAddedStartIdx;

  /** The index of the reservoir to cluster when the idle_update will be called. */
  uint32_t reservoirUpdateStartIdx;
};

//#################### TYPEDEFS ####################
typedef boost::shared_ptr<ScoreRelocaliserState> ScoreRelocaliserState_Ptr;
typedef boost::shared_ptr<const ScoreRelocaliserState> ScoreRelocaliserState_CPtr;

}

#endif
