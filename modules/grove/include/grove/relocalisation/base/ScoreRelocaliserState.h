/**
 * grove: ScoreRelocaliserState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCORERELOCALISERSTATE
#define H_GROVE_SCORERELOCALISERSTATE

#include "../../keypoints/Keypoint3DColour.h"
#include "../../reservoirs/interface/ExampleReservoirs.h"
#include "../../scoreforests/ScorePrediction.h"

namespace grove {

/**
 * \brief An instance of this struct holds all of the data required to perform training and relocalisation with a SCoRe-based relocaliser.
 *
 * In particular, it holds:
 *
 * - The example reservoirs used when training the relocaliser.
 * - A memory block containing the 3D modal clusters used for the actual camera relocalisation.
 */
struct ScoreRelocaliserState
{
  //#################### TYPEDEFS ####################

  typedef ExampleReservoirs<Keypoint3DColour> Reservoirs;
  typedef boost::shared_ptr<Reservoirs> Reservoirs_Ptr;

  //#################### PUBLIC MEMBER VARIABLES ####################

  /** The example reservoirs associated with each leaf in the forest. */
  Reservoirs_Ptr exampleReservoirs;

  /** The index of the reservoir that was updated when the train function was last called. */
  uint32_t lastFeaturesAddedStartIdx;

  /** A memory block storing the 3D modal clusters associated with each leaf in the forest. */
  ScorePredictionsMemoryBlock_Ptr predictionsBlock;

  /** The index of the first reservoir to cluster when the relocaliser is updated. */
  uint32_t reservoirUpdateStartIdx;

  //#################### CONSTRUCTORS ####################

  ScoreRelocaliserState();

  //#################### PUBLIC MEMBER FUNCTIONS ####################

  /**
   * \brief Loads the relocaliser state from a folder on disk.
   *
   * \param inputFolder The folder containing the relocaliser state data.
   *
   * \throws std::runtime_error If loading the relocaliser state fails.
   */
  void load_from_disk(const std::string& inputFolder);

  /**
   * \brief Saves the relocaliser state to a folder on disk.
   *
   * \param outputFolder  The folder in which to save the relocaliser state.
   *
   * \throws std::runtime_error  If saving the relocaliser state fails.
   */
  void save_to_disk(const std::string& outputFolder) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<ScoreRelocaliserState> ScoreRelocaliserState_Ptr;
typedef boost::shared_ptr<const ScoreRelocaliserState> ScoreRelocaliserState_CPtr;

}

#endif
