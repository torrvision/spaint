/**
 * grove: ScoreRelocaliserState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCORERELOCALISERSTATE
#define H_GROVE_SCORERELOCALISERSTATE

#include <ORUtils/DeviceType.h>

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
class ScoreRelocaliserState
{
  //#################### TYPEDEFS ####################
public:
  typedef ExampleReservoirs<Keypoint3DColour> Reservoirs;
  typedef boost::shared_ptr<Reservoirs> Reservoirs_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The device on which the relocaliser should operate. */
  ORUtils::DeviceType m_deviceType;

  /** The capacity (maximum size) of each example reservoir. */
  uint32_t m_reservoirCapacity;

  /** The total number of example reservoirs used by the relocaliser. */
  uint32_t m_reservoirCount;

  /** The seed for the random number generators used by the example reservoirs. */
  uint32_t m_rngSeed;

  //#################### PUBLIC VARIABLES ####################
public:
  /** The example reservoirs associated with each leaf in the forest. */
  Reservoirs_Ptr exampleReservoirs;

  /** The index of the first reservoir that was clustered when the train function was last called. */
  uint32_t lastExamplesAddedStartIdx;

  /** A memory block storing the 3D modal clusters associated with each leaf in the forest. */
  ScorePredictionsMemoryBlock_Ptr predictionsBlock;

  /** The index of the first reservoir to cluster when the relocaliser is updated. */
  uint32_t reservoirUpdateStartIdx;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs the internal state for a ScoRe-based relocaliser.
   *
   * \param reservoirCount    The total number of example reservoirs used by the relocaliser.
   * \param reservoirCapacity The capacity (maximum size) of each example reservoir.
   * \param deviceType        The device on which the relocaliser should operate.
   * \param rngSeed           The seed for the random number generators used by the example reservoirs.
   */
  ScoreRelocaliserState(uint32_t reservoirCount, uint32_t reservoirCapacity, ORUtils::DeviceType deviceType, uint32_t rngSeed);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Loads the relocaliser state from a folder on disk.
   *
   * \param inputFolder The folder containing the relocaliser state data.
   *
   * \throws std::runtime_error If loading the relocaliser state fails.
   */
  void load_from_disk(const std::string& inputFolder);

  /**
   * \brief Resets the relocaliser state.
   */
  void reset();

  /**
   * \brief Saves the relocaliser state to a folder on disk.
   *
   * \param outputFolder  The folder in which to save the relocaliser state.
   *
   * \throws std::runtime_error If saving the relocaliser state fails.
   */
  void save_to_disk(const std::string& outputFolder) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<ScoreRelocaliserState> ScoreRelocaliserState_Ptr;
typedef boost::shared_ptr<const ScoreRelocaliserState> ScoreRelocaliserState_CPtr;

}

#endif
