/**
 * grove: DecisionForestFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_DECISIONFORESTFACTORY
#define H_GROVE_DECISIONFORESTFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/DecisionForest.h"

namespace grove {

/**
 * \brief This struct can be used to construct decision forests.
 *
 * \param DescriptorType The type of descriptors used to evaluate the forest (must have a floating-point member array
 *                       named "data").
 * \param TreeCount      The number of trees in the forest (fixed at compilation time to allow the definition of a data
 *                       type representing the leaf indices).
 */
template <typename DescriptorType, int TreeCount>
struct DecisionForestFactory
{
  //#################### TYPEDEFS ####################
  typedef DecisionForest<DescriptorType, TreeCount> Forest;
  typedef boost::shared_ptr<Forest> Forest_Ptr;
  typedef boost::shared_ptr<const Forest> Forest_CPtr;

  //#################### STATIC MEMBER FUNCTIONS ####################
  /**
   * \brief Creates a DecisionForest loading its structure from a file.
   *
   * \param deviceType The device on which the DecisionForest should operate.
   * \param fileName   The path to the forest to load.
   *
   * \return A decision forest.
   *
   * \throw std::runtime_error if the forest cannot be loaded.
   */
  static Forest_Ptr make_forest(ITMLib::ITMLibSettings::DeviceType deviceType, const std::string &fileName);

//#################### SCOREFOREST INTEROP FUNCTIONS ####################
#ifdef WITH_SCOREFORESTS
  /**
   * \brief Creates a DecisionForest copying its structure from a ScoreForest forest.
   *
   * \param deviceType The device on which the DecisionForest should operate.
   * \param fileName   The forest loaded by the ScoreForests project.
   *
   * \return A decision forest.
   *
   * \throw std::runtime_error if the forest cannot be converted.
   */
  static Forest_Ptr make_forest(ITMLib::ITMLibSettings::DeviceType deviceType, const EnsembleLearner &pretrainedForest);
#endif
};

} // namespace grove

#endif
