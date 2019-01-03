/**
 * grove: DecisionForestFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_DECISIONFORESTFACTORY
#define H_GROVE_DECISIONFORESTFACTORY

#include <ORUtils/DeviceType.h>

#include "interface/DecisionForest.h"

namespace grove {

/**
 * \brief This struct can be used to construct decision forests.
 *
 * \tparam DescriptorType The type of descriptor used to find the leaves. Must have a floating-point member array named "data".
 * \tparam TreeCount      The number of trees in the forest. Fixed at compilation time to allow the definition of a data type
 *                        representing the leaf indices.
 */
template <typename DescriptorType, int TreeCount>
struct DecisionForestFactory
{
  //#################### TYPEDEFS ####################

  typedef DecisionForest<DescriptorType,TreeCount> Forest;
  typedef boost::shared_ptr<Forest> Forest_Ptr;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Constructs a decision forest by loading the branching structure of a pre-trained forest from a file on disk.
   *
   * \param filename   The path to the file containing the forest.
   * \param deviceType The device on which the decision forest should operate.
   * \return           The constructed forest.
   *
   * \throws std::runtime_error If the forest cannot be loaded.
   */
  static Forest_Ptr make_forest(const std::string& filename, ORUtils::DeviceType deviceType);

#ifdef WITH_SCOREFORESTS
  /**
   * \brief Constructs a decision forest by converting an EnsembleLearner that was pre-trained using ScoreForests.
   *
   * \param pretrainedForest  The pre-trained forest to convert.
   * \param deviceType        The device on which the decision forest should operate.
   * \return                  The constructed forest.
   *
   * \throws std::runtime_error If the forest cannot be converted.
   */
  static Forest_Ptr make_forest(const EnsembleLearner& pretrainedForest, DeviceType deviceType);
#endif

  /**
   * \brief Constructs a balanced decision forest with random split functions, using parameters specified by the user.
   *
   * \param settings   The settings to use to create the forest.
   * \param deviceType The device on which the decision forest should operate.
   * \return           The constructed forest.
   *
   * \throws std::runtime_error If the forest cannot be created.
   */
  static Forest_Ptr make_randomly_generated_forest(const tvgutil::SettingsContainer_CPtr& settings, ORUtils::DeviceType deviceType);
};

}

#endif
