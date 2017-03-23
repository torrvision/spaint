/**
 * grove: DecisionForestFactory.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "DecisionForestFactory.h"

#include "cpu/DecisionForest_CPU.h"

#ifdef WITH_CUDA
#include "cuda/DecisionForest_CUDA.h"
#endif

namespace grove {

//#################### STATIC MEMBER FUNCTIONS ####################

template <typename DescriptorType, int TreeCount>
typename DecisionForestFactory<DescriptorType, TreeCount>::Forest_Ptr
    DecisionForestFactory<DescriptorType, TreeCount>::make_forest(ITMLib::ITMLibSettings::DeviceType deviceType,
                                                                  const std::string &fileName)
{
  Forest_Ptr forest;

  if (deviceType == ITMLib::ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    forest.reset(new DecisionForest_CUDA<DescriptorType, TreeCount>(fileName));
#else
    throw std::runtime_error(
        "Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    forest.reset(new DecisionForest_CPU<DescriptorType, TreeCount>(fileName));
  }

  return forest;
}

//#################### SCOREFOREST INTEROP FUNCTIONS ####################

#ifdef WITH_SCOREFORESTS

template <typename DescriptorType, int TreeCount>
typename DecisionForestFactory<DescriptorType, TreeCount>::Forest_Ptr
    DecisionForestFactory<DescriptorType, TreeCount>::make_forest(ITMLib::ITMLibSettings::DeviceType deviceType,
                                                                  const EnsembleLearner &pretrainedForest)
{
  Forest_Ptr forest;

  if (deviceType == ITMLib::ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    forest.reset(new DecisionForest_CUDA<DescriptorType, TreeCount>(pretrainedForest));
#else
    throw std::runtime_error(
        "Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    forest.reset(new DecisionForest_CPU<DescriptorType, TreeCount>(pretrainedForest));
  }

  return forest;
}

#endif

} // namespace grove
