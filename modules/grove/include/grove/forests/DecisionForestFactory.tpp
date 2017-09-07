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

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

template <typename DescriptorType, int TreeCount>
typename DecisionForestFactory<DescriptorType,TreeCount>::Forest_Ptr
DecisionForestFactory<DescriptorType,TreeCount>::make_forest(const std::string& filename, ITMLib::ITMLibSettings::DeviceType deviceType)
{
  Forest_Ptr forest;

  if(deviceType == ITMLib::ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    forest.reset(new DecisionForest_CUDA<DescriptorType,TreeCount>(filename));
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    forest.reset(new DecisionForest_CPU<DescriptorType,TreeCount>(filename));
  }

  return forest;
}

#ifdef WITH_SCOREFORESTS
template <typename DescriptorType, int TreeCount>
typename DecisionForestFactory<DescriptorType,TreeCount>::Forest_Ptr
DecisionForestFactory<DescriptorType,TreeCount>::make_forest(const EnsembleLearner& pretrainedForest, ITMLib::ITMLibSettings::DeviceType deviceType)
{
  Forest_Ptr forest;

  if(deviceType == ITMLib::ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    forest.reset(new DecisionForest_CUDA<DescriptorType,TreeCount>(pretrainedForest));
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    forest.reset(new DecisionForest_CPU<DescriptorType,TreeCount>(pretrainedForest));
  }

  return forest;
}
#endif

}
