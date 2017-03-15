/**
 * grove: RelocaliserFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation//RelocaliserFactory.h"
#include "relocalisation/cpu/ScoreRelocaliser_CPU.h"

#ifdef WITH_CUDA
#include "relocalisation/cuda/ScoreRelocaliser_CUDA.h"
#endif

using namespace ITMLib;

namespace grove {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

ScoreRelocaliser_Ptr RelocaliserFactory::make_score_relocaliser(ITMLibSettings::DeviceType deviceType, const std::string &forestFilename)
{
  ScoreRelocaliser_Ptr relocaliser;

  if (deviceType == ITMLib::ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    relocaliser.reset(new ScoreRelocaliser_CUDA(forestFilename));
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    relocaliser.reset(new ScoreRelocaliser_CPU(forestFilename));
  }

  return relocaliser;
}

}
