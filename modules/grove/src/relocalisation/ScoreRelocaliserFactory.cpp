/**
 * grove: ScoreRelocaliserFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/ScoreRelocaliserFactory.h"
using namespace tvgutil;

#include "relocalisation/cpu/ScoreForestRelocaliser_CPU.h"
#include "relocalisation/cpu/ScoreGTRelocaliser_CPU.h"
#ifdef WITH_TORCH
  #include "relocalisation/cpu/ScoreNetRelocaliser_CPU.h"
#endif

#ifdef WITH_CUDA
  #include "relocalisation/cuda/ScoreForestRelocaliser_CUDA.h"
  #include "relocalisation/cuda/ScoreGTRelocaliser_CUDA.h"
  #ifdef WITH_TORCH
    #include "relocalisation/cuda/ScoreNetRelocaliser_CUDA.h"
  #endif
#endif

namespace grove {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

ScoreRelocaliser_Ptr ScoreRelocaliserFactory::make_score_relocaliser(const std::string& relocaliserType, const std::string& relocaliserNamespace,
                                                                     const SettingsContainer_CPtr& settings, ORUtils::DeviceType deviceType)
{
  ScoreRelocaliser_Ptr relocaliser;

  if(deviceType == ORUtils::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    if(relocaliserType == "forest") relocaliser.reset(new ScoreForestRelocaliser_CUDA(settings, relocaliserNamespace));
    else if(relocaliserType == "gt") relocaliser.reset(new ScoreGTRelocaliser_CUDA(settings, relocaliserNamespace));
  #ifdef WITH_TORCH
    else if(relocaliserType == "net") relocaliser.reset(new ScoreNetRelocaliser_CUDA(settings, relocaliserNamespace));
  #endif
    else throw std::runtime_error("Error: Unknown or unavailable relocaliser type");
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    if(relocaliserType == "forest") relocaliser.reset(new ScoreForestRelocaliser_CPU(settings, relocaliserNamespace));
    else if(relocaliserType == "gt") relocaliser.reset(new ScoreGTRelocaliser_CPU(settings, relocaliserNamespace));
  #ifdef WITH_TORCH
    else if(relocaliserType == "net") relocaliser.reset(new ScoreNetRelocaliser_CPU(settings, relocaliserNamespace));
  #endif
    else throw std::runtime_error("Error: Unknown or unavailable relocaliser type");
  }

  return relocaliser;
}

}
