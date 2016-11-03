/**
 * relocconverter: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "spaint/randomforest/cuda/GPUForest_CUDA.h"

#include <iostream>

#include <DatasetRGBDInfiniTAM.hpp>

using namespace spaint;

int main(int argc, char *argv[])
{
  const int nbTrees = GPUFOREST_NTREES;
  const float proportionOfDataGivenToLearner = 1.0f;
  const std::string learnerType = "DFBP";
  const bool loadSavedForest = true;
  const int learnFromTree = 0;
  const bool loadFeatures = false;
  const int randomSeed = 42;

  if (argc < 4)
  {
    std::cerr << "Usage: " << argv[0]
        << " \"scoreforest config file\" \"7scenes base path\" \"output forest filename\""
        << std::endl;
    return 1;
  }

  const std::string configFile = argv[1];
  const std::string dataPath = argv[2];
  const std::string outputFile = argv[3];

  boost::shared_ptr<DatasetRGBD7Scenes> m_dataset(
      new DatasetRGBDInfiniTAM(configFile, dataPath, nbTrees,
          proportionOfDataGivenToLearner, learnerType, loadSavedForest,
          loadSavedForest, loadFeatures, randomSeed));
  m_dataset->LoadForest();

  GPUForest_Ptr m_gpuForest(new GPUForest_CUDA(*m_dataset->GetForest()));

  std::cout << "Saving forest in: " << outputFile << std::endl;
  m_gpuForest->save_structure_to_file(outputFile);

  return 0;
}
