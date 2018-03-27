/**
 * grove: ScoreRelocaliserState.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/base/ScoreRelocaliserState.h"

#include <fstream>

#include <boost/filesystem.hpp>
namespace bf = boost::filesystem;

#include <ORUtils/MemoryBlockPersister.h>
using namespace ORUtils;

namespace grove {

//#################### CONSTRUCTORS ####################

ScoreRelocaliserState::ScoreRelocaliserState()
: lastFeaturesAddedStartIdx(0), reservoirUpdateStartIdx(0)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ScoreRelocaliserState::load_from_disk(const std::string& inputFolder)
{
  const bf::path inputPath(inputFolder);

  // Load the reservoirs.
  exampleReservoirs->load_from_disk(inputFolder);

  // Load the predictions.
  MemoryBlockPersister::LoadMemoryBlock((inputPath / "scorePredictions.bin").string(), *predictionsBlock, MEMORYDEVICE_CPU);

  // If we're using the GPU, copy the predictions across.
  predictionsBlock->UpdateDeviceFromHost();

  // Load the rest of the data.
  const std::string dataFile = (inputPath / "scoreState.txt").string();
  std::ifstream inFile(dataFile.c_str());
  inFile >> lastFeaturesAddedStartIdx >> reservoirUpdateStartIdx;
  if(!inFile) throw std::runtime_error("Error: Couldn't load relocaliser data from " + dataFile);
}

void ScoreRelocaliserState::save_to_disk(const std::string& outputFolder) const
{
  const bf::path outputPath(outputFolder);

  // Save the reservoirs.
  exampleReservoirs->save_to_disk(outputFolder);

  // If we're using the GPU, copy the predictions across to the CPU so that they can be saved.
  predictionsBlock->UpdateHostFromDevice();

  // Save the predictions.
  MemoryBlockPersister::SaveMemoryBlock((outputPath / "scorePredictions.bin").string(), *predictionsBlock, MEMORYDEVICE_CPU);

  // Save the rest of the data.
  const std::string dataFile = (outputPath / "scoreState.txt").string();
  std::ofstream outFile(dataFile.c_str());
  outFile << lastFeaturesAddedStartIdx << ' ' << reservoirUpdateStartIdx;
  if(!outFile) throw std::runtime_error("Error: Couldn't save relocaliser data in " + dataFile);
}

}
