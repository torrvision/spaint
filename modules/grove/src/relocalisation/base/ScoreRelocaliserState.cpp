/**
 * grove: ScoreRelocaliserState.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/base/ScoreRelocaliserState.h"

#include <boost/filesystem.hpp>
namespace bf = boost::filesystem;

#include <ORUtils/MemoryBlockPersister.h>
using namespace ORUtils;

#include <orx/base/MemoryBlockFactory.h>
using namespace orx;

#include "reservoirs/ExampleReservoirsFactory.h"

namespace grove {

//#################### CONSTRUCTORS ####################

ScoreRelocaliserState::ScoreRelocaliserState(uint32_t reservoirCount, uint32_t reservoirCapacity, DeviceType deviceType, uint32_t rngSeed)
: m_deviceType(deviceType), m_reservoirCapacity(reservoirCapacity), m_reservoirCount(reservoirCount), m_rngSeed(rngSeed)
{
  reset();
}

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
  inFile >> lastExamplesAddedStartIdx >> reservoirUpdateStartIdx;
  if(!inFile) throw std::runtime_error("Error: Couldn't load relocaliser data from " + dataFile);
}

void ScoreRelocaliserState::reset()
{
  // Set up the reservoirs if they aren't currently allocated.
  if(!exampleReservoirs)
  {
    exampleReservoirs = ExampleReservoirsFactory<Keypoint3DColour>::make_reservoirs(m_reservoirCount, m_reservoirCapacity, m_deviceType, m_rngSeed);
  }

  // Set up the predictions block if it isn't currently allocated.
  if(!predictionsBlock)
  {
    predictionsBlock = MemoryBlockFactory::instance().make_block<ScorePrediction>(m_reservoirCount);
  }

  exampleReservoirs->reset();
  lastExamplesAddedStartIdx = 0;
  predictionsBlock->Clear();
  reservoirUpdateStartIdx = 0;
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
  outFile << lastExamplesAddedStartIdx << ' ' << reservoirUpdateStartIdx;
  if(!outFile) throw std::runtime_error("Error: Couldn't save relocaliser data in " + dataFile);
}

}
