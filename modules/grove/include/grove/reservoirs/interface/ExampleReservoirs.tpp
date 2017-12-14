/**
 * grove: ExampleReservoirs.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ExampleReservoirs.h"

#include <boost/filesystem.hpp>
namespace bf = boost::filesystem;

#include <itmx/base/MemoryBlockFactory.h>

#include <ORUtils/MemoryBlockPersister.h>
using namespace ORUtils;

namespace grove {

//#################### CONSTRUCTORS ####################

template <typename ExampleType>
ExampleReservoirs<ExampleType>::ExampleReservoirs(uint32_t reservoirCount, uint32_t reservoirCapacity, uint32_t rngSeed)
: m_reservoirCapacity(reservoirCapacity), m_reservoirCount(reservoirCount), m_rngSeed(rngSeed)
{
  itmx::MemoryBlockFactory& mbf = itmx::MemoryBlockFactory::instance();

  // One row per reservoir, width equal to the capacity.
  m_reservoirs = mbf.make_image<ExampleType>(Vector2i(reservoirCapacity, reservoirCount));
  m_reservoirAddCalls = mbf.make_block<int>(reservoirCount);
  m_reservoirSizes = mbf.make_block<int>(reservoirCount);
}

//#################### DESTRUCTOR ####################

template <typename ExampleType>
ExampleReservoirs<ExampleType>::~ExampleReservoirs()
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename ExampleType>
template <int ReservoirIndexCount>
void ExampleReservoirs<ExampleType>::add_examples(const ExampleImage_CPtr& examples, const boost::shared_ptr<const ORUtils::Image<ORUtils::VectorX<int,ReservoirIndexCount> > >& reservoirIndices)
{
  // Check the preconditions.
  if(examples->noDims != reservoirIndices->noDims)
  {
    throw std::invalid_argument("The example and reservoir indices images should have the same size.");
  }

  accept(AddExamplesCaller<ReservoirIndexCount>(examples, reservoirIndices));
}

template <typename ExampleType>
template <int ReservoirIndexCount>
void ExampleReservoirs<ExampleType>::add_examples(const ExampleImage_CPtr& examples, const boost::shared_ptr<ORUtils::Image<ORUtils::VectorX<int,ReservoirIndexCount> > >& reservoirIndices)
{
  const boost::shared_ptr<const ORUtils::Image<ORUtils::VectorX<int,ReservoirIndexCount> > > reservoirIndicesConst = reservoirIndices;
  add_examples(examples, reservoirIndicesConst);
}

template <typename ExampleType>
uint32_t ExampleReservoirs<ExampleType>::get_reservoir_capacity() const
{
  return m_reservoirCapacity;
}

template <typename ExampleType>
typename ExampleReservoirs<ExampleType>::ExampleImage_CPtr ExampleReservoirs<ExampleType>::get_reservoirs() const
{
  return m_reservoirs;
}

template <typename ExampleType>
uint32_t ExampleReservoirs<ExampleType>::get_reservoir_count() const
{
  return m_reservoirCount;
}

template <typename ExampleType>
ITMIntMemoryBlock_CPtr ExampleReservoirs<ExampleType>::get_reservoir_sizes() const
{
  return m_reservoirSizes;
}

template<typename ExampleType>
void ExampleReservoirs<ExampleType>::load_from_disk(const std::string& inputFolder)
{
  bf::path inputPath(inputFolder);

  // Load the data on the CPU.
  MemoryBlockPersister::LoadImageFrom((inputPath / "reservoirs.bin").string(), *m_reservoirs, MEMORYDEVICE_CPU);
  MemoryBlockPersister::LoadMemoryBlock((inputPath / "reservoirAddCalls.bin").string(), *m_reservoirAddCalls, MEMORYDEVICE_CPU);
  MemoryBlockPersister::LoadMemoryBlock((inputPath / "reservoirSizes.bin").string(), *m_reservoirSizes, MEMORYDEVICE_CPU);

  // Update everything on the GPU, NOP in case we are using the CPU version of the class.
  m_reservoirs->UpdateDeviceFromHost();
  m_reservoirAddCalls->UpdateDeviceFromHost();
  m_reservoirSizes->UpdateDeviceFromHost();

  // Call the overridable hook function to allow subclasses to perform additional loading steps.
  load_from_disk_sub(inputFolder);
}

template <typename ExampleType>
void ExampleReservoirs<ExampleType>::reset()
{
  // Note: There is no need to clear m_reservoirs - it is sufficient to simply reset the size of each reservoir to 0.
  m_reservoirAddCalls->Clear();
  m_reservoirSizes->Clear();
}

template<typename ExampleType>
void ExampleReservoirs<ExampleType>::save_to_disk(const std::string& outputFolder)
{
  bf::path outputPath(outputFolder);

  // Update everything on the CPU, NOP in case we are using the CPU version of the class.
  m_reservoirs->UpdateHostFromDevice();
  m_reservoirAddCalls->UpdateHostFromDevice();
  m_reservoirSizes->UpdateHostFromDevice();

  // Save the data.
  MemoryBlockPersister::SaveImage((outputPath / "reservoirs.bin").string(), *m_reservoirs, MEMORYDEVICE_CPU);
  MemoryBlockPersister::SaveMemoryBlock((outputPath / "reservoirAddCalls.bin").string(), *m_reservoirAddCalls, MEMORYDEVICE_CPU);
  MemoryBlockPersister::SaveMemoryBlock((outputPath / "reservoirSizes.bin").string(), *m_reservoirSizes, MEMORYDEVICE_CPU);

  // Call the overridable hook function to allow subclasses to perform additional saving steps.
  save_to_disk_sub(outputFolder);
}

}
