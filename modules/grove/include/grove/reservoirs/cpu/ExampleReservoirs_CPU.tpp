/**
 * grove: ExampleReservoirs_CPU.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ExampleReservoirs_CPU.h"

#include <boost/filesystem.hpp>
namespace bf = boost::filesystem;

#include <ORUtils/MemoryBlockPersister.h>

#include <orx/base/MemoryBlockFactory.h>

#include "../shared/ExampleReservoirs_Shared.h"

namespace grove {

//#################### CONSTRUCTORS ####################

template <typename ExampleType>
ExampleReservoirs_CPU<ExampleType>::ExampleReservoirs_CPU(uint32_t reservoirCount, uint32_t reservoirCapacity, uint32_t rngSeed)
: ExampleReservoirs<ExampleType>(reservoirCount, reservoirCapacity, rngSeed)
{
  orx::MemoryBlockFactory& mbf = orx::MemoryBlockFactory::instance();
  m_rngs = mbf.make_block<CPURNG>();

  reset();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename ExampleType>
void ExampleReservoirs_CPU<ExampleType>::reset()
{
  ExampleReservoirs<ExampleType>::reset();
  reinit_rngs();
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename ExampleType>
void ExampleReservoirs_CPU<ExampleType>::accept(const Visitor& visitor)
{
  visitor.visit(*this);
}

template <typename ExampleType>
template <int ReservoirIndexCount>
void ExampleReservoirs_CPU<ExampleType>::add_examples_sub(const ExampleImage_CPtr& examples, const boost::shared_ptr<const ORUtils::Image<ORUtils::VectorX<int,ReservoirIndexCount> > >& reservoirIndices)
{
  const Vector2i imgSize = examples->noDims;
  const size_t exampleCount = imgSize.width * imgSize.height;

  // Check that we have enough random number generators and reallocate them if not.
  if(m_rngs->dataSize < exampleCount)
  {
    m_rngs->Resize(exampleCount);
    reinit_rngs();
  }

  const ExampleType *examplesPtr = examples->GetData(MEMORYDEVICE_CPU);
  int *reservoirAddCalls = this->m_reservoirAddCalls->GetData(MEMORYDEVICE_CPU);
  const ORUtils::VectorX<int,ReservoirIndexCount> *reservoirIndicesPtr = reservoirIndices->GetData(MEMORYDEVICE_CPU);
  int *reservoirSizes = this->m_reservoirSizes->GetData(MEMORYDEVICE_CPU);
  ExampleType *reservoirs = this->m_reservoirs->GetData(MEMORYDEVICE_CPU);
  CPURNG *rngs = m_rngs->GetData(MEMORYDEVICE_CPU);

  // Add each example to the relevant reservoirs.
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int y = 0; y < imgSize.height; ++y)
  {
    for(int x = 0; x < imgSize.width; ++x)
    {
      const int linearIdx = y * imgSize.x + x;

      add_example_to_reservoirs(
        examplesPtr[linearIdx], reservoirIndicesPtr[linearIdx].v, ReservoirIndexCount, reservoirs,
        reservoirSizes, reservoirAddCalls, this->m_reservoirCapacity, rngs[linearIdx]
      );
    }
  }
}

template<typename ExampleType>
void ExampleReservoirs_CPU<ExampleType>::load_from_disk_sub(const std::string& inputFolder)
{
  // Load the RNG states.
  bf::path inputPath(inputFolder);
  ORUtils::MemoryBlockPersister::LoadMemoryBlock((inputPath / "reservoirRngs.bin").string(), *m_rngs, MEMORYDEVICE_CPU);
}

template <typename ExampleType>
void ExampleReservoirs_CPU<ExampleType>::reinit_rngs()
{
  // Reinitialise each random number generator based on the specified seed.
  CPURNG *rngs = m_rngs->GetData(MEMORYDEVICE_CPU);
  const uint32_t rngCount = static_cast<uint32_t>(m_rngs->dataSize);
  for(uint32_t i = 0; i < rngCount; ++i)
  {
    rngs[i].reset(this->m_rngSeed + i);
  }
}

template<typename ExampleType>
void ExampleReservoirs_CPU<ExampleType>::save_to_disk_sub(const std::string& outputFolder)
{
  // Save the RNG states.
  bf::path outputPath(outputFolder);
  ORUtils::MemoryBlockPersister::SaveMemoryBlock((outputPath / "reservoirRngs.bin").string(), *m_rngs, MEMORYDEVICE_CPU);
}

}
