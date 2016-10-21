/**
 * spaint: GPUReservoir.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_GPURESERVOIR
#define H_SPAINT_GPURESERVOIR

#include "../../util/MemoryBlockFactory.h"
#include "ITMLib/Utils/ITMMath.h"

#include <tvgutil/numbers/RandomNumberGenerator.h>

namespace spaint
{

template<typename ExampleType>
class GPUReservoir
{
public:
  typedef ORUtils::MemoryBlock<ExampleType> ExampleMemoryBlock;
  typedef boost::shared_ptr<ExampleMemoryBlock> ExampleMemoryBlock_Ptr;
  typedef boost::shared_ptr<const ExampleMemoryBlock> ExampleMemoryBlock_CPtr;

  explicit GPUReservoir(size_t capacity, uint32_t rngSeed = 42)
  {
    m_data = MemoryBlockFactory::instance().make_block<ExampleType>(capacity);
    m_data->dataSize = 0;
    m_allocatedSize = capacity;
    m_totalAddCalls = 0;
    m_rngSeed = rngSeed;
    m_rng.reset(new tvgutil::RandomNumberGenerator(m_rngSeed));
  }

  const ExampleMemoryBlock_CPtr get_examples() const
  {
    return m_data;
  }

  int get_size() const
  {
    return m_data->dataSize;
  }

  int get_capacity() const
  {
    return m_allocatedSize;
  }

  bool add_example(const ExampleType &example)
  {
    bool added = false;

    if (m_data->dataSize < m_allocatedSize)
    {
      // Just add it in the first available slot
      m_data->GetData(MEMORYDEVICE_CPU)[m_data->dataSize++] = example;
      added = true;
    }
    else
    {
      size_t k = m_rng->generate_int_from_uniform(0, static_cast<int>(m_totalAddCalls) - 1);
      if(k < m_allocatedSize)
      {
        m_data->GetData(MEMORYDEVICE_CPU)[k] = example;
        added = true;
      }
    }

    m_totalAddCalls++;
    return added;
  }

  void clear()
  {
    // No need to actually do anything
    m_data->dataSize = 0;
    m_totalAddCalls = 0;
    m_rng.reset(new tvgutil::RandomNumberGenerator(m_rngSeed));
  }

private:
  ExampleMemoryBlock_Ptr m_data;

  // because the dataSize of the memoryBlock will change depending on the number of stored elements
  size_t m_allocatedSize;
  size_t m_totalAddCalls;

  uint32_t m_rngSeed;
  tvgutil::RandomNumberGenerator_Ptr m_rng;
};

typedef GPUReservoir<Vector3f> PositionReservoir;
typedef boost::shared_ptr<PositionReservoir> PositionReservoir_Ptr;
typedef boost::shared_ptr<const PositionReservoir> PositionReservoir_CPtr;

}

#endif
