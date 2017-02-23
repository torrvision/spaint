/**
 * grove: ExampleReservoirs.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ExampleReservoirs.h"

#include <spaint/util/MemoryBlockFactory.h>

namespace grove {

//#################### CONSTRUCTORS ####################

template<typename ExampleType, typename IndexType>
ExampleReservoirs<ExampleType, IndexType>::ExampleReservoirs(
    uint32_t reservoirCapacity, uint32_t reservoirCount, uint32_t rngSeed)
{
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();

  m_capacity = reservoirCapacity;
  m_reservoirCount = reservoirCount;
  m_rngSeed = rngSeed;

  // One row per reservoir, width equal to the capacity.
  m_data = mbf.make_image<ExampleType>(Vector2i(reservoirCapacity, reservoirCount));
  m_reservoirsSize = mbf.make_block<int>(reservoirCount);
  m_reservoirsAddCalls = mbf.make_block<int>(reservoirCount);

  // No calls to virtual clear() in the constructor.
  // No need to clear m_data if the size is 0.
  m_reservoirsSize->Clear();
  m_reservoirsAddCalls->Clear();
}

//#################### DESTRUCTOR ####################

template<typename ExampleType, typename IndexType>
ExampleReservoirs<ExampleType, IndexType>::~ExampleReservoirs()
{
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template<typename ExampleType, typename IndexType>
uint32_t ExampleReservoirs<ExampleType, IndexType>::get_capacity() const
{
  return m_capacity;
}

template<typename ExampleType, typename IndexType>
typename ExampleReservoirs<ExampleType, IndexType>::ExampleImage_CPtr ExampleReservoirs<
ExampleType, IndexType>::get_reservoirs() const
{
  return m_data;
}

template<typename ExampleType, typename IndexType>
uint32_t ExampleReservoirs<ExampleType, IndexType>::get_reservoirs_count() const
{
  return m_reservoirCount;
}

template<typename ExampleType, typename IndexType>
ITMIntMemoryBlock_CPtr ExampleReservoirs<ExampleType, IndexType>::get_reservoirs_size() const
{
  return m_reservoirsSize;
}

//#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################

template<typename ExampleType, typename IndexType>
void ExampleReservoirs<ExampleType, IndexType>::clear()
{
  m_reservoirsSize->Clear();
  m_reservoirsAddCalls->Clear();
}

}
