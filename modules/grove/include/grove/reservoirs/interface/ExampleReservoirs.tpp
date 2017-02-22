/**
 * grove: ExampleReservoirs.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ExampleReservoirs.h"

#include <spaint/util/MemoryBlockFactory.h>

namespace grove {

template<typename ExampleType, typename IndexType>
ExampleReservoirs<ExampleType, IndexType>::ExampleReservoirs(
    uint32_t reservoirCapacity, uint32_t reservoirCount, uint32_t rngSeed)
{
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();

  m_capacity = reservoirCapacity;
  m_rngSeed = rngSeed;

  // One row per reservoir
  m_data = mbf.make_image<ExampleType>(Vector2i(reservoirCapacity, reservoirCount));
  m_reservoirsSize = mbf.make_block<int>(reservoirCount);
  m_reservoirsAddCalls = mbf.make_block<int>(reservoirCount);

  m_reservoirsSize->Clear();
  m_reservoirsAddCalls->Clear();
}

template<typename ExampleType, typename IndexType>
ExampleReservoirs<ExampleType, IndexType>::~ExampleReservoirs()
{
}

template<typename ExampleType, typename IndexType>
typename ExampleReservoirs<ExampleType, IndexType>::ExampleImage_CPtr ExampleReservoirs<
ExampleType, IndexType>::get_reservoirs() const
{
  return m_data;
}

template<typename ExampleType, typename IndexType>
ITMIntMemoryBlock_CPtr ExampleReservoirs<ExampleType, IndexType>::get_reservoirs_size() const
{
  return m_reservoirsSize;
}

template<typename ExampleType, typename IndexType>
int ExampleReservoirs<ExampleType, IndexType>::get_reservoirs_count() const
{
  return m_data->noDims.height;
}

template<typename ExampleType, typename IndexType>
int ExampleReservoirs<ExampleType, IndexType>::get_capacity() const
{
  return m_data->noDims.width;
}

template<typename ExampleType, typename IndexType>
void ExampleReservoirs<ExampleType, IndexType>::clear()
{
  m_reservoirsSize->Clear();
  m_reservoirsAddCalls->Clear();
}

}
