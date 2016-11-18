/**
 * spaint: ExampleReservoirs.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/interface/ExampleReservoirs.h"
#include "util/MemoryBlockFactory.h"

namespace spaint
{

ExampleReservoirs::ExampleReservoirs(size_t capacity, size_t nbLeaves,
    uint32_t rngSeed)
{
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();

  m_reservoirCapacity = capacity;
  m_rngSeed = rngSeed;

  // One row per leaf
  m_data = mbf.make_image<ExampleType>(Vector2i(capacity, nbLeaves));
  m_reservoirsSize = mbf.make_image<int>(Vector2i(1, nbLeaves));
  m_reservoirsAddCalls = mbf.make_image<int>(Vector2i(1, nbLeaves));

  m_reservoirsSize->Clear();
  m_reservoirsAddCalls->Clear();
}

ExampleReservoirs::~ExampleReservoirs()
{
}

ExampleReservoirs::ExampleReservoirsImage_CPtr ExampleReservoirs::get_reservoirs() const
{
  return m_data;
}

ITMIntImage_CPtr ExampleReservoirs::get_reservoirs_size() const
{
  return m_reservoirsSize;
}

int ExampleReservoirs::get_reservoirs_count() const
{
  return m_data->noDims.height;
}

int ExampleReservoirs::get_capacity() const
{
  return m_data->noDims.width;
}

}
