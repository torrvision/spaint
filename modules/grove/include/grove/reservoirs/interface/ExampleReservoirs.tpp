/**
 * grove: ExampleReservoirs.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ExampleReservoirs.h"

#include <itmx/base/MemoryBlockFactory.h>

namespace grove {

//#################### CONSTRUCTORS ####################

template <typename ExampleType>
ExampleReservoirs<ExampleType>::ExampleReservoirs(uint32_t reservoirCapacity, uint32_t reservoirCount, uint32_t rngSeed)
: m_reservoirCapacity(reservoirCapacity), m_reservoirCount(reservoirCount), m_rngSeed(rngSeed)
{
  itmx::MemoryBlockFactory& mbf = itmx::MemoryBlockFactory::instance();

  // One row per reservoir, width equal to the capacity.
  m_reservoirs = mbf.make_image<ExampleType>(Vector2i(reservoirCapacity, reservoirCount));
  m_reservoirAddCalls = mbf.make_block<int>(reservoirCount);
  m_reservoirSizes = mbf.make_block<int>(reservoirCount);

  // No calls to virtual clear() in the constructor.
  // No need to clear m_reservoirs if the size is 0.
  m_reservoirSizes->Clear();
  m_reservoirAddCalls->Clear();
}

//#################### DESTRUCTOR ####################

template <typename ExampleType>
ExampleReservoirs<ExampleType>::~ExampleReservoirs()
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename ExampleType>
template <int IndexLength>
void ExampleReservoirs<ExampleType>::add_examples(const ExampleImage_CPtr& examples, const boost::shared_ptr<const ORUtils::Image<ORUtils::VectorX<int, IndexLength> > >& reservoirIndices)
{
  // Check preconditions.
  if(examples->noDims != reservoirIndices->noDims)
    throw std::invalid_argument("The example and indices images should have the same size.");

  // Compute the step between elements of the reservoirIndices image.
  const uint32_t indexStep = sizeof(ORUtils::VectorX<int, IndexLength>);

  // Extract raw memory pointers.
  const char *reservoirIndicesCPU = reinterpret_cast<const char*>(reservoirIndices->GetData(MEMORYDEVICE_CPU));
  const char *reservoirIndicesCUDA = reinterpret_cast<const char*>(reservoirIndices->GetData(MEMORYDEVICE_CUDA));

  // Call the non-templated virtual function.
  add_examples(examples, reservoirIndicesCPU, reservoirIndicesCUDA, IndexLength, indexStep);
}

template <typename ExampleType>
template <int IndexLength>
void ExampleReservoirs<ExampleType>::add_examples(const ExampleImage_CPtr& examples, const boost::shared_ptr<ORUtils::Image<ORUtils::VectorX<int, IndexLength> > >& reservoirIndices)
{
  const boost::shared_ptr<const ORUtils::Image<ORUtils::VectorX<int, IndexLength> > > reservoirIndicesConst = reservoirIndices;
  add_examples(examples, reservoirIndicesConst);
}

template <typename ExampleType>
void ExampleReservoirs<ExampleType>::clear()
{
  m_reservoirSizes->Clear();
  m_reservoirAddCalls->Clear();
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

}
