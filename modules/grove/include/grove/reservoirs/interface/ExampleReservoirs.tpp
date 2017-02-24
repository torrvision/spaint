/**
 * grove: ExampleReservoirs.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ExampleReservoirs.h"

#include <spaint/util/MemoryBlockFactory.h>
using spaint::MemoryBlockFactory;

namespace grove {

//#################### CONSTRUCTORS ####################

template <typename ExampleType>
ExampleReservoirs<ExampleType>::ExampleReservoirs(
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

template <typename ExampleType>
ExampleReservoirs<ExampleType>::~ExampleReservoirs()
{
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename ExampleType>
uint32_t ExampleReservoirs<ExampleType>::get_capacity() const
{
  return m_capacity;
}

template <typename ExampleType>
typename ExampleReservoirs<ExampleType>::ExampleImage_CPtr ExampleReservoirs<ExampleType>::get_reservoirs() const
{
  return m_data;
}

template <typename ExampleType>
uint32_t ExampleReservoirs<ExampleType>::get_reservoirs_count() const
{
  return m_reservoirCount;
}

template <typename ExampleType>
ITMIntMemoryBlock_CPtr ExampleReservoirs<ExampleType>::get_reservoirs_size() const
{
  return m_reservoirsSize;
}

template <typename ExampleType>
template <int IndexLength>
void ExampleReservoirs<ExampleType>::add_examples(const ExampleImage_CPtr &examples,
    const boost::shared_ptr<const ORUtils::Image<ORUtils::VectorX<int, IndexLength> > > &reservoirIndices)
{
  // Check preconditions.
  if(examples->noDims != reservoirIndices->noDims)
    throw std::invalid_argument("The example and indices images should have the same size.");

  // Compute the step between elements of the reservoirIndices image.
  const uint32_t indexStep = sizeof(ORUtils::VectorX<int, IndexLength>);

  // Extract raw memory pointers.
  const char* reservoirIndicesCPU = reinterpret_cast<const char*>(reservoirIndices->GetData(MEMORYDEVICE_CPU));
  const char* reservoirIndicesCUDA = reinterpret_cast<const char*>(reservoirIndices->GetData(MEMORYDEVICE_CUDA));

  // Call the non-templated virtual function.
  add_examples(examples, reservoirIndicesCPU, reservoirIndicesCUDA, IndexLength, indexStep);
}

template <typename ExampleType>
template <int IndexLength>
void ExampleReservoirs<ExampleType>::add_examples(const ExampleImage_CPtr &examples,
    const boost::shared_ptr<ORUtils::Image<ORUtils::VectorX<int, IndexLength> > > &reservoirIndices)
{
  const boost::shared_ptr<const ORUtils::Image<ORUtils::VectorX<int, IndexLength> > > reservoirIndicesConst = reservoirIndices;
  add_examples(examples, reservoirIndicesConst);
}

//#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################

template <typename ExampleType>
void ExampleReservoirs<ExampleType>::clear()
{
  m_reservoirsSize->Clear();
  m_reservoirsAddCalls->Clear();
}

}
