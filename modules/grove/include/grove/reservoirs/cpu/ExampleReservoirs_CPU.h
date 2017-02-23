/**
 * grove: ExampleReservoirs_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLERESERVOIRSCPU
#define H_GROVE_EXAMPLERESERVOIRSCPU

#include "../interface/ExampleReservoirs.h"

#include <tvgutil/numbers/SimpleRandomNumberGenerator_CPU.h>

namespace grove {

/**
 * \brief An instance of this class one can be used to store a number of "Examples" in a set of fixed-size reservoirs using the CPU.
 *
 * \param ExampleType The type of the examples stored in the reservoirs. Must have a member named "valid", convertible to boolean.
 * \param IndexType   A vector-type used to select the reservoirs wherein to store each example. Must have a "size()" function returning
 *                    the number of reservoirs to associate to each example, and an "operator[]" returning one of the "size()" reservoir
 *                    indices for each call.
 */
template <typename ExampleType, typename IndexType>
class ExampleReservoirs_CPU : public ExampleReservoirs<ExampleType, IndexType>
{
  //#################### TYPEDEFS ####################
public:
  using typename ExampleReservoirs<ExampleType, IndexType>::ExampleImage_CPtr;
  using typename ExampleReservoirs<ExampleType, IndexType>::IndexImage_CPtr;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an instance of the ExampleReservoirs_CPU class.
   *
   * \param reservoirCapacity The capacity of each reservoir.
   * \param reservoirCount    The number of reservoirs to create.
   * \param rngSeed           The seed for the random number generation routines used to decide whether to add examples to reservoirs.
   */
  ExampleReservoirs_CPU(uint32_t reservoirCapacity, uint32_t reservoirCount, uint32_t rngSeed = 42);

  //#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Add examples to the reservoirs.
   *
   * \note  Adding examples to a reservoir that is filled to capacity may cause older examples to be randomly discarded.
   *
   * \param examples         The examples to add to the reservoirs. Only those that have the "valid" member set to true
   *                         will be added.
   * \param reservoirIndices Indices of the reservoirs wherein to add each element of the examples image. Must have the same size as examples.
   *
   * \throws std::invalid_argument If examples and reservoirIndices have different dimensions.
   */
  virtual void add_examples(const ExampleImage_CPtr &examples, const IndexImage_CPtr &reservoirIndices);

  /**
   * \brief Clear the reservoirs. Discards all examples and reinitialises the random number generator.
   */
  virtual void clear();

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** A set of random number generators used during the add operation. */
  tvgutil::CPURNGMemoryBlock_Ptr m_randomStates;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Initialises the random number generation states with a known seed.
   */
  void init_random();
};

}

#endif
