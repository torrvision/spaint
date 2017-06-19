/**
 * grove: ExampleReservoirs_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLERESERVOIRSCPU
#define H_GROVE_EXAMPLERESERVOIRSCPU

#include "../interface/ExampleReservoirs.h"
#include "../../numbers/CPURNG.h"

namespace grove {

/**
 * \brief An instance of this class can be used to store a number of examples in a set of fixed-size reservoirs using the CPU.
 *
 * \param ExampleType The type of example stored in the reservoirs. Must have a member named "valid", convertible to bool.
 */
template <typename ExampleType>
class ExampleReservoirs_CPU : public ExampleReservoirs<ExampleType>
{
  //#################### TYPEDEFS AND USINGS ####################
public:
  typedef ExampleReservoirs<ExampleType> Base;
  using typename Base::ExampleImage_CPtr;
  using typename Base::Visitor;

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** A set of random number generators (used when adding examples). */
  CPURNGMemoryBlock_Ptr m_rngs;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a set of example reservoirs.
   *
   * \param reservoirCapacity The capacity of each reservoir.
   * \param reservoirCount    The number of reservoirs to create.
   * \param rngSeed           The seed for the random number generator.
   */
  ExampleReservoirs_CPU(uint32_t reservoirCapacity, uint32_t reservoirCount, uint32_t rngSeed = 42);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void reset();

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void accept(const Visitor& visitor)
  {
    visitor.visit(*this);
  }

  /** Derived Implementation */
  template <int ReservoirIndexCount>
  void add_examples_sub(const ExampleImage_CPtr& examples, const boost::shared_ptr<const ORUtils::Image<ORUtils::VectorX<int,ReservoirIndexCount> > >& reservoirIndices);

  /**
   * \brief Re-initialises the random number generators with a known seed.
   */
  void reinit_rngs();

  //#################### FRIENDS ####################

  friend class ExampleReservoirs<ExampleType>;
};

}

#endif
