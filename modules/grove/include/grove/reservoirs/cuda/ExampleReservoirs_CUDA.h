/**
 * grove: ExampleReservoirs_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLERESERVOIRSCUDA
#define H_GROVE_EXAMPLERESERVOIRSCUDA

#include "../interface/ExampleReservoirs.h"
#include "../../numbers/CUDARNG.h"

namespace grove {

/**
 * \brief An instance of this class one can be used to store a number of "Examples" in a set of fixed-size reservoirs using CUDA.
 *
 * \param ExampleType The type of the examples stored in the reservoirs. Must have a member named "valid", convertible to boolean.
 * \param IndexType   A vector-type used to select the reservoirs wherein to store each example. Must have a "size()" function returning
 *                    the number of reservoirs to associate to each example, and an "operator[]" returning one of the "size()" reservoir
 *                    indices for each call.
 */
template <typename ExampleType>
class ExampleReservoirs_CUDA : public ExampleReservoirs<ExampleType>
{
  //#################### TYPEDEFS AND USINGS ####################
public:
  typedef ExampleReservoirs<ExampleType> Base;
  using typename Base::ExampleImage_CPtr;
  using typename Base::Visitor;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an instance of the ExampleReservoirs_CUDA class.
   *
   * \param reservoirCapacity The capacity of each reservoir.
   * \param reservoirCount    The number of reservoirs to create.
   * \param rngSeed           The seed for the random number generation routines used to decide whether to add examples to reservoirs.
   */
  ExampleReservoirs_CUDA(uint32_t reservoirCapacity, uint32_t reservoirCount, uint32_t rngSeed = 42);

  //#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################
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

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** A set of random number generators used by the CUDA threads. */
  CUDARNGMemoryBlock_Ptr m_rngs;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Initialises the random number generation states with a known seed.
   */
  void init_random();

  //#################### FRIENDS ####################

  friend class ExampleReservoirs<ExampleType>;
};

}

#endif
