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
 * \brief An instance of this class one can be used to store a number of "Examples" in a set of fixed-size reservoirs using the CPU.
 *
 * \param ExampleType The type of the examples stored in the reservoirs. Must have a member named "valid", convertible to boolean.
 * \param IndexType   A vector-type used to select the reservoirs wherein to store each example. Must have a "size()" function returning
 *                    the number of reservoirs to associate to each example, and an "operator[]" returning one of the "size()" reservoir
 *                    indices for each call.
 */
template <typename ExampleType>
class ExampleReservoirs_CPU : public ExampleReservoirs<ExampleType>
{
  //#################### TYPEDEFS ####################
public:
  using typename ExampleReservoirs<ExampleType>::ExampleImage_CPtr;

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
   * \brief Clear the reservoirs. Discards all examples and reinitialises the random number generator.
   */
  virtual void clear();

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  virtual void accept(Visitor& visitor)
  {
    visitor.visit(*this);
  }

  /**
   * \brief Add examples to the reservoirs.
   *
   * \note  This method will be called by the templated add_examples method, the reservoirIndicesCPU and reservoirIndicesCUDA
   *        parameters will point to the beginning of the memory associated to the "index image".
   *        Access to the reservoirIndicesCount indices is strided according to reservoirIndicesStep.
   *
   * \param examples              The examples to add to the reservoirs. Only those that have the "valid" member set to true
   *                              will be added.
   * \param reservoirIndicesCPU   Raw pointer to the CPU memory associated to the indices.
   * \param reservoirIndicesCUDA  Raw pointer to the CUDA memory associated to the indices.
   * \param reservoirIndicesCount Number of integer indices for each element of the indices image.
   * \param reservoirIndicesStep  Step between the beginning of neighboring elements in the indices image.
   */
  virtual void add_examples(const ExampleImage_CPtr& examples, const char *reservoirIndicesCPU,
                            const char *reservoirIndicesCUDA, uint32_t reservoirIndicesCount,
                            uint32_t reservoirIndicesStep);

  template <int IndexLength>
  void add_examples(const ExampleImage_CPtr& examples, const ORUtils::VectorX<int,IndexLength> *reservoirIndicesCPU,
                    const ORUtils::VectorX<int,IndexLength> *reservoirIndicesCUDA)
  {
    // TODO
  }

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** A set of random number generators used during the add operation. */
  CPURNGMemoryBlock_Ptr m_rngs;

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
