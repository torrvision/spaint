/**
 * grove: ExampleReservoirs_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLERESERVOIRS_CUDA
#define H_GROVE_EXAMPLERESERVOIRS_CUDA

#include "../interface/ExampleReservoirs.h"
#include "../../numbers/CUDARNG.h"

namespace grove {

/**
 * \brief An instance of this class can be used to store a number of examples in a set of fixed-size reservoirs using CUDA.
 *
 * \param ExampleType The type of example stored in the reservoirs. Must have a member named "valid", convertible to bool.
 */
template <typename ExampleType>
class ExampleReservoirs_CUDA : public ExampleReservoirs<ExampleType>
{
  //#################### TYPEDEFS AND USINGS ####################
public:
  typedef ExampleReservoirs<ExampleType> Base;
  using typename Base::ExampleImage_CPtr;
  using typename Base::Visitor;

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** A set of random number generators (used when adding examples). */
  CUDARNGMemoryBlock_Ptr m_rngs;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a set of example reservoirs.
   *
   * \param reservoirCount    The number of reservoirs to create.
   * \param reservoirCapacity The capacity of each reservoir.
   * \param rngSeed           The seed for the random number generator.
   */
  ExampleReservoirs_CUDA(uint32_t reservoirCount, uint32_t reservoirCapacity, uint32_t rngSeed = 42);

  //#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################
public:
  /** Override. */
  virtual void load_from_disk(const std::string& inputFolder);

  /** Override */
  virtual void reset();

  /** Override. */
  virtual void save_to_disk(const std::string& outputFolder);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void accept(const Visitor& visitor);

  /** Derived Implementation */
  template <int ReservoirIndexCount>
  void add_examples_sub(const ExampleImage_CPtr& examples, const boost::shared_ptr<const ORUtils::Image<ORUtils::VectorX<int,ReservoirIndexCount> > >& reservoirIndices);

  /**
   * \brief Reinitialises the random number generators using known seeds.
   */
  void reinit_rngs();

  //#################### FRIENDS ####################

  friend class ExampleReservoirs<ExampleType>;
};

}

#endif
