/**
 * grove: ExampleReservoirs.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLERESERVOIRS
#define H_GROVE_EXAMPLERESERVOIRS

#include <itmx/base/ITMImagePtrTypes.h>
#include <itmx/base/ITMMemoryBlockPtrTypes.h>

namespace grove {

//#################### FORWARD DECLARATIONS ####################

template <typename ExampleType> class ExampleReservoirs_CPU;
template <typename ExampleType> class ExampleReservoirs_CUDA;

/**
 * \brief An instance of a class deriving from this one can be used to store a number of examples in a set of fixed-size reservoirs.
 *
 * \tparam ExampleType  The type of example stored in the reservoirs. Must have a member named "valid", convertible to bool.
 */
template <typename ExampleType>
class ExampleReservoirs
{
  //#################### TYPEDEFS ####################
public:
  typedef ORUtils::Image<ExampleType> ExampleImage;
  typedef boost::shared_ptr<ExampleImage> ExampleImage_Ptr;
  typedef boost::shared_ptr<const ExampleImage> ExampleImage_CPtr;

  typedef ORUtils::Image<ExampleType> ReservoirsImage;
  typedef boost::shared_ptr<ReservoirsImage> ReservoirsImage_Ptr;
  typedef boost::shared_ptr<const ReservoirsImage> ReservoirsImage_CPtr;

  //#################### NESTED TYPES ####################
protected:
  struct Visitor
  {
    virtual void visit(ExampleReservoirs_CPU<ExampleType>& target) const = 0;
    virtual void visit(ExampleReservoirs_CUDA<ExampleType>& target) const = 0;
  };

  template <int IndexLength>
  struct AddExamplesVisitor : Visitor
  {
    ExampleImage_CPtr examples;
    boost::shared_ptr<const ORUtils::Image<ORUtils::VectorX<int,IndexLength> > > reservoirIndices;

    AddExamplesVisitor(const ExampleImage_CPtr& examples_, const boost::shared_ptr<const ORUtils::Image<ORUtils::VectorX<int,IndexLength> > >& reservoirIndices_)
    : examples(examples_), reservoirIndices(reservoirIndices_)
    {}

    virtual void visit(ExampleReservoirs_CPU<ExampleType>& target) const
    {
      target.add_examples(examples, reservoirIndices);
    }

    virtual void visit(ExampleReservoirs_CUDA<ExampleType>& target) const
    {
      //target.add_examples(examples, reservoirIndices);
    }
  };

  //#################### PROTECTED MEMBER VARIABLES ####################
protected:
  /** The capacity of each reservoir. */
  uint32_t m_reservoirCapacity;

  /** The number of reservoirs. */
  uint32_t m_reservoirCount;

  /** The example reservoirs: an image in which each row allows the storage of up to m_reservoirCapacity examples. */
  ReservoirsImage_Ptr m_reservoirs;

  /** The number of times the insertion of an example has been attempted for each reservoir. Has an element for each reservoir (i.e. row in m_reservoirs). */
  ITMIntMemoryBlock_Ptr m_reservoirAddCalls;

  /** The current size of each reservoir. Has an element for each reservoir (i.e. row in m_reservoirs). */
  ITMIntMemoryBlock_Ptr m_reservoirSizes;

  /** The seed for the random number generation. */
  uint32_t m_rngSeed;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs an instance of the ExampleReservoirs class.
   *
   * \param reservoirCapacity The capacity of each reservoir.
   * \param reservoirCount    The number of reservoirs to create.
   * \param rngSeed           The seed for the random number generation routines used to decide whether to add examples to reservoirs.
   */
  ExampleReservoirs(uint32_t reservoirCapacity, uint32_t reservoirCount, uint32_t rngSeed = 42);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the example reservoirs.
   */
  virtual ~ExampleReservoirs();

  //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
private:
  virtual void accept(Visitor& visitor) = 0;

  /**
   * \brief Add examples to the reservoirs. Virtual, non-templated, method, implemented in the CPU and CUDA subclasses.
   *
   * \note  This method will be called by the templated add_examples method, the reservoirIndicesCPU and reservoirIndicesCUDA
   *        parameters will point to the beginning of the memory associated to the "index image".
   *        Access to the reservoirIndicesCount indices is strided according to reservoirIndicesStep.
   *
   * \param examples              The examples to add to the reservoirs. Only those that have the "valid" member set to true will be added.
   * \param reservoirIndicesCPU   Raw pointer to the CPU memory associated to the indices.
   * \param reservoirIndicesCUDA  Raw pointer to the CUDA memory associated to the indices.
   * \param reservoirIndicesCount Number of integer indices for each element of the indices image.
   * \param reservoirIndicesStep  Step between the beginning of neighboring elements in the indices image.
   */
  virtual void add_examples(const ExampleImage_CPtr& examples, const char *reservoirIndicesCPU,
                            const char *reservoirIndicesCUDA, uint32_t reservoirIndicesCount,
                            uint32_t reservoirIndicesStep) = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Add examples to the reservoirs. Templated on the number of reservoirs an example will be added to.
   *
   * \note  Adding examples to a reservoir that is filled to capacity may cause older examples to be randomly discarded.
   *
   * \param examples         The examples to add to the reservoirs. Only those that have the "valid" member set to true will be added.
   * \param reservoirIndices Indices of the reservoirs wherein to add each element of the examples image. Must have the same size as examples.
   *
   * \throws std::invalid_argument If examples and reservoirIndices have different dimensions.
   */
  template <int IndexLength>
  void add_examples(const ExampleImage_CPtr& examples, const boost::shared_ptr<const ORUtils::Image<ORUtils::VectorX<int, IndexLength> > >& reservoirIndices);

  /**
   * \brief Add examples to the reservoirs. Templated on the number of reservoirs an example will be added to. Non-const variant.
   *
   * \note  Adding examples to a reservoir that is filled to capacity may cause older examples to be randomly discarded.
   *
   * \param examples         The examples to add to the reservoirs. Only those that have the "valid" member set to true will be added.
   * \param reservoirIndices Indices of the reservoirs wherein to add each element of the examples image. Must have the same size as examples.
   *
   * \throws std::invalid_argument If examples and reservoirIndices have different dimensions.
   */
  template <int IndexLength>
  void add_examples(const ExampleImage_CPtr& examples, const boost::shared_ptr<ORUtils::Image<ORUtils::VectorX<int, IndexLength> > >& reservoirIndices);

  /**
   * \brief Clears the reservoirs, discards all examples and reinitialises the random number generator.
   */
  virtual void clear();

  /**
   * \brief Gets the capacity of each reservoir.
   *
   * \note All the reservoirs have the same capacity.
   *
   * \return The capacity of each reservoir.
   */
  uint32_t get_reservoir_capacity() const;

  /**
   * \brief Gets the example reservoirs.
   *
   * \note These are stored in an image in which each row corresponds to a reservoir. Each row can store up to
   *       get_reservoir_capacity() examples, but only the first get_reservoir_sizes()[rowIdx] are valid.
   *
   * \return The example reservoirs.
   */
  ReservoirsImage_CPtr get_reservoirs() const;

  /**
   * \brief Gets the number of example reservoirs.
   *
   * \return The number of example reservoirs.
   */
  uint32_t get_reservoir_count() const;

  /**
   * \brief Gets the current size of each reservoir.
   *
   * \return A memory block containing the current size of each reservoir.
   */
  ITMIntMemoryBlock_CPtr get_reservoir_sizes() const;
};

}

#endif
