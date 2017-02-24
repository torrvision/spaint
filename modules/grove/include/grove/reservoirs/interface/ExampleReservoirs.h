/**
 * grove: ExampleReservoirs.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLERESERVOIRS
#define H_GROVE_EXAMPLERESERVOIRS

#include <ITMLib/Utils/ITMMath.h>

#include <spaint/util/ITMImagePtrTypes.h>
#include <spaint/util/ITMMemoryBlockPtrTypes.h>

namespace grove
{

/**
 * \brief An instance of a class deriving from this one can be used to store a number of "Examples" in a set of fixed-size reservoirs.
 *
 * \param ExampleType The type of the examples stored in the reservoirs. Must have a member named "valid", convertible to boolean.
 */
template<typename ExampleType>
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

  //#################### PROTECTED MEMBER VARIABLES ####################
protected:
  /** The capacity of each reservoir. */
  uint32_t m_capacity;

  /** The actual reservoirs: an image wherein each row allows the storage of up to m_capacity examples. */
  ReservoirsImage_Ptr m_data;

  /** The number of reservoirs. */
  uint32_t m_reservoirCount;

  /** The number of times the insertion of an example has been attempted for each reservoir. Has an element for each reservoir (i.e. row in m_data). */
  ITMIntMemoryBlock_Ptr m_reservoirsAddCalls;

  /** The current size of each reservoir.  Has an element for each reservoir (i.e. row in m_data). */
  ITMIntMemoryBlock_Ptr m_reservoirsSize;

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
   * \brief Destroys an instance of the ExampleReservoirs class.
   */
  virtual ~ExampleReservoirs();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the reservoir capacity.
   *
   * \return The reservoir capacity.
   */
  uint32_t get_capacity() const;

  /**
   * \brief Returns a pointer to the reservoirs.
   *
   * \note One reservoir per row, each row has get_capacity() length but only get_reservoirs_size()[rowIdx] are valid.
   *
   * \return A constant pointer to the example reservoirs.
   */
  ReservoirsImage_CPtr get_reservoirs() const;

  /**
   * \brief Gets the number of reservoirs.
   *
   * \return The number of reservoirs.
   */
  uint32_t get_reservoirs_count() const;

  /**
   * \brief Returns the current size of each reservoir.
   *
   * \return A constant pointer to a vector storing the size of each reservoir.
   */
  ITMIntMemoryBlock_CPtr get_reservoirs_size() const;

  //#################### PUBLIC TEMPLATED MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Add examples to the reservoirs. Templated on the number of reservoirs an example will be added to.
   *
   * \note  Adding examples to a reservoir that is filled to capacity may cause older examples to be randomly discarded.
   *
   * \param examples         The examples to add to the reservoirs. Only those that have the "valid" member set to true
   *                         will be added.
   * \param reservoirIndices Indices of the reservoirs wherein to add each element of the examples image. Must have the same size as examples.
   *
   * \throws std::invalid_argument If examples and reservoirIndices have different dimensions.
   */
  template <int IndexLength>
  void add_examples(const ExampleImage_CPtr &examples,
                    const boost::shared_ptr<const ORUtils::Image<ORUtils::VectorX<int, IndexLength> > > &reservoirIndices);

  /**
   * \brief Add examples to the reservoirs. Templated on the number of reservoirs an example will be added to. Non-const variant.
   *
   * \note  Adding examples to a reservoir that is filled to capacity may cause older examples to be randomly discarded.
   *
   * \param examples         The examples to add to the reservoirs. Only those that have the "valid" member set to true
   *                         will be added.
   * \param reservoirIndices Indices of the reservoirs wherein to add each element of the examples image. Must have the same size as examples.
   *
   * \throws std::invalid_argument If examples and reservoirIndices have different dimensions.
   */
  template <int IndexLength>
  void add_examples(const ExampleImage_CPtr &examples,
                    const boost::shared_ptr<ORUtils::Image<ORUtils::VectorX<int, IndexLength> > > &reservoirIndices);

  //#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Clear the reservoirs. Discards all examples and reinitialises the random number generator.
   */
  virtual void clear();

  //#################### PROTECTED VIRTUAL MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Add examples to the reservoirs. Virtual, non-templated, method, implemented in the CPU and CUDA subclasses.
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
  virtual void add_examples(const ExampleImage_CPtr &examples, const char *reservoirIndicesCPU,
                            const char *reservoirIndicesCUDA, uint32_t reservoirIndicesCount,
                            uint32_t reservoirIndicesStep) = 0;
};

}

#endif
