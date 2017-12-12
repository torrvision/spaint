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
#ifdef WITH_CUDA
    virtual void visit(ExampleReservoirs_CUDA<ExampleType>& target) const = 0;
#endif
  };

private:
  template <int ReservoirIndexCount>
  struct AddExamplesCaller : Visitor
  {
    typedef ORUtils::Image<ORUtils::VectorX<int,ReservoirIndexCount> > ReservoirIndexImage;
    typedef boost::shared_ptr<const ReservoirIndexImage> ReservoirIndexImage_CPtr;

    ExampleImage_CPtr examples;
    ReservoirIndexImage_CPtr reservoirIndices;

    AddExamplesCaller(const ExampleImage_CPtr& examples_, const ReservoirIndexImage_CPtr& reservoirIndices_)
    : examples(examples_), reservoirIndices(reservoirIndices_)
    {}

    virtual void visit(ExampleReservoirs_CPU<ExampleType>& target) const  { target.add_examples_sub(examples, reservoirIndices); }
#ifdef WITH_CUDA
    virtual void visit(ExampleReservoirs_CUDA<ExampleType>& target) const { target.add_examples_sub(examples, reservoirIndices); }
#endif
  };

  //#################### PROTECTED MEMBER VARIABLES ####################
protected:
  /** The capacity (maximum size) of each reservoir. */
  uint32_t m_reservoirCapacity;

  /** The number of reservoirs. */
  uint32_t m_reservoirCount;

  /** The example reservoirs: an image in which each row allows the storage of up to m_reservoirCapacity examples. */
  ReservoirsImage_Ptr m_reservoirs;

  /** The number of times the insertion of an example has been attempted for each reservoir. Has an element for each reservoir (i.e. row in m_reservoirs). */
  ITMIntMemoryBlock_Ptr m_reservoirAddCalls;

  /** The current size of each reservoir. Has an element for each reservoir (i.e. row in m_reservoirs). */
  ITMIntMemoryBlock_Ptr m_reservoirSizes;

  /** The seed for the random number generator. */
  uint32_t m_rngSeed;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a set of example reservoirs.
   *
   * \param reservoirCount    The number of reservoirs to create.
   * \param reservoirCapacity The capacity (maximum size) of each reservoir.
   * \param rngSeed           The seed for the random number generator.
   */
  ExampleReservoirs(uint32_t reservoirCount, uint32_t reservoirCapacity, uint32_t rngSeed);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the example reservoirs.
   */
  virtual ~ExampleReservoirs();

  //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Accepts a visitor.
   *
   * \param visitor The visitor to accept.
   */
  virtual void accept(const Visitor& visitor) = 0;

  //#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Loads the reservoir state from a folder on disk.
   *
   * \param inputFolder  The folder containing the reservoir state.
   *
   * \throws std::runtime_error  If the loading failed.
   */
  virtual void load_from_disk(const std::string& inputFolder);

  /**
   * \brief Saves the reservoir state to a folder on disk.
   *
   * \param outputFolder  The folder wherein to save the reservoir state.
   *
   * \throws std::runtime_error  If the saving failed.
   */
  virtual void save_to_disk(const std::string& outputFolder);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds some examples to the reservoirs.
   *
   * \note Adding examples to a reservoir that is filled to capacity may cause older examples to be randomly discarded.
   *
   * \tparam ReservoirIndexCount  The number of reservoirs to which an example will be added.
   *
   * \param examples         The examples to add to the reservoirs. Only those that have the "valid" member set to true will be added.
   * \param reservoirIndices The indices of the reservoirs to which to add each element of the examples image. Must have the same size as examples.
   *
   * \throws std::invalid_argument If examples and reservoirIndices have different dimensions.
   */
  template <int ReservoirIndexCount>
  void add_examples(const ExampleImage_CPtr& examples, const boost::shared_ptr<const ORUtils::Image<ORUtils::VectorX<int,ReservoirIndexCount> > >& reservoirIndices);

  /**
   * \brief Adds examples to the reservoirs.
   *
   * \note This is a variant of the other add_examples function that allows us to pass in non-const images.
   */
  template <int ReservoirIndexCount>
  void add_examples(const ExampleImage_CPtr& examples, const boost::shared_ptr<ORUtils::Image<ORUtils::VectorX<int,ReservoirIndexCount> > >& reservoirIndices);

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
   * \brief Gets the current size of each example reservoir.
   *
   * \return A memory block containing the current size of each example reservoir.
   */
  ITMIntMemoryBlock_CPtr get_reservoir_sizes() const;

  /**
   * \brief Clears the reservoirs, discards all examples and reinitialises the random number generator.
   */
  virtual void reset();
};

}

#endif
