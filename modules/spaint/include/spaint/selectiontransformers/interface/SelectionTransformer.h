/**
 * spaint: SelectionTransformer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_SELECTIONTRANSFORMER
#define H_SPAINT_SELECTIONTRANSFORMER

#include <ITMLib/Objects/ITMScene.h>
#include <ITMLib/Utils/ITMLibSettings.h>

#include <tvginput/InputState.h>

#include "SelectionTransformerVisitor.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to transform one selection of voxels in the scene into another.
 */
class SelectionTransformer
{
  //#################### TYPEDEFS ####################
public:
  typedef ORUtils::MemoryBlock<Vector3s> Selection;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The device on which the transformer is operating. */
  const ITMLib::ITMLibSettings::DeviceType m_deviceType;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a selection transformer.
   *
   * \param deviceType  The device on which the transformer is operating.
   */
  explicit SelectionTransformer(ITMLib::ITMLibSettings::DeviceType deviceType);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the selection transformer.
   */
  virtual ~SelectionTransformer();

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Accepts a visitor.
   *
   * \param visitor The visitor to accept.
   */
  virtual void accept(const SelectionTransformerVisitor& visitor) const = 0;

  /**
   * \brief Computes the size of the output selection of voxels corresponding to the specified input selection.
   *
   * \param inputSelectionMB  A memory block containing the input selection of voxels.
   * \return                  The size of the output selection of voxels corresponding to the specified input selection.
   */
  virtual size_t compute_output_selection_size(const Selection& inputSelectionMB) const = 0;

  /**
   * \brief Transforms one selection of voxels in the scene into another.
   *
   * \param inputSelectionMB  A memory block containing the input selection of voxels.
   * \param outputSelectionMB A memory block into which to store the output selection of voxels.
   */
  virtual void transform_selection(const Selection& inputSelectionMB, Selection& outputSelectionMB) const = 0;

  /**
   * \brief Updates the selection transformer.
   *
   * \param inputState  The current input state.
   */
  virtual void update(const tvginput::InputState& inputState) = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Transforms one selection of voxels in the scene into another.
   *
   * This function returns a raw pointer to memory allocated with new. It is the caller's responsibility
   * to ensure that this memory is eventually deleted. The easiest way to ensure this is to immediately
   * wrap the raw pointer in a shared_ptr when this function returns. This function can't use shared_ptr
   * because this header is included in a .cu file that's compiled by nvcc, and nvcc can't handle Boost.
   *
   * \param inputSelectionMB  A memory block containing the input selection of voxels.
   * \return                  A pointer to a memory block containing the output selection of voxels.
   */
  Selection *transform_selection(const Selection& inputSelectionMB) const;
};

}

#endif
