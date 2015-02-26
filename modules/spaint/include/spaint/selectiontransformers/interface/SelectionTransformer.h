/**
 * spaint: SelectionTransformer.h
 */

#ifndef H_SPAINT_SELECTIONTRANSFORMER
#define H_SPAINT_SELECTIONTRANSFORMER

#include <ITMLib/Objects/ITMScene.h>

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to transform one selection of voxels in the scene into another.
 */
class SelectionTransformer
{
  //#################### TYPEDEFS ####################
public:
  typedef ORUtils::MemoryBlock<Vector3s> Selection;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the selection transformer.
   */
  virtual ~SelectionTransformer() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Computes the size of the output selection of voxels corresponding to the specified input selection.
   *
   * \param inputSelectionMB  A memory block containing the input selection of voxels.
   * \return                  The size of the output selection of voxels corresponding to the specified input selection.
   */
  virtual int compute_output_selection_size(const Selection& inputSelectionMB) const = 0;

  /**
   * \brief Transforms one selection of voxels in the scene into another.
   *
   * \param inputSelectionMB  A memory block containing the input selection of voxels.
   * \param outputSelectionMB A memory block into which to store the output selection of voxels.
   */
  virtual void transform_selection(const Selection& inputSelectionMB, Selection& outputSelectionMB) const = 0;
};

}

#endif
