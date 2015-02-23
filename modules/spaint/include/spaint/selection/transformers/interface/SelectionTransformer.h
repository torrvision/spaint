/**
 * spaint: SelectionTransformer.h
 */

#ifndef H_SPAINT_SELECTIONTRANSFORMER
#define H_SPAINT_SELECTIONTRANSFORMER

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/ITMScene.h>

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to transform one selection of voxels in the scene into another.
 */
class SelectionTransformer
{
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
  virtual int compute_output_selection_size(const ORUtils::MemoryBlock<Vector3s>& inputSelectionMB) const = 0;

  /**
   * \brief Transforms one selection of voxels in the scene into another.
   *
   * \param inputSelectionMB  A memory block containing the input selection of voxels.
   * \param outputSelectionMB A memory block into which to store the output selection of voxels.
   */
  virtual void transform_selection(const ORUtils::MemoryBlock<Vector3s>& inputSelectionMB, ORUtils::MemoryBlock<Vector3s>& outputSelectionMB) const = 0;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<const SelectionTransformer> SelectionTransformer_CPtr;

}

#endif
