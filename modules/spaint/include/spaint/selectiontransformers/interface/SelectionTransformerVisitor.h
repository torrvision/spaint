/**
 * spaint: SelectionTransformerVisitor.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_SELECTIONTRANSFORMERVISITOR
#define H_SPAINT_SELECTIONTRANSFORMERVISITOR

namespace spaint {

//#################### FORWARD DECLARATIONS ####################

class VoxelToCubeSelectionTransformer;

/**
 * \brief An instance of a class deriving from this one can be used to visit selection transformers.
 */
class SelectionTransformerVisitor
{
  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the visitor.
   */
  virtual ~SelectionTransformerVisitor() = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Visits a voxel to cube selection transformer.
   *
   * \param transformer The selection transformer.
   */
  virtual void visit(const VoxelToCubeSelectionTransformer& transformer) const = 0;
};

}

#endif
