/**
 * spaint: VoxelToCubeSelectionTransformer.h
 */

#ifndef H_SPAINT_VOXELTOCUBESELECTIONTRANSFORMER
#define H_SPAINT_VOXELTOCUBESELECTIONTRANSFORMER

#include "SelectionTransformer.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to expand a selection of individual voxels to a selection of cubes around the initial voxels.
 */
class VoxelToCubeSelectionTransformer : public SelectionTransformer
{
  //#################### PROTECTED VARIABLES ####################
protected:
  /** TODO */
  const int m_cubeSideLength;

  /** TODO */
  const int m_cubeSize;

  /** The (Manhattan) radius to select around each initial voxel. */
  const int m_radius;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a voxel to cube selection transformer.
   *
   * \param radius  The (Manhattan) radius to select around each initial voxel.
   */
  explicit VoxelToCubeSelectionTransformer(int radius)
  : m_cubeSideLength(2 * radius + 1),
    m_cubeSize(m_cubeSideLength * m_cubeSideLength * m_cubeSideLength),
    m_radius(radius)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual int compute_output_selection_size(const ORUtils::MemoryBlock<Vector3s>& inputSelectionMB) const
  {
    return inputSelectionMB.dataSize * m_cubeSize;
  }
};

}

#endif
