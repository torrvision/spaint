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
  : m_radius(radius)
  {}
};

}

#endif
