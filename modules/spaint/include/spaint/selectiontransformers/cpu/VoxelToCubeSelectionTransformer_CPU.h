/**
 * spaint: VoxelToCubeSelectionTransformer_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_VOXELTOCUBESELECTIONTRANSFORMER_CPU
#define H_SPAINT_VOXELTOCUBESELECTIONTRANSFORMER_CPU

#include "../interface/VoxelToCubeSelectionTransformer.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to expand a selection of individual voxels into a selection of voxel cubes around the initial voxels using the CPU.
 */
class VoxelToCubeSelectionTransformer_CPU : public VoxelToCubeSelectionTransformer
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a voxel to cube selection transformer that uses the CPU.
   *
   * \param radius  The (Manhattan) radius (in voxels) to select around each initial voxel.
   */
  explicit VoxelToCubeSelectionTransformer_CPU(int radius);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void transform_selection(const Selection& inputSelectionMB, Selection& outputSelectionMB) const;
};

}

#endif
