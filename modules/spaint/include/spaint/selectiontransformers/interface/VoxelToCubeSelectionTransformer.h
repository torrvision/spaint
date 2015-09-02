/**
 * spaint: VoxelToCubeSelectionTransformer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_VOXELTOCUBESELECTIONTRANSFORMER
#define H_SPAINT_VOXELTOCUBESELECTIONTRANSFORMER

#include "SelectionTransformer.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to expand a selection of individual voxels into a selection of voxel cubes around the initial voxels.
 */
class VoxelToCubeSelectionTransformer : public SelectionTransformer
{
  //#################### PROTECTED VARIABLES ####################
protected:
  /** The (Manhattan) radius (in voxels) to select around each initial voxel. */
  int m_radius;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a voxel to cube selection transformer.
   *
   * \param radius      The initial (Manhattan) radius (in voxels) to select around each initial voxel.
   * \param deviceType  The device on which the transformer is operating.
   */
  VoxelToCubeSelectionTransformer(int radius, ITMLibSettings::DeviceType deviceType);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void accept(const SelectionTransformerVisitor& visitor) const;

  /** Override */
  virtual size_t compute_output_selection_size(const Selection& inputSelectionMB) const;

  /**
   * \brief Gets the (Manhattan) radius (in voxels) to select around each initial voxel.
   *
   * \return  The (Manhattan) radius (in voxels) to select around each initial voxel.
   */
  int get_radius() const;

  /** Override */
  virtual void update(const tvginput::InputState& inputState);

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Calculates the length of each side of one of the cubes (in voxels).
   *
   * \return  The length of each side of one of the cubes (in voxels).
   */
  int cube_side_length() const;

  /**
   * \brief Calculates the number of voxels in each cube.
   *
   * \return  The number of voxels in each cube.
   */
  int cube_size() const;
};

}

#endif
