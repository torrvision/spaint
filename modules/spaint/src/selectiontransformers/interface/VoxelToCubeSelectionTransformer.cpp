/**
 * spaint: VoxelToCubeSelectionTransformer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "selectiontransformers/interface/VoxelToCubeSelectionTransformer.h"
using namespace ITMLib;
using namespace tvginput;

namespace spaint {

//#################### CONSTRUCTORS ####################

VoxelToCubeSelectionTransformer::VoxelToCubeSelectionTransformer(int radius, ITMLibSettings::DeviceType deviceType)
: SelectionTransformer(deviceType),
  m_radius(radius)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VoxelToCubeSelectionTransformer::accept(const SelectionTransformerVisitor& visitor) const
{
  visitor.visit(*this);
}

size_t VoxelToCubeSelectionTransformer::compute_output_selection_size(const Selection& inputSelectionMB) const
{
  // We create one cube for each initial voxel.
  return inputSelectionMB.dataSize * cube_size();
}

int VoxelToCubeSelectionTransformer::get_radius() const
{
  return m_radius;
}

void VoxelToCubeSelectionTransformer::update(const InputState& inputState)
{
  // Allow the user to change the selection radius.
  const int minRadius = 1;
  const int maxRadius = 10;
  static bool canChange = true;

  if(!inputState.key_down(KEYCODE_RSHIFT) && inputState.key_down(KEYCODE_LEFTBRACKET))
  {
    if(canChange && m_radius > minRadius) --m_radius;
    canChange = false;
  }
  else if(!inputState.key_down(KEYCODE_RSHIFT) && inputState.key_down(KEYCODE_RIGHTBRACKET))
  {
    if(canChange && m_radius < maxRadius) ++m_radius;
    canChange = false;
  }
  else canChange = true;
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

int VoxelToCubeSelectionTransformer::cube_side_length() const
{
  return 2 * m_radius + 1;
}

int VoxelToCubeSelectionTransformer::cube_size() const
{
  int cubeSideLength = cube_side_length();
  return cubeSideLength * cubeSideLength * cubeSideLength;
}

}
