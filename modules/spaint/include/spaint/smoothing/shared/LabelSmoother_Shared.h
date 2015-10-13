/**
 * spaint: LabelSmoother_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_LABELSMOOTHER_SHARED
#define H_SPAINT_LABELSMOOTHER_SHARED

#include "../../markers/shared/VoxelMarker_Shared.h"

namespace spaint {

/**
 * \brief Fills in the label of the specified voxel from its neighbours if a significant number of those within range share the same label.
 *
 * \param voxelIndex                      The index of the voxel in the raycast result.
 * \param width                           The width of the raycast result.
 * \param height                          The height of the raycast result.
 * \param maxLabelCount                   The maximum number of labels that can be in use.
 * \param raycastResult                   The raycast result.
 * \param voxelData                       The scene's voxel data.
 * \param indexData                       The scene's index data.
 * \param maxSquaredDistanceBetweenVoxels The maximum squared distance allowed between the positions of the neighbour and the voxel of interest if smoothing is to occur.
 */
_CPU_AND_GPU_CODE_
inline void smooth_from_neighbours(int voxelIndex, int width, int height, int maxLabelCount, const Vector4f *raycastResult,
                                   SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                   float maxSquaredDistanceBetweenVoxels)
{
  // Note: We declare the label count array with a fixed maximum size here for simplicity.
  //       The size will need to be changed if we ever want to use more than 32 labels.
  unsigned char labelCounts[32] = {0,};

  // Look up the position and image coordinates of the target voxel.
  Vector3f loc = raycastResult[voxelIndex].toVector3();
  int x = voxelIndex % width;
  int y = voxelIndex / width;

  bool foundPoint;

  // For each neighbouring voxel:
  for(int dx = -1; dx <= 1; ++dx)
  {
    for(int dy = -1; dy <= 1; ++dy)
    {
      if(dx == 0 && dy == 0) continue;

      // Compute the image coordinates of the neighbouring voxel and check that it is within the raycast result.
      int neighbourX = x + dx, neighbourY = y + dy;
      if(neighbourX < 0 || neighbourX >= width || neighbourY < 0 || neighbourY >= height) continue;

      // Look up the position and properties of the neighbouring voxel.
      int neighbourVoxelIndex = neighbourY * width + neighbourX;
      Vector3f neighbourLoc = raycastResult[neighbourVoxelIndex].toVector3();
      const SpaintVoxel neighbourVoxel = readVoxel(voxelData, indexData, neighbourLoc.toIntRound(), foundPoint);
      if(!foundPoint) continue;

      // If the neighbouring voxel is near enough to the target voxel:
      Vector3f posOffset = neighbourLoc - loc;
      if(dot(posOffset, posOffset) <= maxSquaredDistanceBetweenVoxels)
      {
        // Increment the count for its label.
        ++labelCounts[neighbourVoxel.packedLabel.label];
      }
    }
  }

  // Calculate the neighbouring label (if any) with maximum support.
  SpaintVoxel::Label bestLabel(0);
  int bestLabelCount = 0;
  for(int i = 1; i < maxLabelCount; ++i)
  {
    if(labelCounts[i] > bestLabelCount)
    {
      bestLabel = i;
      bestLabelCount = labelCounts[i];
    }
  }

  // If there is a best label, and at least a specified number of the neigbouring voxels are labelled with it,
  // use it to update the label of the target voxel.
  const int bestLabelThreshold = 6;
  if(bestLabel != 0 && bestLabelCount >= bestLabelThreshold)
  {
    mark_voxel(loc.toShortRound(), SpaintVoxel::PackedLabel(bestLabel, SpaintVoxel::LG_PROPAGATED), NULL, voxelData, indexData);
  }
}

}

#endif
