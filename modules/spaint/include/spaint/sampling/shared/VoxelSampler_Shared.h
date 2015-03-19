/**
 * spaint: VoxelSampler_Shared.h
 */

#ifndef H_SPAINT_VOXELSAMPLER_SHARED
#define H_SPAINT_VOXELSAMPLER_SHARED

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>

namespace spaint {

/**
 * \brief Sets the voxel count for the specified label.
 *
 * \param label                 The label for which to set the voxel count.
 * \param raycastResultSize     The size of the raycast result image (in pixels).
 * \param voxelMaskPrefixSums   The prefix sums for the voxel masks.
 * \param voxelCountsForLabels  An array into which to write the numbers of voxels sampled for each label.
 */
_CPU_AND_GPU_CODE_
inline void set_voxel_count(int label, int raycastResultSize, const unsigned int *voxelMaskPrefixSums, unsigned int *voxelCountsForLabels)
{
  voxelCountsForLabels[label] += voxelMaskPrefixSums[label * (raycastResultSize+1) + raycastResultSize];
}

/**
 * \brief Updates the voxel masks for the various labels based on the contents of the specified voxel (if it exists).
 *
 * \param voxelIndex        The index of the voxel whose entries in the mask should be updated.
 * \param raycastResult     The current raycast result image.
 * \param raycastResultSize The size of the raycast result image (in pixels).
 * \param voxelData         The scene's voxel data.
 * \param indexData         The scene's index data.
 * \param voxelMasks        An array into which to write the voxel masks indicating which voxels may be used as examples of which semantic labels.
 */
_CPU_AND_GPU_CODE_
inline void update_masks_for_voxel(int voxelIndex, const Vector4f *raycastResult, int raycastResultSize,
                                   const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                   int labelCount, unsigned char *voxelMasks)
{
  Vector3i loc = raycastResult[voxelIndex].toVector3().toIntRound();
  bool isFound;
  int voxelAddress = findVoxel(indexData, loc, isFound);
  const SpaintVoxel *voxel = isFound ? &voxelData[voxelAddress] : NULL;

  // Update the voxel masks for the various labels.
  for(int k = 0; k < labelCount; ++k)
  {
    voxelMasks[k * (raycastResultSize+1) + voxelIndex] = voxel && voxel->label == k ? 1 : 0;
  }
}

/**
 * \brief Attempts to write the location of the specified voxel to the segment of the voxel location array
 *        corresponding to the label (if any) for which it could serve as a sample.
 *
 * Note 1: If we already have enough samples for a particular label, we will avoid writing any more.
 * Note 2: If the voxel cannot serve as a sample for any label, nothing is written.
 *
 * \param voxelIndex  TODO
 */
_CPU_AND_GPU_CODE_
inline void write_voxel_location(int voxelIndex, const Vector4f *raycastResult, int raycastResultSize,
                                 const unsigned char *voxelMasks, const unsigned int *voxelMaskPrefixSums,
                                 int labelCount, Vector3s *voxelLocations)
{
  // For each label:
  for(int k = 0; k < labelCount; ++k)
  {
    // If the voxel we're processing is a suitable sample for this label:
    if(voxelMasks[k * (raycastResultSize+1) + voxelIndex])
    {
      // Calculate its location.
      Vector3s loc = raycastResult[voxelIndex].toVector3().toShortRound();

      // Determine the index to which it should be written in the segment of the voxel location array that corresponds to this label.
      unsigned int i = voxelMaskPrefixSums[k * (raycastResultSize+1) + voxelIndex];

      // Provided it won't cause us to exceed the maximum number of voxels for this label, write the voxel's location to the voxel location array.
      voxelLocations[k * raycastResultSize + i] = loc;
    }
  }
}

}

#endif
