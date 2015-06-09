/**
 * spaint: VoxelSampler_Shared.h
 */

#ifndef H_SPAINT_VOXELSAMPLER_SHARED
#define H_SPAINT_VOXELSAMPLER_SHARED

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>

namespace spaint {

/**
 * \brief Copies the location of a randomly-chosen candidate voxel for each label across to the sampled voxel locations array.
 *
 * Note 1: The voxel locations to sample are chosen in advance and passed in.
 * Note 2: The numbers of voxel locations sampled for the various labels can differ (e.g. if there are not enough candidates for a given label).
 *         A candidate voxel index of -1 indicates that no voxel location has been sampled for a given label and voxel index.
 *
 * \param voxelIndex              The index of the voxel currently being processed for each label (each thread processes one voxel per label).
 * \param labelMask               A mask indicating which labels are currently in use.
 * \param maxLabelCount           The maximum number of labels that can be in use.
 * \param maxVoxelsPerLabel       The maximum number of voxels to sample for each label.
 * \param raycastResultSize       The size of the raycast result (in pixels).
 * \param candidateVoxelLocations An array containing the locations of the candidate voxels (grouped by label).
 * \param candidateVoxelIndices   An array specifying which candidate voxels should be sampled for each label.
 * \param sampledVoxelLocations   An array into which to write the locations of the sampled voxels.
 */
_CPU_AND_GPU_CODE_
inline void copy_sampled_voxel_locations(int voxelIndex, const bool *labelMask, size_t maxLabelCount, size_t maxVoxelsPerLabel, int raycastResultSize,
                                         const Vector3s *candidateVoxelLocations, const int *candidateVoxelIndices, Vector3s *sampledVoxelLocations)
{
  for(size_t k = 0; k < maxLabelCount; ++k)
  {
    if(labelMask[k])
    {
      int candidateVoxelIndex = candidateVoxelIndices[k * maxVoxelsPerLabel + voxelIndex];
      if(candidateVoxelIndex != -1)
      {
        sampledVoxelLocations[k * maxVoxelsPerLabel + voxelIndex] = candidateVoxelLocations[k * raycastResultSize + candidateVoxelIndex];
      }
    }
  }
}

/**
 * \brief Updates the voxel masks for the various labels based on the contents of the specified voxel (if it exists).
 *
 * \param voxelIndex        The index of the voxel whose entries in the mask should be updated.
 * \param raycastResult     The current raycast result.
 * \param raycastResultSize The size of the raycast result (in pixels).
 * \param voxelData         The scene's voxel data.
 * \param indexData         The scene's index data.
 * \param maxLabelCount     The maximum number of labels that can be in use.
 * \param voxelMasks        An array into which to write the voxel masks indicating which voxels may be used as examples of which labels.
 */
_CPU_AND_GPU_CODE_
inline void update_masks_for_voxel(int voxelIndex, const Vector4f *raycastResult, int raycastResultSize,
                                   const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                   size_t maxLabelCount, unsigned char *voxelMasks)
{
  // Note: We do not need to explicitly use the label mask in this function, since no voxel will ever be marked with an unused label.

  Vector3i loc = raycastResult[voxelIndex].toVector3().toIntRound();
  bool isFound;
  int voxelAddress = findVoxel(indexData, loc, isFound);
  const SpaintVoxel *voxel = isFound ? &voxelData[voxelAddress] : NULL;

  // Update the voxel masks for the various labels (even the ones that are not currently active).
  for(size_t k = 0; k < maxLabelCount; ++k)
  {
    voxelMasks[k * (raycastResultSize + 1) + voxelIndex] = voxel && voxel->label == k ? 1 : 0;
  }
}

/**
 * \brief Writes the number of candidate voxels that are available for the specified label into the voxel counts array.
 *
 * \param label                 The label for which to store the number of available candidate voxels.
 * \param raycastResultSize     The size of the raycast result (in pixels).
 * \param voxelMaskPrefixSums   The prefix sums for the voxel masks.
 * \param labelMask             A mask indicating which labels are currently in use.
 * \param voxelCountsForLabels  An array into which to write the numbers of candidate voxels that are available for each label.
 */
_CPU_AND_GPU_CODE_
inline void write_candidate_voxel_count(int label, int raycastResultSize, const bool *labelMask, const unsigned int *voxelMaskPrefixSums,
                                        unsigned int *voxelCountsForLabels)
{
  voxelCountsForLabels[label] = labelMask[label] ? voxelMaskPrefixSums[label * (raycastResultSize+1) + raycastResultSize] : 0;
}

/**
 * \brief Writes the location of the specified candidate voxel to the segment of the candidate voxel
 *        locations array corresponding to the label (if any) for which it is a candidate sample.
 *
 * Note: If the voxel cannot serve as a sample for any label, nothing is written.
 *
 * \param voxelIndex              The index of the voxel whose location might be written.
 * \param raycastResult           The current raycast result.
 * \param raycastResultSize       The size of the raycast result (in pixels).
 * \param voxelMasks              An array containing the voxel masks indicating which voxels may be used as examples of which labels.
 * \param voxelMaskPrefixSums     The prefix sums for the voxel masks.
 * \param maxLabelCount           The maximum number of labels that can be in use.
 * \param candidateVoxelLocations An array into which to write the locations of the candidate voxels.
 */
_CPU_AND_GPU_CODE_
inline void write_candidate_voxel_location(int voxelIndex, const Vector4f *raycastResult, int raycastResultSize,
                                           const unsigned char *voxelMasks, const unsigned int *voxelMaskPrefixSums,
                                           size_t maxLabelCount, Vector3s *candidateVoxelLocations)
{
  // Note: We do not need to explicitly use the label mask in this function, since there will never be any candidates for unused labels.

  // For each possible label:
  for(size_t k = 0; k < maxLabelCount; ++k)
  {
    const int maskOffset = k * (raycastResultSize + 1) + voxelIndex;

    // If the voxel we're processing is a candidate for this label:
    if(voxelMasks[maskOffset])
    {
      // Calculate its location.
      Vector3s loc = raycastResult[voxelIndex].toVector3().toShortRound();

      // Determine the index to which it should be written in the segment of the candidate voxel location array that corresponds to this label.
      unsigned int i = voxelMaskPrefixSums[maskOffset];

      // Write the candidate voxel's location to the candidate voxel locations array.
      candidateVoxelLocations[k * raycastResultSize + i] = loc;
    }
  }
}

}

#endif
