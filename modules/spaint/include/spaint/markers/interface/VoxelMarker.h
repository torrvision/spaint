/**
 * spaint: VoxelMarker.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_VOXELMARKER
#define H_SPAINT_VOXELMARKER

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/Scene/ITMScene.h>

#include "../shared/VoxelMarker_Settings.h"
#include "../../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to mark a set of voxels with a semantic label.
 */
class VoxelMarker
{
  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the voxel marker.
   */
  virtual ~VoxelMarker() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Clears the labels of some or all of the voxels in an array, depending on the settings specified.
   *
   * \param voxels      The array of voxels.
   * \param voxelCount  The number of voxels in the array.
   * \param settings    The settings to use for the label-clearing operation.
   */
  virtual void clear_labels(SpaintVoxel *voxels, int voxelCount, ClearingSettings settings) const = 0;

  /**
   * \brief Marks a set of voxels in the scene with the specified semantic label.
   *
   * \param voxelLocationsMB  A memory block containing the locations of the voxels in the scene.
   * \param label             The semantic label with which to mark the voxels.
   * \param scene             The scene.
   * \param mode              The marking mode.
   * \param oldVoxelLabelsMB  An optional memory block into which to store the old semantic labels of the voxels being marked.
   */
  virtual void mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, SpaintVoxel::PackedLabel label,
                           ITMLib::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene, MarkingMode mode,
                           ORUtils::MemoryBlock<SpaintVoxel::PackedLabel> *oldVoxelLabelsMB = NULL) const = 0;

  /**
   * \brief Marks a set of voxels in the scene with the specified semantic labels.
   *
   * \param voxelLocationsMB  A memory block containing the locations of the voxels in the scene.
   * \param voxelLabelsMB     A memory block containing the semantic labels with which to mark the voxels (one per voxel).
   * \param scene             The scene.
   * \param mode              The marking mode.
   * \param oldVoxelLabelsMB  An optional memory block into which to store the old semantic labels of the voxels being marked.
   */
  virtual void mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                           const ORUtils::MemoryBlock<SpaintVoxel::PackedLabel>& voxelLabelsMB,
                           ITMLib::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene, MarkingMode mode,
                           ORUtils::MemoryBlock<SpaintVoxel::PackedLabel> *oldVoxelLabelsMB = NULL) const = 0;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<const VoxelMarker> VoxelMarker_CPtr;

}

#endif
