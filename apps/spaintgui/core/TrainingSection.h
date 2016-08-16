/**
 * spaintgui: TrainingSection.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_TRAININGSECTION
#define H_SPAINTGUI_TRAININGSECTION

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>

#include <spaint/sampling/interface/PerLabelVoxelSampler.h>
#include <spaint/selectors/Selector.h>

#include "TrainingState.h"

/**
 * \brief TODO
 */
class TrainingSection
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMLibSettings> Settings_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The maximum number of labels that can be in use. */
  size_t m_maxLabelCount;

  /** The maximum number of voxels per label from which to train each frame. */
  size_t m_maxTrainingVoxelsPerLabel;

  /** A memory block in which to store a mask indicating which labels are currently in use and from which we want to train. */
  boost::shared_ptr<ORUtils::MemoryBlock<bool> > m_trainingLabelMaskMB;

  /** The voxel sampler used in training mode. */
  spaint::PerLabelVoxelSampler_CPtr m_trainingSampler;

  /** A memory block in which to store the number of voxels sampled for each label for training purposes. */
  boost::shared_ptr<ORUtils::MemoryBlock<unsigned int> > m_trainingVoxelCountsMB;

  /** A memory block in which to store the locations of the voxels sampled for training purposes. */
  spaint::Selector::Selection_Ptr m_trainingVoxelLocationsMB;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  TrainingSection(const Vector2i& depthImageSize, unsigned int seed, const Settings_CPtr& settings, size_t maxLabelCount);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  size_t get_max_training_voxel_count() const;

  /** TODO */
  virtual void run(TrainingState& state, const RenderState_CPtr& samplingRenderState);
};

#endif
