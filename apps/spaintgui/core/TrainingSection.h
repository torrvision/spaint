/**
 * spaintgui: TrainingSection.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_TRAININGSECTION
#define H_SPAINTGUI_TRAININGSECTION

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>

#include <spaint/sampling/interface/PerLabelVoxelSampler.h>

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

  /** The voxel sampler used in training mode. */
  spaint::PerLabelVoxelSampler_CPtr m_trainingSampler;

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
