/**
 * spaintgui: PredictionSection.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_PREDICTIONSECTION
#define H_SPAINTGUI_PREDICTIONSECTION

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>

#include <spaint/sampling/interface/UniformVoxelSampler.h>

#include "PredictionState.h"

/**
 * \brief TODO
 */
class PredictionSection
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMLibSettings> Settings_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The voxel sampler used in prediction mode. */
  spaint::UniformVoxelSampler_CPtr m_predictionSampler;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  PredictionSection(const Vector2i& depthImageSize, unsigned int seed, const Settings_CPtr& settings);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** TODO */
  virtual void run(PredictionState& state, const RenderState_CPtr& samplingRenderState);
};

#endif
