/**
 * spaint: PredictionComponent.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_PREDICTIONCOMPONENT
#define H_SPAINT_PREDICTIONCOMPONENT

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>
#include <ITMLib/Utils/ITMLibSettings.h>

#include "PredictionModel.h"
#include "../sampling/interface/UniformVoxelSampler.h"
#include "../selectors/Selector.h"

namespace spaint {

/**
 * \brief TODO
 */
class PredictionComponent
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMLibSettings> Settings_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The maximum number of voxels for which to predict labels each frame. */
  size_t m_maxPredictionVoxelCount;

  /** A memory block in which to store the labels predicted for the various voxels. */
  boost::shared_ptr<ORUtils::MemoryBlock<SpaintVoxel::PackedLabel> > m_predictionLabelsMB;

  /** The voxel sampler used in prediction mode. */
  UniformVoxelSampler_CPtr m_predictionSampler;

  /** A memory block in which to store the locations of the voxels sampled for prediction purposes. */
  Selector::Selection_Ptr m_predictionVoxelLocationsMB;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  PredictionComponent(const Vector2i& depthImageSize, unsigned int seed, const Settings_CPtr& settings);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  size_t get_max_prediction_voxel_count() const;

  /** TODO */
  virtual void run(PredictionModel& model, const RenderState_CPtr& samplingRenderState);
};

}

#endif
