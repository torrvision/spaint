/**
 * spaintgui: PredictionState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_PREDICTIONSTATE
#define H_SPAINTGUI_PREDICTIONSTATE

#include <rafl/core/RandomForest.h>

#include <spaint/features/interface/FeatureCalculator.h>
#include <spaint/sampling/interface/UniformVoxelSampler.h>
#include <spaint/util/SpaintVoxel.h>

#include "Interactor.h"

/**
 * \brief TODO
 */
class PredictionState
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const rafl::RandomForest<spaint::SpaintVoxel::Label> > RandomForest_CPtr;

  //#################### DESTRUCTOR ####################
public:
  virtual ~PredictionState() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  virtual const spaint::FeatureCalculator_CPtr& get_feature_calculator() const = 0;

  /**
   * \brief TODO
   */
  virtual RandomForest_CPtr get_forest() const = 0;

  /**
   * \brief TODO
   */
  virtual const Interactor_Ptr& get_interactor() const = 0;

  /**
   * \brief TODO
   */
  virtual size_t get_max_prediction_voxel_count() const = 0;

  /**
   * \brief TODO
   */
  virtual const Model_Ptr& get_model() const = 0;

  /**
   * \brief TODO
   */
  virtual const boost::shared_ptr<ORUtils::MemoryBlock<float> >& get_prediction_features() = 0;

  /**
   * \brief TODO
   */
  virtual const boost::shared_ptr<ORUtils::MemoryBlock<spaint::SpaintVoxel::PackedLabel> >& get_prediction_labels() = 0;

  /**
   * \brief TODO
   */
  virtual const spaint::UniformVoxelSampler_CPtr& get_prediction_sampler() const = 0;

  /**
   * \brief TODO
   */
  virtual const spaint::Selector::Selection_Ptr& get_prediction_voxel_locations() = 0;
};

#endif
