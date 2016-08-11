/**
 * spaintgui: TrainingState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_TRAININGSTATE
#define H_SPAINTGUI_TRAININGSTATE

#include <rafl/core/RandomForest.h>

#include <spaint/features/interface/FeatureCalculator.h>
#include <spaint/sampling/interface/PerLabelVoxelSampler.h>
#include <spaint/selectors/Selector.h>
#include <spaint/util/SpaintVoxel.h>

#include "Model.h"

/**
 * \brief TODO
 */
class TrainingState
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<rafl::RandomForest<spaint::SpaintVoxel::Label> > RandomForest_Ptr;

  //#################### DESTRUCTOR ####################
public:
  virtual ~TrainingState() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  virtual const spaint::FeatureCalculator_CPtr& get_feature_calculator() const = 0;

  /**
   * \brief TODO
   */
  virtual const RandomForest_Ptr& get_forest() = 0;

  /**
   * \brief TODO
   */
  virtual size_t get_max_training_voxels_per_label() const = 0;

  /**
   * \brief TODO
   */
  virtual const Model_Ptr& get_model() const = 0;

  /**
   * \brief TODO
   */
  virtual const boost::shared_ptr<ORUtils::MemoryBlock<float> >& get_training_features() const = 0;

  /**
   * \brief TODO
   */
  virtual const boost::shared_ptr<ORUtils::MemoryBlock<bool> >& get_training_label_mask() const = 0;

  /**
   * \brief TODO
   */
  virtual const spaint::PerLabelVoxelSampler_CPtr& get_training_sampler() const = 0;

  /**
   * \brief TODO
   */
  virtual const boost::shared_ptr<ORUtils::MemoryBlock<unsigned int> >& get_training_voxel_counts() const = 0;

  /**
   * \brief TODO
   */
  virtual const spaint::Selector::Selection_Ptr& get_training_voxel_locations() = 0;
};

#endif
