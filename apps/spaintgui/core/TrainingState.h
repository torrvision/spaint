/**
 * spaintgui: TrainingState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_TRAININGSTATE
#define H_SPAINTGUI_TRAININGSTATE

#include <rafl/core/RandomForest.h>

#include <spaint/features/interface/FeatureCalculator.h>
#include <spaint/util/LabelManager.h>
#include <spaint/util/SpaintVoxel.h>

/**
 * \brief TODO
 */
class TrainingState
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<rafl::RandomForest<spaint::SpaintVoxel::Label> > RandomForest_Ptr;
  typedef ITMLib::ITMScene<spaint::SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;

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
  virtual const spaint::LabelManager_Ptr& get_label_manager() const = 0;

  /**
   * \brief TODO
   */
  virtual const Scene_Ptr& get_scene() const = 0;

  /**
   * \brief TODO
   */
  virtual const boost::shared_ptr<ORUtils::MemoryBlock<float> >& get_training_features() const = 0;
};

#endif
