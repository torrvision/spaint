/**
 * spaint: TrainingModel.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_TRAININGMODEL
#define H_SPAINT_TRAININGMODEL

#include <rafl/core/RandomForest.h>

#include "../features/interface/FeatureCalculator.h"
#include "../util/LabelManager.h"
#include "../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief TODO
 */
class TrainingModel
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<rafl::RandomForest<SpaintVoxel::Label> > RandomForest_Ptr;
  typedef ITMLib::ITMScene<SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;

  //#################### DESTRUCTOR ####################
public:
  virtual ~TrainingModel() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  virtual const FeatureCalculator_CPtr& get_feature_calculator() const = 0;

  /**
   * \brief TODO
   */
  virtual const RandomForest_Ptr& get_forest() = 0;

  /**
   * \brief TODO
   */
  virtual const LabelManager_Ptr& get_label_manager() const = 0;

  /**
   * \brief Gets the current reconstructed scene.
   *
   * \return  The current reconstructed scene.
   */
  virtual const Scene_Ptr& get_scene() = 0;

  /**
   * \brief TODO
   */
  virtual const boost::shared_ptr<ORUtils::MemoryBlock<float> >& get_training_features() const = 0;
};

}

#endif
