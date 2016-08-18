/**
 * spaint: PredictionModel.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_PREDICTIONMODEL
#define H_SPAINT_PREDICTIONMODEL

#include <rafl/core/RandomForest.h>

#include "../features/interface/FeatureCalculator.h"
#include "../markers/interface/VoxelMarker.h"
#include "../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief TODO
 */
class PredictionModel
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<rafl::RandomForest<SpaintVoxel::Label> > RandomForest_Ptr;
  typedef ITMLib::ITMScene<SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;

  //#################### DESTRUCTOR ####################
public:
  virtual ~PredictionModel() {}

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
  virtual const boost::shared_ptr<ORUtils::MemoryBlock<float> >& get_prediction_features() = 0;

  /**
   * \brief TODO
   */
  virtual const Scene_Ptr& get_scene() const = 0;

  /**
   * \brief TODO
   */
  virtual const VoxelMarker_CPtr& get_voxel_marker() const = 0;
};

}

#endif
