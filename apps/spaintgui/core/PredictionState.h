/**
 * spaintgui: PredictionState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_PREDICTIONSTATE
#define H_SPAINTGUI_PREDICTIONSTATE

#include <rafl/core/RandomForest.h>

#include <spaint/features/interface/FeatureCalculator.h>
#include <spaint/markers/interface/VoxelMarker.h>
#include <spaint/util/SpaintVoxel.h>

/**
 * \brief TODO
 */
class PredictionState
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<rafl::RandomForest<spaint::SpaintVoxel::Label> > RandomForest_Ptr;
  typedef ITMLib::ITMScene<spaint::SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;

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
  virtual const spaint::VoxelMarker_CPtr& get_voxel_marker() const = 0;
};

#endif
