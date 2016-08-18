/**
 * spaint: FeatureInspectionModel.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_FEATUREINSPECTIONMODEL
#define H_SPAINT_FEATUREINSPECTIONMODEL

#include "../features/interface/FeatureCalculator.h"
#include "../selectors/Selector.h"

namespace spaint {

/**
 * \brief TODO
 */
class FeatureInspectionModel
{
  //#################### TYPEDEFS ####################
private:
  typedef ITMLib::ITMScene<SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;

  //#################### DESTRUCTOR ####################
public:
  virtual ~FeatureInspectionModel() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  virtual const FeatureCalculator_CPtr& get_feature_calculator() const = 0;

  /**
   * \brief TODO
   */
  virtual size_t get_patch_size() const = 0;

  /**
   * \brief TODO
   */
  virtual const Scene_Ptr& get_scene() const = 0;

  /**
   * \brief TODO
   */
  virtual Selector_CPtr get_selector() const = 0;
};

}

#endif
