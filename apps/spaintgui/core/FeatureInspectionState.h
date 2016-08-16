/**
 * spaintgui: FeatureInspectionState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_FEATUREINSPECTIONSTATE
#define H_SPAINTGUI_FEATUREINSPECTIONSTATE

#include <spaint/features/interface/FeatureCalculator.h>
#include <spaint/selectors/Selector.h>

/**
 * \brief TODO
 */
class FeatureInspectionState
{
  //#################### TYPEDEFS ####################
private:
  typedef ITMLib::ITMScene<spaint::SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;

  //#################### DESTRUCTOR ####################
public:
  virtual ~FeatureInspectionState() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  virtual const spaint::FeatureCalculator_CPtr& get_feature_calculator() const = 0;

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
  virtual spaint::Selector_CPtr get_selector() const = 0;
};

#endif
