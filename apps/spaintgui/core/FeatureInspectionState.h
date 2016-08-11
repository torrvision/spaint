/**
 * spaintgui: FeatureInspectionState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_FEATUREINSPECTIONSTATE
#define H_SPAINTGUI_FEATUREINSPECTIONSTATE

#include <spaint/features/interface/FeatureCalculator.h>

#include "Interactor.h"

/**
 * \brief TODO
 */
class FeatureInspectionState
{
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
  virtual const Interactor_Ptr& get_interactor() const = 0;

  /**
   * \brief TODO
   */
  virtual const Model_Ptr& get_model() const = 0;

  /**
   * \brief TODO
   */
  virtual size_t get_patch_size() const = 0;
};

#endif
