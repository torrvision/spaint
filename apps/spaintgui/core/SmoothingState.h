/**
 * spaintgui: SmoothingState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_SMOOTHINGSTATE
#define H_SPAINTGUI_SMOOTHINGSTATE

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/Scene/ITMScene.h>

#include <spaint/util/SpaintVoxel.h>

/**
 * \brief TODO
 */
class SmoothingState
{
  //#################### TYPEDEFS ####################
private:
  typedef ITMLib::ITMScene<spaint::SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;

  //#################### DESTRUCTOR ####################
public:
  virtual ~SmoothingState() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  virtual const Scene_Ptr& get_scene() const = 0;
};

#endif
