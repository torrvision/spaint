/**
 * spaintgui: PropagationState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_PROPAGATIONSTATE
#define H_SPAINTGUI_PROPAGATIONSTATE

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/Scene/ITMScene.h>

#include <spaint/util/SpaintVoxel.h>

/**
 * \brief TODO
 */
class PropagationState
{
  //#################### TYPEDEFS ####################
private:
  typedef ITMLib::ITMScene<spaint::SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;

  //#################### DESTRUCTOR ####################
public:
  virtual ~PropagationState() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  virtual const Scene_Ptr& get_scene() const = 0;

  /**
   * \brief TODO
   */
  virtual spaint::SpaintVoxel::Label get_semantic_label() const = 0;
};

#endif
