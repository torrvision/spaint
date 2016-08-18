/**
 * spaint: SmoothingModel.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SMOOTHINGMODEL
#define H_SPAINT_SMOOTHINGMODEL

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/Scene/ITMScene.h>

#include "../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief TODO
 */
class SmoothingModel
{
  //#################### TYPEDEFS ####################
private:
  typedef ITMLib::ITMScene<SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief TODO
   */
  virtual ~SmoothingModel() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the current reconstructed scene.
   *
   * \return  The current reconstructed scene.
   */
  virtual const Scene_Ptr& get_scene() = 0;
};

}

#endif
