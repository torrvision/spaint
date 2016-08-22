/**
 * spaint: SmoothingContext.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SMOOTHINGCONTEXT
#define H_SPAINT_SMOOTHINGCONTEXT

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/Scene/ITMScene.h>

#include "../util/LabelManager.h"
#include "../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one provides the shared context needed by a smoothing component.
 */
class SmoothingContext
{
  //#################### TYPEDEFS ####################
private:
  typedef ITMLib::ITMScene<SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;
  typedef boost::shared_ptr<const ITMLib::ITMLibSettings> Settings_CPtr;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the smoothing context.
   */
  virtual ~SmoothingContext() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  virtual const LabelManager_Ptr& get_label_manager() = 0;
  virtual const Scene_Ptr& get_scene() = 0;
  virtual const Settings_CPtr& get_settings() const = 0;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SmoothingContext> SmoothingContext_Ptr;

}

#endif
