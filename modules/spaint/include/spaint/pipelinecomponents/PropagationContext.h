/**
 * spaint: PropagationContext.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_PROPAGATIONCONTEXT
#define H_SPAINT_PROPAGATIONCONTEXT

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/Scene/ITMScene.h>
#include <ITMLib/Utils/ITMLibSettings.h>

#include "../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one provides the shared context needed by a propagation component.
 */
class PropagationContext
{
  //#################### TYPEDEFS ####################
private:
  typedef ITMLib::ITMScene<SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;
  typedef boost::shared_ptr<const ITMLib::ITMLibSettings> Settings_CPtr;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the propagation context.
   */
  virtual ~PropagationContext() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  virtual const Vector2i& get_depth_image_size() const = 0;
  virtual const Scene_Ptr& get_scene() = 0;
  virtual SpaintVoxel::Label get_semantic_label() const = 0;
  virtual const Settings_CPtr& get_settings() const = 0;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<PropagationContext> PropagationContext_Ptr;

}

#endif
