/**
 * spaint: PropagationContext.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_PROPAGATIONCONTEXT
#define H_SPAINT_PROPAGATIONCONTEXT

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/Scene/ITMScene.h>

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

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the propagation context.
   */
  virtual ~PropagationContext() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  virtual const Scene_Ptr& get_scene() = 0;
  virtual SpaintVoxel::Label get_semantic_label() const = 0;
};

}

#endif
