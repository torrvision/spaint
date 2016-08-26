/**
 * spaint: SmoothingComponent.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SMOOTHINGCOMPONENT
#define H_SPAINT_SMOOTHINGCOMPONENT

#include <ITMLib/Objects/RenderStates/ITMRenderState.h>

#include "SmoothingContext.h"
#include "../smoothing/interface/LabelSmoother.h"

namespace spaint {

/**
 * \brief An instance of this pipeline component can be used to smoothe the labels in a scene to remove noise.
 */
class SmoothingComponent
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The shared context needed for smoothing. */
  SmoothingContext_Ptr m_context;

  /** The label smoother. */
  LabelSmoother_CPtr m_labelSmoother;

  /** The ID of the scene on which the component should operate. */
  std::string m_sceneID;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a smoothing component.
   *
   * \param context The shared context needed for smoothing.
   * \param sceneID The ID of the scene on which the component should operate.
   */
  SmoothingComponent(const SmoothingContext_Ptr& context, const std::string& sceneID);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Runs the smoothing component, smoothing the labels in the target scene to remove noise.
   *
   * \param renderState The voxel render state associated with the camera position from which to smoothe.
   */
  void run(const VoxelRenderState_CPtr& renderState);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SmoothingComponent> SmoothingComponent_Ptr;

}

#endif
