/**
 * spaint: SLAMContext.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SLAMCONTEXT
#define H_SPAINT_SLAMCONTEXT

#include <map>

#include <ITMLib/Engines/Visualisation/Interface/ITMSurfelVisualisationEngine.h>
#include <ITMLib/Engines/Visualisation/Interface/ITMVisualisationEngine.h>

#include "../slamstate/SLAMState.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one provides the shared context needed by SLAM components.
 */
class SLAMContext
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMSurfelVisualisationEngine<SpaintSurfel> > SurfelVisualisationEngine_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMVisualisationEngine<SpaintVoxel,ITMVoxelIndex> > VoxelVisualisationEngine_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The states of the SLAM reconstructions for the various scenes. */
  std::map<std::string,SLAMState_Ptr> m_slamStates;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the SLAM context.
   */
  virtual ~SLAMContext() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  virtual const Settings_CPtr& get_settings() const = 0;
  virtual SurfelVisualisationEngine_CPtr get_surfel_visualisation_engine() const = 0;
  virtual std::string get_tag() const = 0;
  virtual VoxelVisualisationEngine_CPtr get_voxel_visualisation_engine() const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the SLAM state for the specified scene.
   *
   * \param sceneID The scene ID.
   * \return        The SLAM state for the specified scene.
   */
  virtual const SLAMState_Ptr& get_slam_state(const std::string& sceneID);

  /**
   * \brief Gets the SLAM state for the specified scene.
   *
   * \param sceneID The scene ID.
   * \return        The SLAM state for the specified scene.
   */
  virtual SLAMState_CPtr get_slam_state(const std::string& sceneID) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SLAMContext> SLAMContext_Ptr;

}

#endif
