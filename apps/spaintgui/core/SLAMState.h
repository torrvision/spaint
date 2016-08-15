/**
 * spaintgui: SLAMState.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_SLAMSTATE
#define H_SPAINTGUI_SLAMSTATE

#include <InputSource/CompositeImageSourceEngine.h>
#include <ITMLib/Core/ITMDenseMapper.h>
#include <ITMLib/Core/ITMTrackingController.h>
#include <ITMLib/Engines/ViewBuilding/Interface/ITMViewBuilder.h>
#include <RelocLib/PoseDatabase.h>
#include <RelocLib/Relocaliser.h>

#include <spaint/trackers/FallibleTracker.h>

#include "Raycaster.h"

/**
* \brief TODO
*/
class SLAMState
{
  //#################### TYPEDEFS ####################
protected:
  typedef boost::shared_ptr<InputSource::CompositeImageSourceEngine> CompositeImageSourceEngine_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMDenseMapper<spaint::SpaintVoxel,ITMVoxelIndex> > DenseMapper_Ptr;
  typedef boost::shared_ptr<RelocLib::PoseDatabase> PoseDatabase_Ptr;
  typedef boost::shared_ptr<RelocLib::Relocaliser> Relocaliser_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMTrackingController> TrackingController_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMViewBuilder> ViewBuilder_Ptr;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief TODO
   */
  virtual ~SLAMState() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  virtual const DenseMapper_Ptr& get_dense_mapper() const = 0;

  /**
   * \brief TODO
   */
  virtual const spaint::FallibleTracker *get_fallible_tracker() const = 0;

  /**
   * \brief TODO
   */
  virtual bool get_fusion_enabled() const = 0;

  /**
   * \brief TODO
   */
  virtual const CompositeImageSourceEngine_Ptr& get_image_source_engine() const = 0;

  /**
   * \brief TODO
   */
  virtual const ITMShortImage_Ptr& get_input_raw_depth_image() const = 0;

  /**
   * \brief TODO
   */
  virtual const ITMUChar4Image_Ptr& get_input_rgb_image() const = 0;

  /**
   * \brief TODO
   */
  virtual const Model_Ptr& get_model() const = 0;

  /**
   * \brief TODO
   */
  virtual const PoseDatabase_Ptr& get_pose_database() const = 0;

  /**
   * \brief TODO
   */
  virtual const Raycaster_Ptr& get_raycaster() const = 0;

  /**
   * \brief TODO
   */
  virtual const Relocaliser_Ptr& get_relocaliser() const = 0;

  /**
   * \brief TODO
   */
  virtual const TrackingController_Ptr& get_tracking_controller() const = 0;

  /**
   * \brief TODO
   */
  virtual const ViewBuilder_Ptr& get_view_builder() const = 0;

  /**
   * \brief TODO
   */
  virtual void set_fusion_enabled(bool fusionEnabled) = 0;
};

#endif
