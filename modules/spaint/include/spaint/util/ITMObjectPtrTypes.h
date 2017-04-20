/**
 * spaint: ITMObjectPtrTypes.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include <boost/shared_ptr.hpp>

#include <InputSource/CompositeImageSourceEngine.h>
#include <ITMLib/Objects/RenderStates/ITMRenderState.h>
#include <ITMLib/Objects/RenderStates/ITMSurfelRenderState.h>
#include <ITMLib/Objects/Tracking/ITMTrackingState.h>
#include <ITMLib/Objects/Views/ITMView.h>
#include <ITMLib/Trackers/Interface/ITMTracker.h>
#include <ITMLib/Utils/ITMLibSettings.h>

typedef boost::shared_ptr<InputSource::CompositeImageSourceEngine> CompositeImageSourceEngine_Ptr;
typedef boost::shared_ptr<InputSource::ImageSourceEngine> ImageSourceEngine_Ptr;
typedef boost::shared_ptr<ITMLib::ITMLibSettings> Settings_Ptr;
typedef boost::shared_ptr<ITMLib::ITMSurfelRenderState> SurfelRenderState_Ptr;
typedef boost::shared_ptr<ITMLib::ITMTracker> Tracker_Ptr;
typedef boost::shared_ptr<ITMLib::ITMTrackingState> TrackingState_Ptr;
typedef boost::shared_ptr<ITMLib::ITMView> View_Ptr;
typedef boost::shared_ptr<ITMLib::ITMRenderState> VoxelRenderState_Ptr;

typedef boost::shared_ptr<const InputSource::CompositeImageSourceEngine> CompositeImageSourceEngine_CPtr;
typedef boost::shared_ptr<const InputSource::ImageSourceEngine> ImageSourceEngine_CPtr;
typedef boost::shared_ptr<const ITMLib::ITMLibSettings> Settings_CPtr;
typedef boost::shared_ptr<const ITMLib::ITMSurfelRenderState> SurfelRenderState_CPtr;
typedef boost::shared_ptr<const ITMLib::ITMTracker> Tracker_CPtr;
typedef boost::shared_ptr<const ITMLib::ITMTrackingState> TrackingState_CPtr;
typedef boost::shared_ptr<const ITMLib::ITMView> View_CPtr;
typedef boost::shared_ptr<const ITMLib::ITMRenderState> VoxelRenderState_CPtr;
