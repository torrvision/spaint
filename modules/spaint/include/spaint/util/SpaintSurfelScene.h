/**
 * spaint: SpaintSurfelScene.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SPAINTSURFELSCENE
#define H_SPAINT_SPAINTSURFELSCENE

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/Scene/ITMSurfelScene.h>

#include "SpaintSurfel.h"

namespace spaint {

typedef ITMLib::ITMSurfelScene<SpaintSurfel> SpaintSurfelScene;
typedef boost::shared_ptr<SpaintSurfelScene> SpaintSurfelScene_Ptr;
typedef boost::shared_ptr<const SpaintSurfelScene> SpaintSurfelScene_CPtr;

}

#endif
