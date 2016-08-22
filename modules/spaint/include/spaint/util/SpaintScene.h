/**
 * spaint: SpaintScene.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SPAINTSCENE
#define H_SPAINT_SPAINTSCENE

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/Scene/ITMScene.h>

#include "SpaintVoxel.h"

namespace spaint {

typedef ITMLib::ITMScene<SpaintVoxel,ITMVoxelIndex> Scene;
typedef boost::shared_ptr<Scene> Scene_Ptr;
typedef boost::shared_ptr<const Scene> Scene_CPtr;

}

#endif
