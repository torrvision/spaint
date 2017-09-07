/**
 * spaint: SpaintVoxelScene.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SPAINTVOXELSCENE
#define H_SPAINT_SPAINTVOXELSCENE

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/Scene/ITMScene.h>

#include "SpaintVoxel.h"

namespace spaint {

typedef ITMLib::ITMScene<SpaintVoxel,ITMVoxelIndex> SpaintVoxelScene;
typedef boost::shared_ptr<SpaintVoxelScene> SpaintVoxelScene_Ptr;
typedef boost::shared_ptr<const SpaintVoxelScene> SpaintVoxelScene_CPtr;

}

#endif
