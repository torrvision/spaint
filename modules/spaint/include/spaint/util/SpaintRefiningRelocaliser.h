/**
 * spaint: SpaintRefiningRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_SPAINT_SPAINTREFININGRELOCALISER
#define H_SPAINT_SPAINTREFININGRELOCALISER

#include <boost/shared_ptr.hpp>

#include <itmx/relocalisation/ICPRefiningRelocaliser.h>

#include "SpaintVoxel.h"

namespace spaint {

typedef itmx::ICPRefiningRelocaliser<SpaintVoxel, ITMVoxelIndex> SpaintRefiningRelocaliser;
typedef boost::shared_ptr<SpaintRefiningRelocaliser> SpaintRefiningRelocaliser_Ptr;
typedef boost::shared_ptr<const SpaintRefiningRelocaliser> SpaintRefiningRelocaliser_CPtr;

} // namespace spaint

#endif // H_SPAINT_SPAINTREFININGRELOCALISER
