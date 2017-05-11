/**
 * spaint: SpaintRefiningRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "util/SpaintRefiningRelocaliser.h"

#include <itmx/relocalisation/ICPRefiningRelocaliser.tpp>

using namespace spaint;

namespace itmx {

//#################### EXPLICIT TEMPLATE INSTANTIATIONS ####################

template class ICPRefiningRelocaliser<SpaintVoxel, ITMVoxelIndex>;

} // namespace itmx
