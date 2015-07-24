/**
 * spaintgui: CPUInstantiations.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include <ITMLib/Engine/ITMDenseMapper.cpp>
#include <ITMLib/Engine/ITMRenTracker.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMSceneReconstructionEngine_CPU.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMSwappingEngine_CPU.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMVisualisationEngine_CPU.cpp>
#include <spaint/util/SpaintVoxel.h>
using namespace spaint;

template class ITMDenseMapper<SpaintVoxel,ITMVoxelIndex>;
template class ITMRenTracker<SpaintVoxel,ITMVoxelIndex>;
template class ITMSceneReconstructionEngine_CPU<SpaintVoxel,ITMVoxelIndex>;
template class ITMSwappingEngine_CPU<SpaintVoxel,ITMVoxelIndex>;
template class ITMVisualisationEngine_CPU<SpaintVoxel,ITMVoxelIndex>;
