/**
 * spaintgui: CUDAInstantiations.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include <ITMLib/Engine/DeviceSpecific/CUDA/ITMRenTracker_CUDA.cu>
#include <ITMLib/Engine/DeviceSpecific/CUDA/ITMSceneReconstructionEngine_CUDA.cu>
#include <ITMLib/Engine/DeviceSpecific/CUDA/ITMSwappingEngine_CUDA.cu>
#include <ITMLib/Engine/DeviceSpecific/CUDA/ITMVisualisationEngine_CUDA.cu>
#include <spaint/util/SpaintVoxel.h>
using namespace spaint;

template class ITMRenTracker_CUDA<SpaintVoxel,ITMVoxelIndex>;
template class ITMSceneReconstructionEngine_CUDA<SpaintVoxel,ITMVoxelIndex>;
template class ITMSwappingEngine_CUDA<SpaintVoxel,ITMVoxelIndex>;
template class ITMVisualisationEngine_CUDA<SpaintVoxel,ITMVoxelIndex>;
