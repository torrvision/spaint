/**
 * spaintgui: CUDAInstantiations.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include <ITMLib/Reconstruction/CUDA/ITMSceneReconstructionEngine_CUDA.tcu>
#include <ITMLib/Swapping/CUDA/ITMSwappingEngine_CUDA.tcu>
#include <ITMLib/Trackers/CUDA/ITMRenTracker_CUDA.tcu>
#include <ITMLib/Visualisation/CUDA/ITMVisualisationEngine_CUDA.tcu>
#include <spaint/util/SpaintVoxel.h>
using namespace spaint;

template class ITMRenTracker_CUDA<SpaintVoxel,ITMVoxelIndex>;
template class ITMSceneReconstructionEngine_CUDA<SpaintVoxel,ITMVoxelIndex>;
template class ITMSwappingEngine_CUDA<SpaintVoxel,ITMVoxelIndex>;
template class ITMVisualisationEngine_CUDA<SpaintVoxel,ITMVoxelIndex>;
