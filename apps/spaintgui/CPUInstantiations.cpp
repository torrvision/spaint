/**
 * spaintgui: CPUInstantiations.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include <ITMLib/Core/ITMDenseMapper.tpp>
#include <ITMLib/Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.tpp>
#include <ITMLib/Swapping/CPU/ITMSwappingEngine_CPU.tpp>
#include <ITMLib/Trackers/CPU/ITMRenTracker_CPU.tpp>
#include <ITMLib/Trackers/Interface/ITMRenTracker.tpp>
#include <ITMLib/Visualisation/CPU/ITMVisualisationEngine_CPU.tpp>
#include <spaint/util/SpaintVoxel.h>
using namespace spaint;

template class ITMDenseMapper<SpaintVoxel,ITMVoxelIndex>;
template class ITMRenTracker<SpaintVoxel,ITMVoxelIndex>;
template class ITMRenTracker_CPU<SpaintVoxel,ITMVoxelIndex>;
template class ITMSceneReconstructionEngine_CPU<SpaintVoxel,ITMVoxelIndex>;
template class ITMSwappingEngine_CPU<SpaintVoxel,ITMVoxelIndex>;
template class ITMVisualisationEngine_CPU<SpaintVoxel,ITMVoxelIndex>;
