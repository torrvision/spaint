/**
 * spaintgui: CPUInstantiations.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include <ITMLib/Core/ITMDenseMapper.tpp>
#include <ITMLib/Engines/Meshing/CPU/ITMMeshingEngine_CPU.tpp>
#include <ITMLib/Engines/Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.tpp>
#include <ITMLib/Engines/Swapping/CPU/ITMSwappingEngine_CPU.tpp>
#include <ITMLib/Engines/Visualisation/CPU/ITMVisualisationEngine_CPU.tpp>
#include <spaint/util/SpaintVoxel.h>
using namespace spaint;

template class ITMDenseMapper<SpaintVoxel,ITMVoxelIndex>;
template class ITMMeshingEngine_CPU<SpaintVoxel,ITMVoxelIndex>;
template class ITMSceneReconstructionEngine_CPU<SpaintVoxel,ITMVoxelIndex>;
template class ITMSwappingEngine_CPU<SpaintVoxel,ITMVoxelIndex>;
template class ITMVisualisationEngine_CPU<SpaintVoxel,ITMVoxelIndex>;
