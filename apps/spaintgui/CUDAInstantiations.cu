/**
 * spaintgui: CUDAInstantiations.cu
 */

#include <ITMLib/Engine/ITMDenseMapper.cpp>
#include <ITMLib/Engine/ITMRenTracker.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMSceneReconstructionEngine_CPU.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMSwappingEngine_CPU.cpp>
#include <ITMLib/Engine/DeviceSpecific/CUDA/ITMRenTracker_CUDA.cu>
#include <ITMLib/Engine/DeviceSpecific/CUDA/ITMSceneReconstructionEngine_CUDA.cu>
#include <ITMLib/Engine/DeviceSpecific/CUDA/ITMSwappingEngine_CUDA.cu>
#include <ITMLib/Engine/DeviceSpecific/CUDA/ITMVisualisationEngine_CUDA.cu>
#include <spaint/util/SpaintVoxel.h>
using namespace spaint;

template class ITMDenseMapper<SpaintVoxel,ITMVoxelIndex>;
template class ITMRenTracker<SpaintVoxel,ITMVoxelIndex>;
template class ITMRenTracker_CUDA<SpaintVoxel,ITMVoxelIndex>;
template class ITMSceneReconstructionEngine_CPU<SpaintVoxel,ITMVoxelIndex>;
template class ITMSceneReconstructionEngine_CUDA<SpaintVoxel,ITMVoxelIndex>;
template class ITMSwappingEngine_CPU<SpaintVoxel,ITMVoxelIndex>;
template class ITMSwappingEngine_CUDA<SpaintVoxel,ITMVoxelIndex>;
template class ITMVisualisationEngine_CUDA<SpaintVoxel,ITMVoxelIndex>;
