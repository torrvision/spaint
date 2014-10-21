/**
 * spaintrunner: CUDAInstantiations.cu
 */

#include <ITMLib/Engine/DeviceSpecific/CUDA/ITMSceneReconstructionEngine_CUDA.cu>
#include <ITMLib/Engine/DeviceSpecific/CUDA/ITMSwappingEngine_CUDA.cu>
#include <ITMLib/Engine/DeviceSpecific/CUDA/ITMVisualisationEngine_CUDA.cu>
#include <spaint/util/SpaintVoxel.h>
using namespace spaint;

template ITMSceneReconstructionEngine_CUDA<SpaintVoxel,ITMVoxelIndex>;
template ITMSwappingEngine_CUDA<SpaintVoxel,ITMVoxelIndex>;
template ITMVisualisationEngine_CUDA<SpaintVoxel,ITMVoxelIndex>;
