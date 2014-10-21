/**
 * spaintrunner: CUDAInstantiations.cu
 */

#include <ITMLib/Engine/DeviceSpecific/CUDA/ITMSceneReconstructionEngine_CUDA.cu>
#include <ITMLib/Engine/DeviceSpecific/CUDA/ITMSwappingEngine_CUDA.cu>
#include <ITMLib/Engine/DeviceSpecific/CUDA/ITMVisualisationEngine_CUDA.cu>
#include <spaint/util/SpaintVoxel.h>
using namespace spaint;

template class ITMSceneReconstructionEngine_CUDA<SpaintVoxel,ITMVoxelIndex>;
template class ITMSwappingEngine_CUDA<SpaintVoxel,ITMVoxelIndex>;
template class ITMVisualisationEngine_CUDA<SpaintVoxel,ITMVoxelIndex>;
