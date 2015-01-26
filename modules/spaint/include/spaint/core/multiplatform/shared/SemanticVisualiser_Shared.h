/**
 * spaint: SemanticVisualiser_Shared.h
 */

#ifndef H_SPAINT_SEMANTICVISUALISER_SHARED
#define H_SPAINT_SEMANTICVISUALISER_SHARED

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>
#include <ITMLib/Engine/DeviceAgnostic/ITMVisualisationEngine.h>

#include "../../voxels/SpaintVoxel.h"

namespace spaint {

//#################### SHARED HELPER FUNCTIONS ####################

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void processPixelSemantic(DEVICEPTR(Vector4u)& dest, const DEVICEPTR(Vector3f)& point, bool foundPoint, const DEVICEPTR(SpaintVoxel) *voxelData,
                                 const DEVICEPTR(ITMVoxelIndex::IndexData) *voxelIndex, Vector3f lightSource, const DEVICEPTR(Vector3u) *labelColours)
{
  dest = Vector4u((uchar)0);
  if(foundPoint)
  {
    Vector3f outNormal;
    float angle;
    computeNormalAndAngle<SpaintVoxel,ITMVoxelIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

    float scale = 0.8f * angle + 0.2f;
    SpaintVoxel voxel = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(point.x), (int)ROUND(point.y), (int)ROUND(point.z)), foundPoint);
    Vector3u colour = labelColours[voxel.w_depth / 32 > 3 ? 3 : voxel.w_depth / 32];
    dest.x = (uchar)(scale * colour.r);
    dest.y = (uchar)(scale * colour.g);
    dest.z = (uchar)(scale * colour.b);
    dest.w = 255;
  }
}

}

#endif
