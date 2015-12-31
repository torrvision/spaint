/**
 * spaint: DepthVisualiser_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_DEPTHVISUALISER_SHARED
#define H_SPAINT_DEPTHVISUALISER_SHARED

#include <cmath>

#include <ITMLib/Objects/Scene/ITMRepresentationAccess.h>
#include <ITMLib/Visualisation/Shared/ITMVisualisationEngine_Shared.h>

namespace spaint {

//#################### SHARED HELPER FUNCTIONS ####################

/**
 * \brief Computes the depth for a pixel in a depth visualisation of the scene.
 *
 * \param dest              A location into which to write the computed depth.
 * \param point             The location of the point (if any) on the scene surface that was hit by a ray passing from the camera through the pixel.
 * \param foundPoint        A flag indicating whether or not any point was actually hit by the ray (true if yes; false if no).
 * \param cameraPosition    The camera position (in world space).
 * \param cameraLookVector  The camera look vector.
 * \param voxelSize         The size of an InfiniTAM voxel (in metres).
 * \param invalidDepthValue The depth value to use for pixels whose rays do not intersect the scene.
 * \param depthType         The type of depth calculation to use.
 */
_CPU_AND_GPU_CODE_
inline void shade_pixel_depth(float& dest, const Vector3f& point, bool foundPoint, const Vector3f& cameraPosition, const Vector3f& cameraLookVector,
                              float voxelSize, float invalidDepthValue, DepthVisualiser::DepthType depthType)
{
  dest = invalidDepthValue;
  if(foundPoint)
  {
    if(depthType == DepthVisualiser::DT_EUCLIDEAN)
    {
      float dx = abs(cameraPosition.x - point.x);
      float dy = abs(cameraPosition.y - point.y);
      float dz = abs(cameraPosition.z - point.z);
      dest = sqrt(dx * dx + dy * dy + dz * dz);
    }
    else if(depthType == DepthVisualiser::DT_ORTHOGRAPHIC)
    {
      dest = ORUtils::dot(point, cameraLookVector) - ORUtils::dot(cameraPosition, cameraLookVector);
    }
  }
}

}

#endif
