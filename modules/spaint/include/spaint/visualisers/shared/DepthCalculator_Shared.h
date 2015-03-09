/**
 * spaint: DepthCalculator_Shared.h
 */

#ifndef H_SPAINT_DEPTHVISUALISER_SHARED
#define H_SPAINT_DEPTHVISUALISER_SHARED

#include <cstdlib>

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>
#include <ITMLib/Engine/DeviceAgnostic/ITMVisualisationEngine.h>

namespace spaint {

//#################### SHARED HELPER FUNCTIONS #################### 

/**
 * \brief Computes the necessart depth of a pixel from the camera plane.
 *
 * \param dest        A location into which to write the computed colour.
 * \param point       The location of the point (if any) on the scene surface that was hit by a ray passing from the camera through the pixel.
 * \param foundPoint  A flag indicating whether of not any point was actually hit by the ray (true if yes; false if no).
 */
_CPU_AND_GPU_CODE_
inline void shade_pixel_depth(float& dest, const Vector3f& cameraPoint, const Vector3f& cameraLookVector, const Vector3f& worldPoint, float voxelSize, bool foundPoint, DepthCalculator::DepthType depthType)
{
  dest = -1.0f;
  if(foundPoint)
  {
    if(depthType == DepthCalculator::DT_ORTHOGRAPHIC)
    {
      dest = (ORUtils::dot(worldPoint, cameraLookVector) - ORUtils::dot(cameraPoint, cameraLookVector)) * voxelSize;
    }
    else if(depthType == DepthCalculator::DT_EUCLIDEAN)
    {
      float dx = abs(cameraPoint.x - worldPoint.x);
      float dy = abs(cameraPoint.y - worldPoint.y);
      float dz = abs(cameraPoint.z - worldPoint.z);

      dest = sqrt(dx * dx + dy * dy + dz * dz) * voxelSize;
    }
  }
}

}

#endif
