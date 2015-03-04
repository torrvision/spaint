/**
 * spaint: DepthCalculator_Shared.h
 */

#ifndef H_SPAINT_DEPTHVISUALISER_SHARED
#define H_SPAINT_DEPTHVISUALISER_SHARED

#include <cstdlib>

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>
#include <ITMLib/Engine/DeviceAgnostic/ITMVisualisationEngine.h>

namespace spaint {

/*_CPU_AND_GPU_CODE_
inline float scale_within_range(float inVal, float minInVal, float maxInVal, float minOutVal, float maxOutVal)
{
  float inRange = maxInVal - minInVal;
  return ((inVal - minInVal)/inRange)*(maxOutVal - minInVal) + minOutVal;
}
*/

//#################### SHARED HELPER FUNCTIONS #################### 

/**
 * \brief Computes the necessart depth of a pixel from the camera plane.
 *
 * \param dest        A location into which to write the computed colour.
 * \param point       The location of the point (if any) on the scene surface that was hit by a ray passing from the camera through the pixel.
 * \param foundPoint  A flag indicating whether of not any point was actually hit by the ray (true if yes; false if no).
 */
_CPU_AND_GPU_CODE_
inline void shade_pixel_euclidean_distance(float& dest, const Vector3f& cameraPoint, const Vector3f& worldPoint, float voxelSize, bool foundPoint)
{
  dest = -1.0f;
  if(foundPoint)
  {
    //In general the distance from a point to a plane may be calculated by taking the dot product between the normal to the plane and the point and adding the distance of the plane
    //from the origin of the coordinate system. 
    //
    //In free view, the normal to the plane at the origin is [0 0 1], which when dotted with the 3D point result in the z coordinate.
    //
    //Now to scale the depth to fit a colour range!

    // Fill in the final colour for the pixel.
    float dx = abs(cameraPoint.x - worldPoint.x);
    float dy = abs(cameraPoint.y - worldPoint.y);
    float dz = abs(cameraPoint.z - worldPoint.z);

    dest = sqrt(dx * dx + dy * dy + dz * dz) * voxelSize;
  }
}

_CPU_AND_GPU_CODE_
inline void shade_pixel_orthographic_distance(float& dest, const Vector3f& cameraPoint, const Vector3f& cameraLookVector, const Vector3f& worldPoint, float voxelSize, bool foundPoint)
{
  dest = -1.0f;
  if(foundPoint)
  {
    //In general the distance from a point to a plane may be calculated by taking the dot product between the normal to the plane and the point and adding the distance of the plane
    //from the origin of the coordinate system. 
    //
    //In free view, the normal to the plane at the origin is [0 0 1], which when dotted with the 3D point result in the z coordinate.
    //
    //Now to scale the depth to fit a colour range!

    // Fill in the final colour for the pixel.
    dest = (ORUtils::dot(worldPoint, cameraLookVector) - ORUtils::dot(cameraPoint, cameraLookVector)) * voxelSize;
  }
}

}

#endif
