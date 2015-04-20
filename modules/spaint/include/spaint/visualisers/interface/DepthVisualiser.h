/**
 * spaint: DepthVisualiser.h
 */

#ifndef H_SPAINT_DEPTHVISUALISER
#define H_SPAINT_DEPTHVISUALISER

#include <ITMLib/Objects/ITMIntrinsics.h>
#include <ITMLib/Objects/ITMPose.h>
#include <ITMLib/Objects/ITMRenderState.h>
#include <ITMLib/Objects/ITMScene.h>

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to render a depth visualisation of an InfiniTAM scene.
 */
class DepthVisualiser
{
  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief An enumeration specifying the different typesof depth calculation that is supported.
   */
  enum DepthType
  {
    DT_EUCLIDEAN,
    DT_ORTHOGRAPHIC
  };

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the semantic visualiser.
   */
  virtual ~DepthVisualiser() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Renders a depth view of the specified scene from the specified camera pose.
   *
   * \param outputImage       The image into which to write the depth calculation.
   * \param renderState       The render state.
   * \param cameraPosition    The camera translation from the origin of the world coordinate system.
   * \param cameraLookVector  The the camera look vector.
   * \param voxelSize         The size of an InfiniTAM voxel.
   * \param depthType         The type of depth calculation.
   */
  virtual void render_depth(ITMFloatImage *outputImage, const ITMLib::Objects::ITMRenderState *renderState, Vector3f cameraPosition, Vector3f cameraLookVector, float voxelSize, DepthType = DT_ORTHOGRAPHIC) const = 0;
};

}

#endif
