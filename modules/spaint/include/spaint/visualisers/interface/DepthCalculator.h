/**
 * spaint: DepthCalculator.h
 */

#ifndef H_SPAINT_DEPTHVISUALISER
#define H_SPAINT_DEPTHVISUALISER

#include <ITMLib/Objects/ITMIntrinsics.h>
#include <ITMLib/Objects/ITMPose.h>
#include <ITMLib/Objects/ITMRenderState.h>
#include <ITMLib/Objects/ITMScene.h>

#include "rigging/SimpleCamera.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to render a depth visualisation of an InfiniTAM scene.
 */
class DepthCalculator
{
  //#################### ENUMERATIONS #################### 
public:
  /**
   * \brief An enumeration specifying the different typesof depth raycasting that is supported.
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
  virtual ~DepthCalculator() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS #################### 
public:
  /**
   * \brief REnders a depth view of the specified scene fro the specified camera pose.
   *
   * \param renderState The render state corresponding to the specified camera pose.
   * \param outputImage The image into which to write the semantic visualisation of the scene.
   */
  virtual void render_euclidean_distance(ITMFloatImage *outputImage, const ITMLib::Objects::ITMRenderState *renderState, const rigging::SimpleCamera *camera, float voxelSize) const = 0;

  virtual void render_orthographic_distance(ITMFloatImage *outputImage, const ITMLib::Objects::ITMRenderState *renderState, const rigging::SimpleCamera *camera, float voxelSize) const = 0;

  virtual void render_depth(ITMFloatImage *outputImage, const ITMLib::Objects::ITMRenderState *renderState, const rigging::SimpleCamera *camera, float voxelSize, DepthType = DT_ORTHOGRAPHIC) const = 0;
};

}

#endif
