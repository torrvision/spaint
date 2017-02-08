/**
 * spaint: LabelPropagator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_LABELPROPAGATOR
#define H_SPAINT_LABELPROPAGATOR

#include <ITMLib/Utils/ITMImageTypes.h>

#include "../../util/SpaintVoxelScene.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to propagate a specified label across surfaces in the scene.
 */
class LabelPropagator
{
  //#################### PROTECTED VARIABLES ####################
protected:
  /** The largest angle allowed between the normals of neighbouring voxels if propagation is to occur. */
  const float m_maxAngleBetweenNormals;

  /** The maximum squared distance allowed between the colours of neighbouring voxels if propagation is to occur. */
  const float m_maxSquaredDistanceBetweenColours;

  /** The maximum squared distance allowed between the positions of neighbouring voxels if propagation is to occur. */
  const float m_maxSquaredDistanceBetweenVoxels;

  /** A memory block in which to store the surface normals of the voxels in the raycast result. */
  boost::shared_ptr<ORUtils::MemoryBlock<Vector3f> > m_surfaceNormalsMB;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a label propagator.
   *
   * \param raycastResultSize                 The size of the raycast result (in pixels).
   * \param maxAngleBetweenNormals            The largest angle allowed between the normals of neighbouring voxels if propagation is to occur.
   * \param maxSquaredDistanceBetweenColours  The maximum squared distance allowed between the colours of neighbouring voxels if propagation is to occur.
   * \param maxSquaredDistanceBetweenVoxels   The maximum squared distance allowed between the positions of neighbouring voxels if propagation is to occur.
   */
  LabelPropagator(size_t raycastResultSize, float maxAngleBetweenNormals, float maxSquaredDistanceBetweenColours, float maxSquaredDistanceBetweenVoxels);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the label propagator.
   */
  virtual ~LabelPropagator();

  //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Calculates the surface normals of the voxels in the raycast result.
   *
   * \param raycastResult The raycast result.
   * \param scene         The scene.
   */
  virtual void calculate_normals(const ITMFloat4Image *raycastResult, const SpaintVoxelScene *scene) const = 0;

  /**
   * \brief Performs the propagation of the specified label across the scene in a device-specific way.
   *
   * \param label         The label to propagate.
   * \param raycastResult The raycast result.
   * \param scene         The scene.
   */
  virtual void perform_propagation(SpaintVoxel::Label label, const ITMFloat4Image *raycastResult, SpaintVoxelScene *scene) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Propagates the specified label across the scene, stopping at position, normal or colour discontinuities.
   *
   * \param label         The label to propagate.
   * \param raycastResult The raycast result.
   * \param scene         The scene.
   */
  void propagate_label(SpaintVoxel::Label label, const ITMFloat4Image *raycastResult, SpaintVoxelScene *scene) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<const LabelPropagator> LabelPropagator_CPtr;

}

#endif
