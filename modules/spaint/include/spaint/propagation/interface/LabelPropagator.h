/**
 * spaint: LabelPropagator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_LABELPROPAGATOR
#define H_SPAINT_LABELPROPAGATOR

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/ITMScene.h>

#include "../../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to propagate labels across surfaces in the scene.
 */
class LabelPropagator
{
  //#################### PROTECTED VARIABLES ####################
protected:
  /** The largest angle allowed between the normals of neighbouring voxels if extrapolation is to occur. */
  const float m_maxAngleBetweenNormals;

  /** The maximum number of labels that can be in use. */
  const size_t m_maxLabelCount;

  /** The maximum squared distance allowed between the colours of neighbouring voxels if extrapolation is to occur. */
  const float m_maxSquaredDistanceBetweenColours;

  /** The maximum squared distance allowed between the positions of neighbouring voxels if extrapolation or interpolation are to occur. */
  const float m_maxSquaredDistanceBetweenVoxels;

  /** A memory block in which to store the surface normals of the voxels in the raycast result. */
  boost::shared_ptr<ORUtils::MemoryBlock<Vector3f> > m_surfaceNormalsMB;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a label propagator.
   *
   * \param raycastResultSize                 The size of the raycast result (in pixels).
   * \param maxLabelCount                     The maximum number of labels that can be in use.
   * \param maxAngleBetweenNormals            The largest angle allowed between the normals of neighbouring voxels if extrapolation is to occur.
   * \param maxSquaredDistanceBetweenColours  The maximum squared distance allowed between the colours of neighbouring voxels if extrapolation is to occur.
   * \param maxSquaredDistanceBetweenVoxels   The maximum squared distance allowed between the positions of neighbouring voxels if extrapolation or interpolation are to occur.
   */
  LabelPropagator(size_t raycastResultSize, size_t maxLabelCount, float maxAngleBetweenNormals, float maxSquaredDistanceBetweenColours, float maxSquaredDistanceBetweenVoxels);

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Interpolates between existing labelled voxels in the scene, filling in the labels of voxels based on their neighbours.
   *
   * \param raycastResult The raycast result.
   * \param scene         The scene.
   */
  virtual void interpolate_labels(const ITMFloat4Image *raycastResult, ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const = 0;

  //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Calculates the surface normals of the voxels in the raycast result.
   *
   * \param raycastResult The raycast result.
   * \param scene         The scene.
   */
  virtual void calculate_normals(const ITMFloat4Image *raycastResult, const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const = 0;

  /**
   * \brief Performs the extrapolation of the specified label across the scene in a device-specific way.
   *
   * \param label         The label to extrapolate.
   * \param raycastResult The raycast result.
   * \param scene         The scene.
   */
  virtual void perform_extrapolation(SpaintVoxel::Label label, const ITMFloat4Image *raycastResult, ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Extrapolates the specified label across the scene, stopping at position, normal or colour discontinuities.
   *
   * \param label         The label to extrapolate.
   * \param raycastResult The raycast result.
   * \param scene         The scene.
   */
  void extrapolate_label(SpaintVoxel::Label label, const ITMFloat4Image *raycastResult, ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<const LabelPropagator> LabelPropagator_CPtr;

}

#endif
