/**
 * spaint: LabelPropagator.h
 */

#ifndef H_SPAINT_LABELPROPAGATOR
#define H_SPAINT_LABELPROPAGATOR

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/ITMScene.h>

#include "../../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to propagate a specified label across surfaces in the scene.
 */
class LabelPropagator
{
  //#################### PROTECTED VARIABLES ####################
protected:
  /** A memory block in which to store the surface normals of the voxels in the raycast result. */
  boost::shared_ptr<ORUtils::MemoryBlock<Vector3f> > m_surfaceNormalsMB;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a label propagator.
   *
   * \param raycastResultSize The size of the raycast result (in pixels).
   */
  explicit LabelPropagator(size_t raycastResultSize);

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
   * \brief Performs the propagation of the specified label across the scene in a device-specific way.
   *
   * \param label         The label to propagate.
   * \param raycastResult The raycast result.
   * \param scene         The scene.
   */
  virtual void perform_propagation(SpaintVoxel::Label label, const ITMFloat4Image *raycastResult,
                                   ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Propagates the specified label across the scene, stopping at position, normal or colour discontinuities.
   *
   * \param label         The label to propagate.
   * \param raycastResult The raycast result.
   * \param scene         The scene.
   */
  void propagate_label(SpaintVoxel::Label label, const ITMFloat4Image *raycastResult,
                       ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<const LabelPropagator> LabelPropagator_CPtr;

}

#endif
