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
  /** TODO */
  boost::shared_ptr<ORUtils::MemoryBlock<Vector3f> > m_normals;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a label propagator.
   *
   * \param raycastResultSize The size of the raycast result (in pixels).
   */
  explicit LabelPropagator(size_t raycastResultSize);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  void propagate_label(SpaintVoxel::Label label, const ITMFloat4Image *raycastResult,
                       ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const;
};

}

#endif
