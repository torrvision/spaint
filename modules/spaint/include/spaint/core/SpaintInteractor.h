/**
 * spaint: SpaintInteractor.h
 */

#ifndef H_SPAINT_SPAINTINTERACTOR
#define H_SPAINT_SPAINTINTERACTOR

#include <boost/shared_ptr.hpp>

#include "SpaintModel.h"
#include "../marking/interface/VoxelMarker.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to interact with the InfiniTAM scene in an spaint model.
 */
class SpaintInteractor
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const VoxelMarker> VoxelMarker_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The spaint model. */
  SpaintModel_Ptr m_model;

  /** The voxel marker (used to apply semantic labels to voxels in the scene). */
  VoxelMarker_CPtr m_voxelMarker;

  //#################### CONSTRUCTORS ####################
public:
  explicit SpaintInteractor(const SpaintModel_Ptr& model);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Marks a set of voxels in the scene with the specified semantic label.
   *
   * \param voxelLocationsMB  A memory block containing the locations of the voxels in the scene.
   * \param label             The semantic label with which to mark the voxels.
   */
  void mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, unsigned char label);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SpaintInteractor> SpaintInteractor_Ptr;
typedef boost::shared_ptr<const SpaintInteractor> SpaintInteractor_CPtr;

}

#endif
