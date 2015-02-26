/**
 * spaint: SpaintInteractor.h
 */

#ifndef H_SPAINT_SPAINTINTERACTOR
#define H_SPAINT_SPAINTINTERACTOR

#include <boost/shared_ptr.hpp>

#include "SpaintModel.h"
#include "../markers/interface/VoxelMarker.h"
#include "../selectors/Selector.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to interact with the InfiniTAM scene in an spaint model.
 */
class SpaintInteractor
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<ITMLib::Objects::ITMRenderState> RenderState_CPtr;
  typedef Selector::Selection Selection;
  typedef boost::shared_ptr<Selection> Selection_CPtr;
  typedef boost::shared_ptr<const VoxelMarker> VoxelMarker_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The spaint model. */
  SpaintModel_Ptr m_model;

  /** The selector to use for selecting voxels in the scene. */
  Selector_Ptr m_selector;

  /** The semantic label to use for manually labelling the scene. */
  unsigned char m_semanticLabel;

  /** The voxel marker (used to apply semantic labels to voxels in the scene). */
  VoxelMarker_CPtr m_voxelMarker;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an interactor that can be used to interact with the InfiniTAM scene.
   *
   * \param model The spaint model.
   */
  explicit SpaintInteractor(const SpaintModel_Ptr& model);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the selector that is being used to select voxels in the scene.
   *
   * \return  The selector that is being used to select voxels in the scene.
   */
  Selector_CPtr get_selector() const;

  /**
   * \brief Gets the semantic label that is being used for manually labelling the scene.
   *
   * \return  The semantic label that is being used for manually labelling the scene.
   */
  unsigned char get_semantic_label() const;

  /**
   * \brief Marks a selection of voxels in the scene with the specified semantic label.
   *
   * \param selection The selection of voxels.
   * \param label     The semantic label with which to mark the voxels.
   */
  void mark_voxels(const Selection_CPtr& selection, unsigned char label);

  /**
   * \brief Selects some voxels in the scene using the current selector.
   *
   * \param inputState  The current input state.
   * \param renderState The render state corresponding to a camera from which the scene is being viewed.
   * \return            The selected voxels in the scene.
   */
  Selection_CPtr select_voxels(const InputState& inputState, const RenderState_CPtr& renderState) const;

  /**
   * \brief Sets the semantic label to use for manually labelling the scene.
   *
   * \param semanticLabel The semantic label to use for manually labelling the scene.
   */
  void set_semantic_label(unsigned char semanticLabel);

  /**
   * \brief Allows the user to change the selector or update the parameters of the current selector.
   *
   * \param inputState  The current input state.
   */
  void update_selector(const InputState& inputState);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SpaintInteractor> SpaintInteractor_Ptr;
typedef boost::shared_ptr<const SpaintInteractor> SpaintInteractor_CPtr;

}

#endif
