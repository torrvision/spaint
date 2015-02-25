/**
 * spaint: PickingSelector.h
 */

#ifndef H_SPAINT_PICKINGSELECTOR
#define H_SPAINT_PICKINGSELECTOR

#include <boost/optional.hpp>

#include <ITMLib/Utils/ITMLibSettings.h>

#include "Selector.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to select a cube of voxels in the scene using picking.
 */
class PickingSelector : public Selector
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<ITMFloat4Image> Float4Image_Ptr;
  typedef boost::shared_ptr<const ITMLibSettings> Settings_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** TODO */
  mutable boost::optional<Vector3f> m_pickPoint;

  /** The selection radius (we select all voxels in a cube of side length 2 * radius + 1, centered on the voxel the user actually clicks). */
  int m_radius;

  /** An image into which the raycast result may be copied when performing picking. */
  mutable Float4Image_Ptr m_raycastResult;

  /** The settings to use for InfiniTAM. */
  Settings_CPtr m_settings;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a picking selector.
   *
   * \param settings  The settings to use for InfiniTAM.
   */
  explicit PickingSelector(const Settings_CPtr& settings);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Determines the nearest scene point (if any) that would be hit by a ray cast through (x,y) on the image plane
   *        when viewed from the camera pose with the specified render state.
   *
   * \param x           The x coordinate of the point on the image plane through which the ray is cast.
   * \param y           The y coordinate of the point on the image plane through which the ray is cast.
   * \param renderState The render state corresponding to a camera pose.
   * \return            The coordinates of the nearest scene point (if any) that is hit by the ray.
   */
  boost::optional<Vector3f> pick(int x, int y, const RenderState_CPtr& renderState) const;

  /**
   * \brief Gets the selection radius.
   *
   * \return  The selection radius.
   */
  int radius() const;

  /** Override */
  virtual Selection_CPtr select_voxels(const InputState& inputState, const RenderState_CPtr& renderState) const;

  /** Override */
  virtual void update(const InputState& inputState);
};

}

#endif
