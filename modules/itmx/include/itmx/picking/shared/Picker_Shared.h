/**
 * itmx: Picker_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_ITMX_PICKER_SHARED
#define H_ITMX_PICKER_SHARED

#include <ORUtils/Math.h>

namespace itmx {

/**
 * \brief Gets the scene point (if any) that would be picked by clicking at a specific location on the image plane.
 *
 * \param x           The x coordinate of the point clicked.
 * \param y           The y coordinate of the point clicked.
 * \param width       The width of the viewing area on the image plane.
 * \param pointImage  An image specifying the scene points that would be picked for every pixel on the image plane.
 * \param pickPoint   A place into which to store the scene point (if any) that would be picked.
 * \return            true, if clicking on the specified location picked a point, or false otherwise.
 */
_CPU_AND_GPU_CODE_
inline bool get_pick_point(int x, int y, int width, const Vector4f *pointImage, Vector3f& pickPoint)
{
  Vector4f p = pointImage[y * width + x];
  pickPoint = Vector3f(p.x, p.y, p.z);
  return p.w > 0;
}

}

#endif
