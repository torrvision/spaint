/**
 * grove: Keypoint3DColour.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_KEYPOINT3DCOLOUR
#define H_GROVE_KEYPOINT3DCOLOUR

#include <boost/shared_ptr.hpp>

#include <ORUtils/Image.h>
#include <ORUtils/Math.h>

namespace grove {

/**
 * \brief An instance of this struct represents a 3D keypoint with an associated colour.
 */
struct Keypoint3DColour
{
  //#################### PUBLIC VARIABLES ####################

  // Note: The variables here are deliberately ordered in this way to reduce padding.
  //       Assuming 32-bit floats, 8-bit unsigned chars and 8-bit bools, this order
  //       gives us instances of size 3 * 4 + 3 * 1 + 1 * 1 = 16. If we swapped the
  //       order of position and colour to make things alphabetical, we'd get larger
  //       instances of size 3 * 1 + 1 + 3 * 4 + 1 * 1 + 3 = 20.

  /** The keypoint's position in space. */
  Vector3f position;

  /** The keypoint's colour. */
  Vector3u colour;

  /** A flag indicating whether or not the keypoint is valid. */
  bool valid;
};

//#################### TYPEDEFS ####################

typedef ORUtils::Image<Keypoint3DColour> Keypoint3DColourImage;
typedef boost::shared_ptr<Keypoint3DColourImage> Keypoint3DColourImage_Ptr;
typedef boost::shared_ptr<const Keypoint3DColourImage> Keypoint3DColourImage_CPtr;

}

#endif
