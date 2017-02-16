/**
 * spaint: RGBDPatchFeature.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_RGBDPATCHFEATURE
#define H_SPAINT_RGBDPATCHFEATURE

#include <boost/shared_ptr.hpp>

#include <ITMLib/Utils/ITMMath.h>

#include <ORUtils/Image.h>

namespace spaint
{

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

  /** A flag representing the validity of the keypoint. */
  bool valid;
};

//#################### TYPEDEFS ####################

typedef ORUtils::Image<Keypoint3DColour> Keypoint3DColourImage;
typedef boost::shared_ptr<Keypoint3DColourImage> Keypoint3DColourImage_Ptr;
typedef boost::shared_ptr<const Keypoint3DColourImage> Keypoint3DColourImage_CPtr;

/**
 * \brief An instance of this type represents a Descriptor for an RGBD patch of an image.
 *
 *        Can be computed as described in:
 *        "Exploiting uncertainty in regression forests for accurate camera relocalization"
 *        by Valentin et al.
 */
struct RGBDPatchDescriptor
{
  //#################### CONSTANTS ####################

  /** Offsets to address the internal union-ed arrays. */
  static const int DEPTH_OFFSET = 0;
  static const int DEPTH_FEATURE_COUNT = 128;
  static const int RGB_OFFSET = DEPTH_FEATURE_COUNT;
  static const int RGB_FEATURE_COUNT = 128;
  static const int FEATURE_COUNT = RGB_FEATURE_COUNT + DEPTH_FEATURE_COUNT;

  //#################### PUBLIC VARIABLES ####################

  /** An anonymous union used to access different parts of the descriptor. */
  union
  {
    /** All the features are stored in this array. */
    float data[FEATURE_COUNT];

    struct
    {
      /** Gives access to the depth-based features in the descriptor. */
      float depth[DEPTH_FEATURE_COUNT];

      /** Gives access to the colour-based features in the descriptor. */
      float rgb[RGB_FEATURE_COUNT];
    };
  };
};

//#################### TYPEDEFS ####################

typedef ORUtils::Image<RGBDPatchDescriptor> RGBDPatchDescriptorImage;
typedef boost::shared_ptr<RGBDPatchDescriptorImage> RGBDPatchDescriptorImage_Ptr;
typedef boost::shared_ptr<const RGBDPatchDescriptorImage> RGBDPatchDescriptorImage_CPtr;

}

#endif
