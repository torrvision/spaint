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
 * \brief An instance of this type represents a 3D Keypoint with an associated colour.
 */
struct Keypoint3DColour
{
  /** The keypoint position in space. */
  Vector3f position;
  /** The keypoint colour. */
  Vector3u colour;
  /** A flag representing the validity of the Keypoint. */
  bool valid;
};

/** Typedefs. */
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
  /** Offsets to address the internal union-ed arrays. */
  static const int DEPTH_OFFSET = 0;
  static const int DEPTH_FEATURE_COUNT = 128;
  static const int RGB_OFFSET = DEPTH_FEATURE_COUNT;
  static const int RGB_FEATURE_COUNT = 128;
  static const int FEATURE_SIZE = RGB_FEATURE_COUNT + DEPTH_FEATURE_COUNT;

  /** Anonymous union used to access different parts of the descriptor. */
  union
  {
    /** All the values are stored in this array. */
    float data[FEATURE_SIZE];
    struct
    {
      /** Gives access to the depth-based values of the descriptor. */
      float depth[DEPTH_FEATURE_COUNT];
      /** Gives access to the colour-based values of the descriptor. */
      float rgb[RGB_FEATURE_COUNT];
    };
  };
};

/** Typedefs. */
typedef ORUtils::Image<RGBDPatchDescriptor> RGBDPatchDescriptorImage;
typedef boost::shared_ptr<RGBDPatchDescriptorImage> RGBDPatchDescriptorImage_Ptr;
typedef boost::shared_ptr<const RGBDPatchDescriptorImage> RGBDPatchDescriptorImage_CPtr;

}

#endif
