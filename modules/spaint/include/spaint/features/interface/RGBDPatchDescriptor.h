/**
 * spaint: RGBDPatchDescriptor.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_RGBDPATCHDESCRIPTOR
#define H_SPAINT_RGBDPATCHDESCRIPTOR

#include <boost/shared_ptr.hpp>

#include <ITMLib/Utils/ITMMath.h>

#include <ORUtils/Image.h>

namespace spaint
{

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
