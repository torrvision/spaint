/**
 * grove: RGBDPatchFeatureDifferenceType.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_RGBDPATCHFEATUREDIFFERENCETYPE
#define H_GROVE_RGBDPATCHFEATUREDIFFERENCETYPE

namespace grove {

/**
 * \brief The values of this enumeration can be used to specify the type of difference features to use.
 *
 * See Section 3.1 in "Exploiting Uncertainty in Regression Forests for Accurate Camera Relocalization".
 */
enum RGBDPatchFeatureDifferenceType
{
  /** With a central difference scheme, each difference is computed between the central pixel and a pixel at a random offset. */
  CENTRAL_DIFFERENCE,

  /** With a pairwise difference scheme, each difference is computed between two pixels at random offsets from the central pixel. */
  PAIRWISE_DIFFERENCE
};

}

#endif
