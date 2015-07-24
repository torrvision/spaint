/**
 * spaint: FeatureCalculatorFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_FEATURECALCULATORFACTORY
#define H_SPAINT_FEATURECALCULATORFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/FeatureCalculator.h"

namespace spaint {

/**
 * \brief This struct can be used to construct feature calculators.
 */
struct FeatureCalculatorFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a VOP feature calculator.
   *
   * \param maxVoxelLocationCount The maximum number of voxel locations for which we will be calculating features at any one time.
   * \param patchSize             The side length of a VOP patch (must be odd).
   * \param patchSpacing          The spacing in the scene (in voxels) between individual pixels in a patch.
   * \param binCount              The number of bins into which to quantize orientations when aligning voxel patches.
   * \param deviceType            The device on which the feature calculator should operate.
   */
  static FeatureCalculator_CPtr make_vop_feature_calculator(size_t maxVoxelLocationCount, size_t patchSize, float patchSpacing, size_t binCount,
                                                            ITMLibSettings::DeviceType deviceType);
};

}

#endif
