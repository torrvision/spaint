/**
 * spaint: FeatureCalculatorFactory.h
 */

#ifndef H_SPAINT_FEATURECALCULATORFACTORY
#define H_SPAINT_FEATURECALCULATORFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/FeatureCalculator.h"

namespace spaint {

/**
 * \brief This class can be used to construct feature calculators.
 */
class FeatureCalculatorFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Makes a VOP feature calculator.
   *
   * \param maxLabelCount     The maximum number of labels that can be in use.
   * \param maxVoxelsPerLabel The maximum number of voxels that have been sampled for each label.
   * \param patchSize         The side length of a VOP patch (must be odd).
   * \param patchSpacing      The spacing in the scene between individual pixels in a patch.
   * \param deviceType        The device on which the feature calculator should operate.
   */
  static FeatureCalculator_CPtr make_vop_feature_calculator(size_t maxLabelCount, size_t maxVoxelsPerLabel, size_t patchSize, float patchSpacing,
                                                            ITMLibSettings::DeviceType deviceType);
};

}

#endif
