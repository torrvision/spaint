/**
 * spaint: VoxelSamplerFactory.h
 */

#ifndef H_SPAINT_VOXELSAMPLERFACTORY
#define H_SPAINT_VOXELSAMPLERFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/VoxelSampler.h"

namespace spaint {

/**
 * \brief This class can be used to construct voxel samplers.
 */
class VoxelSamplerFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Makes a voxel sampler.
   *
   * \param labelManager      The label manager.
   * \param maxVoxelsPerLabel The maximum number of voxels to sample for each label.
   * \param raycastResultSize The size of the raycast result (in pixels).
   * \param seed              The seed for the random number generator.
   * \param deviceType        The device on which the sampler should operate.
   * \return                  The voxel sampler.
   */
  static VoxelSampler_CPtr make(const LabelManager_CPtr& labelManager, int maxVoxelsPerLabel, int raycastResultSize, unsigned int seed,
                                ITMLibSettings::DeviceType deviceType);
};

}

#endif
