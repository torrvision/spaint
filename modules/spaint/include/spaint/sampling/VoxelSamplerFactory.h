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
   * \param maxLabelCount     The maximum number of labels that can be in use.
   * \param maxVoxelsPerLabel The maximum number of voxels to sample for each label.
   * \param raycastResultSize The size of the raycast result (in pixels).
   * \param seed              The seed for the random number generator.
   * \param deviceType        The device on which the sampler should operate.
   * \return                  The voxel sampler.
   */
  static VoxelSampler_CPtr make(size_t maxLabelCount, size_t maxVoxelsPerLabel, int raycastResultSize, unsigned int seed, ITMLibSettings::DeviceType deviceType);
};

}

#endif
