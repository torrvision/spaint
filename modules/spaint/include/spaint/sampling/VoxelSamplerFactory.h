/**
 * spaint: VoxelSamplerFactory.h
 */

#ifndef H_SPAINT_VOXELSAMPLERFACTORY
#define H_SPAINT_VOXELSAMPLERFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/PerLabelVoxelSampler.h"
#include "interface/UniformVoxelSampler.h"

namespace spaint {

/**
 * \brief This class can be used to construct voxel samplers.
 */
class VoxelSamplerFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Makes a per-label voxel sampler.
   *
   * \param maxLabelCount     The maximum number of labels that can be in use.
   * \param maxVoxelsPerLabel The maximum number of voxels to sample for each label.
   * \param raycastResultSize The size of the raycast result (in pixels).
   * \param seed              The seed for the random number generator.
   * \param deviceType        The device on which the sampler should operate.
   * \return                  The voxel sampler.
   */
  static VoxelSampler_CPtr make_per_label_sampler(size_t maxLabelCount, size_t maxVoxelsPerLabel, int raycastResultSize, unsigned int seed,
                                                  ITMLibSettings::DeviceType deviceType);

  /**
   * \brief Makes a uniform voxel sampler.
   *
   * \param raycastResultSize The size of the raycast result (in pixels).
   * \param seed              The seed for the random number generator.
   * \param deviceType        The device on which the sampler should operate.
   * \return                  The voxel sampler.
   */
  static UniformVoxelSampler_CPtr make_uniform_sampler(int raycastResultSize, unsigned int seed, ITMLibSettings::DeviceType deviceType);
};

}

#endif
