/**
 * spaint: VisualisationGeneratorFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_SPAINT_VISUALISATIONGENERATORFACTORY
#define H_SPAINT_VISUALISATIONGENERATORFACTORY

#include "VisualisationGenerator.h"

namespace spaint {

/**
 * \brief This struct can be used to construct visualisation generators.
 */
struct VisualisationGeneratorFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a visualisation generator.
   *
   * \param voxelVisualisationEngine  The InfiniTAM engine used for rendering a voxel scene.
   * \param surfelVisualisationEngine The InfinITAM engine used for rendering a surfel scene.
   * \param labelManager              The label manager.
   * \param settings                  The settings to use for InfiniTAM.
   *
   * \return A visualisation generator.
   */
  static VisualisationGenerator_Ptr make_visualisation_generator(const VoxelVisualisationEngine_CPtr& voxelVisualisationEngine,
                                                                 const SurfelVisualisationEngine_CPtr& surfelVisualisationEngine,
                                                                 const LabelManager_CPtr& labelManager,
                                                                 const Settings_CPtr& settings);

  /**
   * \brief Makes a visualisation generator.
   *
   * \param settings       The settings to use for InfiniTAM.
   * \param maxLabelCount  The maximum number of labels handled by the visualisation generator (0 = no semantic visualisation).
   *
   * \return  A visualisation generator.
   */
  static VisualisationGenerator_Ptr make_visualisation_generator(const Settings_CPtr &settings, size_t maxLabelCount = 0);
};

}

#endif
