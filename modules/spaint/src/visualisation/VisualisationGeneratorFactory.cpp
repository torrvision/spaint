/**
 * spaint: VisualisationGeneratorFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "visualisation/VisualisationGeneratorFactory.h"

#include <ITMLib/Engines/Visualisation/ITMSurfelVisualisationEngineFactory.h>
#include <ITMLib/Engines/Visualisation/ITMVisualisationEngineFactory.h>
using namespace ITMLib;

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

VisualisationGenerator_Ptr VisualisationGeneratorFactory::make_visualisation_generator(
    const VoxelVisualisationEngine_CPtr &voxelVisualisationEngine, const SurfelVisualisationEngine_CPtr &surfelVisualisationEngine,
    const LabelManager_CPtr &labelManager, const Settings_CPtr &settings)
{
  return VisualisationGenerator_Ptr(new VisualisationGenerator(voxelVisualisationEngine, surfelVisualisationEngine, labelManager, settings));
}

VisualisationGenerator_Ptr VisualisationGeneratorFactory::make_visualisation_generator(const Settings_CPtr &settings, size_t maxLabelCount)
{
  // Create default engines.
  VoxelVisualisationEngine_CPtr voxelVisualisationEngine(ITMVisualisationEngineFactory::MakeVisualisationEngine<SpaintVoxel,ITMVoxelIndex>(settings->deviceType));
  SurfelVisualisationEngine_CPtr surfelVisualisationEngine(ITMSurfelVisualisationEngineFactory<SpaintSurfel>::make_surfel_visualisation_engine(settings->deviceType));
  LabelManager_CPtr labelManager(maxLabelCount > 0 ? new LabelManager(maxLabelCount) : NULL);

  return make_visualisation_generator(voxelVisualisationEngine, surfelVisualisationEngine, labelManager, settings);
}

}
