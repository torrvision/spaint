/**
 * spaint: RelocaliserFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2019. All rights reserved.
 */

#include "relocalisation/RelocaliserFactory.h"

#ifdef WITH_GROVE
#include <grove/relocalisation/ScoreRelocaliserFactory.h>
using namespace grove;
#endif

#include <itmx/relocalisation/FernRelocaliser.h>
using namespace itmx;

#include <orx/relocalisation/BackgroundRelocaliser.h>
#include <orx/relocalisation/CascadeRelocaliser.h>
#include <orx/relocalisation/EnsembleRelocaliser.h>
#include <orx/relocalisation/NullRelocaliser.h>
using namespace orx;

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

Relocaliser_Ptr RelocaliserFactory::make_relocaliser(std::string relocaliserType, const Vector2i& depthImageSize, bool relocaliseEveryFrame,
                                                     const Refiner& refineWithICP, const GroundTruthProvider& getGroundTruthTrajectory,
                                                     const Settings_CPtr& settings)
{
#ifndef WITH_GROVE
  // If we're trying to use a Grove relocaliser and Grove has not been built, fall back to the ferns relocaliser and issue a warning.
  if(relocaliserType == "cascade" || relocaliserType == "ensemble" || relocaliserType == "forest" || relocaliserType == "gt" || relocaliserType == "net")
  {
    relocaliserType = "ferns";
    std::cerr << "Warning: Cannot use a Grove relocaliser because BUILD_GROVE is disabled in CMake. Falling back to random ferns.\n";
  }
#endif

  // Construct a relocaliser of the specified type.
  if(relocaliserType == "cascade")
  {
  #ifdef WITH_GROVE
    // Look up the number of inner SCoRe relocalisers to instantiate.
    const std::string relocaliserNamespace = "CascadeRelocaliser.";
    const int innerRelocaliserCount = settings->get_first_value<int>(relocaliserNamespace + "innerRelocaliserCount");

    // Construct the inner relocalisers.
    ScoreRelocaliser_Ptr primaryRelocaliser;
    std::vector<Relocaliser_Ptr> innerRelocalisers(innerRelocaliserCount);
    for(int i = innerRelocaliserCount - 1; i >= 0; --i)
    {
      const std::string innerRelocaliserNamespace = relocaliserNamespace + "R" + boost::lexical_cast<std::string>(i) + ".";
      const std::string innerRelocaliserType = settings->get_first_value<std::string>(innerRelocaliserNamespace + "relocaliserType", "forest");
      ScoreRelocaliser_Ptr innerRelocaliser = ScoreRelocaliserFactory::make_score_relocaliser(innerRelocaliserType, innerRelocaliserNamespace, settings, settings->deviceType);

      if(i == innerRelocaliserCount - 1) primaryRelocaliser = innerRelocaliser;
      else innerRelocaliser->set_backing_relocaliser(primaryRelocaliser);

      innerRelocalisers[i] = refineWithICP(innerRelocaliser);
    }

    // Construct the cascade relocaliser itself.
    return Relocaliser_Ptr(new CascadeRelocaliser(innerRelocalisers, settings, relocaliserNamespace));
  #else
    // This should never happen.
    throw std::runtime_error("Error: Cannot construct a Grove relocaliser when BUILD_GROVE is disabled in CMake.");
  #endif
  }
  else if(relocaliserType == "ensemble")
  {
  #ifdef WITH_GROVE
    // Look up the number of inner SCoRe relocalisers to instantiate.
    const std::string relocaliserNamespace = "EnsembleRelocaliser.";
    const int innerRelocaliserCount = settings->get_first_value<int>(relocaliserNamespace + "innerRelocaliserCount");

    Relocaliser_Ptr ensembleRelocaliser;

    int deviceCount = 1;
    cudaGetDeviceCount(&deviceCount);
    if(deviceCount > 1) ORcudaSafeCall(cudaSetDevice(1));

    // Construct the ensemble relocaliser.
    std::vector<Relocaliser_Ptr> innerRelocalisers(innerRelocaliserCount);
    for(int i = 0; i < innerRelocaliserCount; ++i)
    {
      const std::string innerRelocaliserNamespace = relocaliserNamespace + "R" + boost::lexical_cast<std::string>(i) + ".";
      const std::string innerRelocaliserType = settings->get_first_value<std::string>(innerRelocaliserNamespace + "relocaliserType", "net");
      innerRelocalisers[i] = make_simple_relocaliser(
        innerRelocaliserType, innerRelocaliserNamespace, depthImageSize, relocaliseEveryFrame, refineWithICP, getGroundTruthTrajectory, settings
      );
    }

    ensembleRelocaliser.reset(new EnsembleRelocaliser(innerRelocalisers));

    Relocaliser_Ptr innerRelocaliser;
    if(deviceCount > 1)
    {
      innerRelocaliser.reset(new BackgroundRelocaliser(ensembleRelocaliser, 1));
      ORcudaSafeCall(cudaSetDevice(0));
    }
    else innerRelocaliser = ensembleRelocaliser;

    return refineWithICP(innerRelocaliser);
  #else
    // This should never happen.
    throw std::runtime_error("Error: Cannot construct a Grove relocaliser when BUILD_GROVE is disabled in CMake.");
  #endif
  }
  else
  {
    return make_simple_relocaliser(relocaliserType, "ScoreRelocaliser.", depthImageSize, relocaliseEveryFrame, refineWithICP, getGroundTruthTrajectory, settings);
  }
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

Relocaliser_Ptr RelocaliserFactory::make_simple_relocaliser(const std::string& relocaliserType, const std::string& relocaliserNamespace, const Vector2i& depthImageSize,
                                                            bool relocaliseEveryFrame, const Refiner& refineWithICP, const GroundTruthProvider& getGroundTruthTrajectory,
                                                            const Settings_CPtr& settings)
{
  // First construct a relocaliser of the specified type.
  Relocaliser_Ptr innerRelocaliser;

  if(relocaliserType == "forest" || relocaliserType == "gt" || relocaliserType == "net")
  {
  #ifdef WITH_GROVE
    // Load the relocaliser from the specified file.
    int deviceCount = 1;
    cudaGetDeviceCount(&deviceCount);
    if(deviceCount > 1) ORcudaSafeCall(cudaSetDevice(1));

    ScoreRelocaliser_Ptr scoreRelocaliser = ScoreRelocaliserFactory::make_score_relocaliser(relocaliserType, relocaliserNamespace, settings, settings->deviceType);

    if(deviceCount > 1)
    {
      innerRelocaliser.reset(new BackgroundRelocaliser(scoreRelocaliser, 1));
      ORcudaSafeCall(cudaSetDevice(0));
    }
    else innerRelocaliser = scoreRelocaliser;

    // If necessary, supply the ground truth trajectory to the relocaliser.
    if(relocaliserType == "gt" ||
       settings->get_first_value<bool>(relocaliserNamespace + "useGroundTruthTrajectory", false) ||
       settings->get_first_value<bool>(relocaliserNamespace + "makeGroundTruthPointsImage", false))
    {
      scoreRelocaliser->set_ground_truth_trajectory(getGroundTruthTrajectory());
    }
  #endif
  }
  else if(relocaliserType == "ferns")
  {
    innerRelocaliser.reset(new FernRelocaliser(
      depthImageSize,
      settings->sceneParams.viewFrustum_min,
      settings->sceneParams.viewFrustum_max,
      FernRelocaliser::get_default_harvesting_threshold(),
      FernRelocaliser::get_default_num_ferns(),
      FernRelocaliser::get_default_num_decisions_per_fern(),
      relocaliseEveryFrame ? FernRelocaliser::ALWAYS_TRY_ADD : FernRelocaliser::DELAY_AFTER_RELOCALISATION
    ));
  }
  else if(relocaliserType == "none")
  {
    innerRelocaliser.reset(new NullRelocaliser);
  }
  else throw std::runtime_error("Error: Unknown relocaliser type '" + relocaliserType + "'");

  // Then decorate this relocaliser with one that uses an ICP tracker to refine the results.
  return refineWithICP(innerRelocaliser);
}

}
