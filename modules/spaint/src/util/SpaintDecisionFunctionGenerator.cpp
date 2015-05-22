/**
 * spaint: SpaintDecisionFunctionGenerator.cpp
 */

#include "util/SpaintDecisionFunctionGenerator.h"

#include <rafl/decisionfunctions/FeatureThresholdingDecisionFunctionGenerator.h>
#include <rafl/decisionfunctions/PairwiseOpAndThresholdDecisionFunctionGenerator.h>
using namespace rafl;

namespace spaint {

//#################### CONSTRUCTORS ####################

SpaintDecisionFunctionGenerator::SpaintDecisionFunctionGenerator(size_t patchSize)
{
  int vopFeatureCount = static_cast<int>(patchSize * patchSize * 3);
  std::pair<int,int> vopFeatureIndexRange(0, vopFeatureCount - 1);
  add_generator(DecisionFunctionGenerator_CPtr(new FeatureThresholdingDecisionFunctionGenerator<SpaintVoxel::LabelType>(vopFeatureIndexRange)));
  add_generator(DecisionFunctionGenerator_CPtr(new PairwiseOpAndThresholdDecisionFunctionGenerator<SpaintVoxel::LabelType>(vopFeatureIndexRange)));
  add_generator(DecisionFunctionGenerator_CPtr(new FeatureThresholdingDecisionFunctionGenerator<SpaintVoxel::LabelType>(std::make_pair(vopFeatureCount, vopFeatureCount + 2))));
  add_generator(DecisionFunctionGenerator_CPtr(new FeatureThresholdingDecisionFunctionGenerator<SpaintVoxel::LabelType>(std::make_pair(vopFeatureCount + 3, vopFeatureCount + 3))));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

std::string SpaintDecisionFunctionGenerator::get_static_type()
{
  return "Spaint";
}

std::string SpaintDecisionFunctionGenerator::get_type() const
{
  return get_static_type();
}

}
