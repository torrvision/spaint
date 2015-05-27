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
  this->add_generator(DecisionFunctionGenerator_CPtr(new FeatureThresholdingDecisionFunctionGenerator<SpaintVoxel::LabelType>(vopFeatureIndexRange)));
  this->add_generator(DecisionFunctionGenerator_CPtr(new PairwiseOpAndThresholdDecisionFunctionGenerator<SpaintVoxel::LabelType>(vopFeatureIndexRange)));
  this->add_generator(DecisionFunctionGenerator_CPtr(new FeatureThresholdingDecisionFunctionGenerator<SpaintVoxel::LabelType>(std::make_pair(vopFeatureCount, vopFeatureCount + 2))));
  this->add_generator(DecisionFunctionGenerator_CPtr(new FeatureThresholdingDecisionFunctionGenerator<SpaintVoxel::LabelType>(std::make_pair(vopFeatureCount + 3, vopFeatureCount + 3))));
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

SpaintDecisionFunctionGenerator::DecisionFunctionGenerator_Ptr SpaintDecisionFunctionGenerator::maker(size_t patchSize)
{
  return DecisionFunctionGenerator_Ptr(new SpaintDecisionFunctionGenerator(patchSize));
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
