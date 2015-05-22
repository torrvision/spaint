/**
 * spaint: SpaintDecisionFunctionGenerator.h
 */

#ifndef H_SPAINT_SPAINTDECISIONFUNCTIONGENERATOR
#define H_SPAINT_SPAINTDECISIONFUNCTIONGENERATOR

#include <rafl/decisionfunctions/CompositeDecisionFunctionGenerator.h>

#include "SpaintVoxel.h"

namespace spaint {

/**
 * \brief TODO
 */
class SpaintDecisionFunctionGenerator : public rafl::CompositeDecisionFunctionGenerator<SpaintVoxel::LabelType>
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   *
   * \param patchSize The side length of a VOP patch.
   */
  explicit SpaintDecisionFunctionGenerator(size_t patchSize);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the type of the decision function generator.
   *
   * \return  The type of the decision function generator.
   */
  static std::string get_static_type();

  /** Override */
  virtual std::string get_type() const;
};

}

#endif
