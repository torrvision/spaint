/**
 * spaint: LabelPropagatorFactory.h
 */

#ifndef H_SPAINT_LABELPROPAGATORFACTORY
#define H_SPAINT_LABELPROPAGATORFACTORY

#include "interface/LabelPropagator.h"

namespace spaint {

/**
 * \brief This struct can be used to construct label propagators.
 */
struct LabelPropagatorFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief TODO
   */
  static LabelPropagator_CPtr make_label_propagator(size_t raycastResultSize);
};

}

#endif
