/**
 * spaint: LabelPropagatorFactory.h
 */

#ifndef H_SPAINT_LABELPROPAGATORFACTORY
#define H_SPAINT_LABELPROPAGATORFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/LabelPropagator.h"

namespace spaint {

/**
 * \brief This struct can be used to construct label propagators.
 */
struct LabelPropagatorFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a label propagator.
   *
   * \param raycastResultSize The size of the raycast result (in pixels).
   * \param deviceType        The device on which the label propagator should operate.
   * \return                  The label propagator.
   */
  static LabelPropagator_CPtr make_label_propagator(size_t raycastResultSize, ITMLibSettings::DeviceType deviceType);
};

}

#endif
