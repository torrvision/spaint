/**
 * spaint: TouchDescriptorCalculator.h
 */

#ifndef H_SPAINT_TOUCHDESCRIPTORCALCULATOR
#define H_SPAINT_TOUCHDESCRIPTORCALCULATOR

#include <arrayfire.h>

#include <rafl/base/Descriptor.h>

namespace spaint {

/**
 * \brief This struct provides a function that allows us to calculate histogram descriptors for images that contain candidate touch components.
 *
 * These are used in conjunction with a random forest that predicts the likelihood of candidate touch components being valid.
 */
struct TouchDescriptorCalculator
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Calculates a global histogram descriptor for an image that contains candidate touch components.
   *
   * \param img The image for which to calculate the descriptor.
   * \return    The descriptor.
   */
  static rafl::Descriptor_CPtr calculate_histogram_descriptor(const af::array& img);
};

}

#endif
