/**
 * spaint: TouchDescriptorCalculator.h
 */

#ifndef H_SPAINT_TOUCHDESCRIPTORCALCULATOR
#define H_SPAINT_TOUCHDESCRIPTORCALCULATOR

#ifdef _MSC_VER
  // Suppress a VC++ warning that is produced when including the ArrayFire header.
  #pragma warning(disable:4275)
#endif

#include <arrayfire.h>

#ifdef _MSC_VER
  // Reenable the suppressed warning for the rest of the translation unit.
  #pragma warning(default:4275)
#endif

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
