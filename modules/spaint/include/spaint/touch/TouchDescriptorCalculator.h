/**
 * spaint: TouchDescriptorCalculator.h
 */

#ifndef H_SPAINT_TOUCHDESCRIPTORCALCULATOR
#define H_SPAINT_TOUCHDESCRIPTORCALCULATOR

#include <arrayfire.h>

#include <rafl/core/RandomForest.h>

namespace spaint {

/**
 * \brief This class contains functions that help us to extract descriptors from candidate components.
 */
class TouchDescriptorCalculator
{
//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Calculates a global histogram descriptor from an image.
   *
   * \param img  The image from which to calculate a histogram descriptor.
   * \return     The histogram descriptor.
   */
  static rafl::Descriptor_CPtr histogram(const af::array& img);
};

}

#endif
