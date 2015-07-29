/**
 * spaint: TouchDescriptorCalculator.cpp
 */

#include "touch/TouchDescriptorCalculator.h"
using namespace rafl;

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

Descriptor_CPtr TouchDescriptorCalculator::calculate_histogram_descriptor(const af::array& img)
{
  // Calculate a histogram from the image using ArrayFire.
  const unsigned int binCount = 64;
  const double minVal = 0.0;
  const double maxVal = 255.0;
  af::array afHistogram = af::histogram(img, binCount, minVal, maxVal);

  // Copy it across to a rafl feature descriptor.
  const float *afHistogramPtr = afHistogram.as(f32).host<float>();
  return Descriptor_CPtr(new Descriptor(afHistogramPtr, afHistogramPtr + binCount));
}

}
