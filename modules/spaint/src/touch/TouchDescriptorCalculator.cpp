/**
 * spaint: TouchDescriptorCalculator.cpp
 */

#include "touch/TouchDescriptorCalculator.h"

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

rafl::Descriptor_CPtr TouchDescriptorCalculator::histogram(const af::array& img)
{
  af::array globalHistogram = af::histogram(img, 64, 0, 255);
  const float *gHist = globalHistogram.as(f32).host<float>();
  const int gHistLen = globalHistogram.dims(0);
  return rafl::Descriptor_CPtr(new rafl::Descriptor(gHist, gHist + gHistLen));
}

}
