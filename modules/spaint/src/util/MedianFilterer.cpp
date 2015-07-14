/**
 * spaint: MedianFilterer.cpp
 */

#include "util/MedianFilterer.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

MedianFilterer::MedianFilterer(unsigned int kernelWidth)
: m_kernelWidth(kernelWidth)
{}

//#################### PUBLIC OPERATORS ####################

void MedianFilterer::operator()(const ITMUChar4Image_CPtr& input, const ITMUChar4Image_Ptr& output) const
try
{
  static ImageProcessor_CPtr imageProcessor(new ImageProcessor_CUDA);
  static boost::shared_ptr<af::array> temp(new af::array(input->noDims.y, input->noDims.x, 4, u8));
  imageProcessor->copy_itm_to_af(input, temp);
  *temp = af::medfilt(*temp, m_kernelWidth, m_kernelWidth);
  imageProcessor->copy_af_to_itm(temp, output);
}
catch(af::exception&)
{
  // Prevent crashes when median filtering fails.
}

}
