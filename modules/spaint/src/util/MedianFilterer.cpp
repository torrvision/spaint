/**
 * spaint: MedianFilterer.cpp
 */

#include "util/MedianFilterer.h"

#include "imageprocessing/ImageProcessorFactory.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

MedianFilterer::MedianFilterer(unsigned int kernelWidth, ITMLibSettings::DeviceType deviceType)
: m_imageProcessor(ImageProcessorFactory::make_image_processor(deviceType)),
  m_kernelWidth(kernelWidth)
{}

//#################### PUBLIC OPERATORS ####################

void MedianFilterer::operator()(const ITMUChar4Image_CPtr& input, const ITMUChar4Image_Ptr& output) const
try
{
  static boost::shared_ptr<af::array> temp(new af::array(input->noDims.y, input->noDims.x, 4, u8));
  m_imageProcessor->copy_itm_to_af(input, temp);
  *temp = af::medfilt(*temp, m_kernelWidth, m_kernelWidth);
  m_imageProcessor->copy_af_to_itm(temp, output);
}
catch(af::exception&)
{
  // Prevent crashes when median filtering fails.
}

}
