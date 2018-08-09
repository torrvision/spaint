/**
 * spaint: MedianFilterer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "imageprocessing/MedianFilterer.h"

#include "imageprocessing/ImageProcessorFactory.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

MedianFilterer::MedianFilterer(unsigned int kernelWidth, DeviceType deviceType)
: m_imageProcessor(ImageProcessorFactory::make_image_processor(deviceType)),
  m_kernelWidth(kernelWidth)
{}

//#################### PUBLIC OPERATORS ####################

void MedianFilterer::operator()(const ITMUChar4Image_CPtr& input, const ITMUChar4Image_Ptr& output) const
try
{
  if(!m_intermediate || ImageProcessor::image_size(input) != ImageProcessor::image_size(m_intermediate))
  {
    m_intermediate.reset(new af::array(input->noDims.y, input->noDims.x, 4, u8));
  }

  m_imageProcessor->copy_itm_to_af(input, m_intermediate);
  *m_intermediate = af::medfilt(*m_intermediate, m_kernelWidth, m_kernelWidth);
  m_imageProcessor->copy_af_to_itm(m_intermediate, output);
}
catch(af::exception&)
{
  // FIXME: Median filtering can occasionally fail due to a possible crash bug in ArrayFire. If this happens,
  //        we currently avoid throwing and instead treat this function as a no-op.
}

}
