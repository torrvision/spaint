/**
 * spaint: TouchUtil.h
 */

#ifndef H_SPAINT_TOUCHUTIL
#define H_SPAINT_TOUCHUTIL

#include "../ocv/OpenCVUtil.h"

#include <arrayfire.h>

#include <rafl/core/RandomForest.h>

namespace spaint {

class TouchUtil
{
public:
  static rafl::Descriptor_CPtr extract_touch_feature(const af::array& img)
  {
    af::array globalHistogram = af::histogram(img, 64, 0, 255);
    const float *gHist = globalHistogram.as(f32).host<float>();
    const int gHistLen = globalHistogram.dims(0);

#if 0
    OpenCVUtil::show_greyscale_figure("img", img.as(u8).host<unsigned char>(), img.dims(1), img.dims(0), OpenCVUtil::COL_MAJOR);
    af::print("globalHistogram", globalHistogram);
    cv::waitKey(10);
#endif

    return rafl::Descriptor_CPtr(new rafl::Descriptor(gHist, gHist + gHistLen));
  }
};

}

#endif
