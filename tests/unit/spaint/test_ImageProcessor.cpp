#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <spaint/imageprocessing/ImageProcessorFactory.h>
using namespace ITMLib;
using namespace spaint;

//#################### FIXTURES ####################

struct Fixture
{
  //#################### PUBLIC VARIABLES ####################

  ImageProcessor_CPtr imageProcessor;

  //#################### CONSTRUCTORS ####################

  Fixture()
  : imageProcessor(ImageProcessorFactory::make_image_processor(WITH_CUDA ? ITMLibSettings::DEVICE_CUDA : ITMLibSettings::DEVICE_CPU))
  {}
};

//#################### HELPER OPERATORS ####################

namespace ORUtils {

template <typename T>
bool operator==(const ORUtils::Image<T>& lhs, const ORUtils::Image<T>& rhs)
{
  if(lhs.dataSize != rhs.dataSize) return false;

  const T *lhsPtr = lhs.GetData(MEMORYDEVICE_CPU);
  const T *rhsPtr = rhs.GetData(MEMORYDEVICE_CPU);
  for(size_t i = 0, size = lhs.dataSize; i < size; ++i)
  {
    if(lhsPtr[i] != rhsPtr[i]) return false;
  }

  return true;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const ORUtils::Image<T>& rhs)
{
  const int width = rhs.noDims.x, height = rhs.noDims.y;
  const T *rhsPtr = rhs.GetData(MEMORYDEVICE_CPU);
  int i = 0;
  for(int y = 0; y < height; ++y)
  {
    for(int x = 0; x < width; ++x)
    {
      os << rhsPtr[i++] << ' ';
    }
    os << '\n';
  }
  return os;
}

}

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_ImageProcessor)

BOOST_FIXTURE_TEST_CASE(copy_af_to_itm_Vector4u_test, Fixture)
{
  // Choose a device for ArrayFire.
  if(af::getDeviceCount() > 1)
  {
    af::setDevice(1);
  }

  // Set up the InfiniTAM input image.
  const int width = 3, height = 2;
  ITMUChar4Image_Ptr itmInputImage(new ITMUChar4Image(Vector2i(width, height), true, true));
  Vector4u *itmInputImagePtr = itmInputImage->GetData(MEMORYDEVICE_CPU);
  for(size_t i = 0, size = itmInputImage->dataSize; i < size; ++i)
  {
    unsigned char c = static_cast<unsigned char>(i);
    itmInputImagePtr[i] = Vector4u(c, c, c, 255);
  }
  itmInputImage->UpdateDeviceFromHost();

  // Copy the InfiniTAM input image to the ArrayFire image.
  ImageProcessor::AFArray_Ptr afImage(new af::array(height, width, 4, u8));
  imageProcessor->copy_itm_to_af(itmInputImage, afImage);

  // Copy the ArrayFire image to the InfiniTAM output image.
  ITMUChar4Image_Ptr itmOutputImage(new ITMUChar4Image(itmInputImage->noDims, true, true));
  imageProcessor->copy_af_to_itm(afImage, itmOutputImage);
  itmOutputImage->UpdateHostFromDevice();

  // Check that the two InfiniTAM images are the same.
  BOOST_CHECK_EQUAL(*itmInputImage, *itmOutputImage);
}

BOOST_AUTO_TEST_SUITE_END()
