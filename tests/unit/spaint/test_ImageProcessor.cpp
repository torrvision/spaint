#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <map>

#ifdef _MSC_VER
  // Suppress some VC++ warnings that are produced when including the Boost headers.
  #pragma warning(disable:4180)
#endif

#include <boost/any.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/function.hpp>
using boost::assign::map_list_of;

#ifdef _MSC_VER
  // Reenable the suppressed warnings for the rest of the translation unit.
  #pragma warning(default:4180)
#endif

#include <spaint/imageprocessing/ImageProcessorFactory.h>
using namespace ITMLib;
using namespace spaint;

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<af::array> AFArray_Ptr;

//#################### FIXTURES ####################

struct CopyImageFixture
{
  //#################### PUBLIC VARIABLES ####################

  std::map<std::string,AFArray_Ptr> afImages;
  int height;
  ImageProcessor_CPtr imageProcessor;
  std::map<std::string,boost::function<boost::any(size_t)> > pixelMakers;
  int width;

  //#################### CONSTRUCTORS ####################

  CopyImageFixture()
  {
    // Choose a device for ArrayFire.
    if(af::getDeviceCount() > 1)
    {
      af::setDevice(1);
    }

    // Set up the fixture.
    imageProcessor = ImageProcessorFactory::make_image_processor(WITH_CUDA ? ITMLibSettings::DEVICE_CUDA : ITMLibSettings::DEVICE_CPU);
    width = 3, height = 2;

    afImages = map_list_of
      (std::string("float"), AFArray_Ptr(new af::array(height, width, f32)))
      (std::string("uchar"), AFArray_Ptr(new af::array(height, width, u8)))
      (std::string("Vector4u"), AFArray_Ptr(new af::array(height, width, 4, u8)))
    .to_container(afImages);

    pixelMakers = map_list_of
      (std::string("float"), &make_single_channel_pixel<float>)
      (std::string("uchar"), &make_single_channel_pixel<unsigned char>)
      (std::string("Vector4u"), &make_vector4u_pixel)
    .to_container(pixelMakers);
  }

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  template <typename T>
  static boost::any make_single_channel_pixel(size_t i)
  {
    return static_cast<T>(i);
  }

  static boost::any make_vector4u_pixel(size_t i)
  {
    unsigned char c = static_cast<unsigned char>(i);
    return Vector4u(c, c, c, 255);
  }
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

//#################### HELPER FUNCTIONS ####################

template <typename T>
void copy_image_test(const std::string& type, CopyImageFixture& fixture)
{
  // Set up the InfiniTAM input image.
  boost::shared_ptr<ORUtils::Image<T> > itmInputImage(new ORUtils::Image<T>(Vector2i(fixture.width, fixture.height), true, true));
  T *itmInputImagePtr = itmInputImage->GetData(MEMORYDEVICE_CPU);
  for(size_t i = 0, size = itmInputImage->dataSize; i < size; ++i)
  {
    itmInputImagePtr[i] = boost::any_cast<T>(fixture.pixelMakers[type](i));
  }
  itmInputImage->UpdateDeviceFromHost();

  // Copy the InfiniTAM input image to the ArrayFire image.
  fixture.imageProcessor->copy_itm_to_af(itmInputImage, fixture.afImages[type]);

  // Copy the ArrayFire image to the InfiniTAM output image.
  boost::shared_ptr<ORUtils::Image<T> > itmOutputImage(new ORUtils::Image<T>(itmInputImage->noDims, true, true));
  fixture.imageProcessor->copy_af_to_itm(fixture.afImages[type], itmOutputImage);
  itmOutputImage->UpdateHostFromDevice();

  // Check that the two InfiniTAM images are the same.
  BOOST_CHECK_EQUAL(*itmInputImage, *itmOutputImage);
}

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_ImageProcessor)

BOOST_FIXTURE_TEST_CASE(copy_image_float_test, CopyImageFixture)
{
  copy_image_test<float>("float", *this);
}

BOOST_FIXTURE_TEST_CASE(copy_image_uchar_test, CopyImageFixture)
{
  copy_image_test<unsigned char>("uchar", *this);
}

BOOST_FIXTURE_TEST_CASE(copy_image_Vector4u_test, CopyImageFixture)
{
  copy_image_test<Vector4u>("Vector4u", *this);
}

BOOST_AUTO_TEST_SUITE_END()
