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

//#################### FIXTURES ####################

/**
 * \brief An instance of this class provides the context needed for an image-copying test.
 */
class CopyImageFixture
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<af::array> AFArray_Ptr;

  //#################### PUBLIC VARIABLES ####################
public:
  /** A set of temporary ArrayFire images that can be used for the various image-copying tests. */
  std::map<std::string,AFArray_Ptr> afImages;

  /** The height of the images being copied. */
  int height;

  /** The image processor used to copy the images. */
  ImageProcessor_CPtr imageProcessor;

  /** A set of functions that can be used to make pixel values of the right types for the various image-copying tests. */
  std::map<std::string,boost::function<boost::any(size_t)> > pixelMakers;

  /** The width of the images being copied. */
  int width;

  //#################### CONSTRUCTORS ####################
public:
  CopyImageFixture()
  {
    // Choose a device for ArrayFire.
    if(af::getDeviceCount() > 1)
    {
      af::setDevice(1);
    }

    // Set up the fixture.
    imageProcessor = ImageProcessorFactory::make_image_processor(
#if WITH_CUDA
      ITMLibSettings::DEVICE_CUDA
#else
      ITMLibSettings::DEVICE_CPU
#endif
    );

    width = 3;
    height = 2;

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

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  template <typename T>
  static boost::any make_single_channel_pixel(size_t rasterIndex)
  {
    return static_cast<T>(rasterIndex);
  }

  static boost::any make_vector4u_pixel(size_t rasterIndex)
  {
    unsigned char c = static_cast<unsigned char>(rasterIndex);
    return Vector4u(c, c, c, 255);
  }
};

//#################### HELPER OPERATORS ####################

namespace ORUtils {

/**
 * \brief Checks whether two InfiniTAM images are equal.
 *
 * \param lhs The left-hand image.
 * \param rhs The right-hand image.
 * \return    true, if the two images are equal, or false otherwise.
 */
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

/**
 * \brief Outputs an InfiniTAM image to a stream.
 *
 * \param os  The stream.
 * \param rhs The image.
 * \return    The stream.
 */
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

/**
 * \brief Runs an image-copying test on the specified type of image.
 *
 * \param type    The name of the type of image on which to run the test.
 * \param fixture The fixture needed for the test.
 */
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

BOOST_FIXTURE_TEST_SUITE(test_ImageProcessor_copy_image, CopyImageFixture)

BOOST_AUTO_TEST_CASE(copy_image_float_test)
{
  copy_image_test<float>("float", *this);
}

BOOST_AUTO_TEST_CASE(copy_image_uchar_test)
{
  copy_image_test<unsigned char>("uchar", *this);
}

BOOST_AUTO_TEST_CASE(copy_image_Vector4u_test)
{
  copy_image_test<Vector4u>("Vector4u", *this);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE_END()
