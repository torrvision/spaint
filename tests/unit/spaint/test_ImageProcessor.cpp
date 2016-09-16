#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <map>

#include <boost/any.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/function.hpp>
using boost::assign::map_list_of;

#include <spaint/imageprocessing/ImageProcessorFactory.h>
using namespace ITMLib;
using namespace spaint;

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<af::array> AFArray_Ptr;

//#################### FIXTURES ####################

struct Fixture
{
  //#################### PUBLIC VARIABLES ####################

  std::map<std::string,AFArray_Ptr> afImages;
  int height;
  ImageProcessor_CPtr imageProcessor;
  std::map<std::string,boost::function<boost::any(size_t)> > valueMakers;
  int width;

  //#################### CONSTRUCTORS ####################

  Fixture()
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
      (std::string("Vector4u"), AFArray_Ptr(new af::array(height, width, 4, u8)))
    .to_container(afImages);

    valueMakers = map_list_of
      (std::string("Vector4u"), &make_vector4u)
    .to_container(valueMakers);
  }

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  static Vector4u make_vector4u(size_t i)
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
void copy_af_to_itm_test(const std::string& type, Fixture& fixture)
{
  // Set up the InfiniTAM input image.
  ITMUChar4Image_Ptr itmInputImage(new ITMUChar4Image(Vector2i(fixture.width, fixture.height), true, true));
  Vector4u *itmInputImagePtr = itmInputImage->GetData(MEMORYDEVICE_CPU);
  for(size_t i = 0, size = itmInputImage->dataSize; i < size; ++i)
  {
    itmInputImagePtr[i] = boost::any_cast<T>(fixture.valueMakers[type](i));
  }
  itmInputImage->UpdateDeviceFromHost();

  // Copy the InfiniTAM input image to the ArrayFire image.
  fixture.imageProcessor->copy_itm_to_af(itmInputImage, fixture.afImages[type]);

  // Copy the ArrayFire image to the InfiniTAM output image.
  ITMUChar4Image_Ptr itmOutputImage(new ITMUChar4Image(itmInputImage->noDims, true, true));
  fixture.imageProcessor->copy_af_to_itm(fixture.afImages[type], itmOutputImage);
  itmOutputImage->UpdateHostFromDevice();

  // Check that the two InfiniTAM images are the same.
  BOOST_CHECK_EQUAL(*itmInputImage, *itmOutputImage);
}

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_ImageProcessor)

BOOST_FIXTURE_TEST_CASE(copy_af_to_itm_Vector4u_test, Fixture)
{
  copy_af_to_itm_test<Vector4u>("Vector4u", *this);
}

BOOST_AUTO_TEST_SUITE_END()
