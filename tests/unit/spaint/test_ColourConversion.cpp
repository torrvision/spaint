#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <spaint/util/ColourConversion_Shared.h>
using namespace spaint;

//#################### HELPER FUNCTIONS ####################

void check_rgb_to_ycbcr_to_rgb(const Vector3u& rgb)
{
  BOOST_CHECK_EQUAL(convert_ycbcr_to_rgb(convert_rgb_to_ycbcr(rgb)), rgb);
}

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_ColourConversion)

BOOST_AUTO_TEST_CASE(convert_rgb_to_ycbcr_test)
{
  // To manually verify that these values make sense, see the CbCr plane image at https://en.wikipedia.org/wiki/YCbCr.
  BOOST_CHECK_EQUAL(convert_rgb_to_ycbcr(Vector3u(0,0,0)), Vector3f(0.0f,127.5f,127.5f));
  BOOST_CHECK_EQUAL(convert_rgb_to_ycbcr(Vector3u(255,0,0)), Vector3f(76.2449951f,84.4049988f,255.0f));
  BOOST_CHECK_EQUAL(convert_rgb_to_ycbcr(Vector3u(0,255,0)), Vector3f(149.684998f,43.0950012f,20.6549988f));
  BOOST_CHECK_EQUAL(convert_rgb_to_ycbcr(Vector3u(0,0,255)), Vector3f(29.0699997f,255.0f,106.845001f));
  BOOST_CHECK_EQUAL(convert_rgb_to_ycbcr(Vector3u(255,255,0)), Vector3f(225.929993f,0.0f,148.154999f));
  BOOST_CHECK_EQUAL(convert_rgb_to_ycbcr(Vector3u(255,0,255)), Vector3f(105.314995f,211.904999f,234.345001f));
  BOOST_CHECK_EQUAL(convert_rgb_to_ycbcr(Vector3u(0,255,255)), Vector3f(178.755005f,170.595001f,0.0f));
  BOOST_CHECK_EQUAL(convert_rgb_to_ycbcr(Vector3u(255,255,255)), Vector3f(255.0f,127.5f,127.5f));
}

BOOST_AUTO_TEST_CASE(convert_rgb_to_ycbcr_to_rgb_test)
{
  check_rgb_to_ycbcr_to_rgb(Vector3u(0,0,0));
  check_rgb_to_ycbcr_to_rgb(Vector3u(255,0,0));
  check_rgb_to_ycbcr_to_rgb(Vector3u(0,255,0));
  check_rgb_to_ycbcr_to_rgb(Vector3u(0,0,255));
  check_rgb_to_ycbcr_to_rgb(Vector3u(255,255,0));
  check_rgb_to_ycbcr_to_rgb(Vector3u(255,0,255));
  check_rgb_to_ycbcr_to_rgb(Vector3u(0,255,255));
  check_rgb_to_ycbcr_to_rgb(Vector3u(255,255,255));
}

BOOST_AUTO_TEST_SUITE_END()
