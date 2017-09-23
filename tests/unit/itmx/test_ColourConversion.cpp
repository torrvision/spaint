#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <itmx/util/ColourConversion_Shared.h>
using namespace itmx;

//#################### HELPER FUNCTIONS ####################

void check_close(float a, float b, float TOL)
{
  BOOST_CHECK_CLOSE(a, b, TOL);
}

void check_close(const float *v1, const float *v2, size_t size, float TOL)
{
  for(size_t i = 0; i < size; ++i) check_close(v1[i], v2[i], TOL);
}

void check_close(const Vector3f& v1, const Vector3f& v2, float TOL)
{
  check_close(v1.v, v2.v, v1.size(), TOL);
}

void check_rgb_to_ycbcr_to_rgb(const Vector3u& rgb)
{
  BOOST_CHECK_EQUAL(convert_ycbcr_to_rgb(convert_rgb_to_ycbcr(rgb)), rgb);
}

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_ColourConversion)

BOOST_AUTO_TEST_CASE(convert_rgb_to_ycbcr_test)
{
  const float TOL = 1e-5f;

  #define CHECK_CLOSE(L,R) check_close(L,R,TOL)

  // To manually verify that these values make sense, see the CbCr plane image at https://en.wikipedia.org/wiki/YCbCr.
  CHECK_CLOSE(convert_rgb_to_ycbcr(Vector3u(0,0,0)), Vector3f(0.0f,127.5f,127.5f));
  CHECK_CLOSE(convert_rgb_to_ycbcr(Vector3u(255,0,0)), Vector3f(76.2449951f,84.4049988f,255.0f));
  CHECK_CLOSE(convert_rgb_to_ycbcr(Vector3u(0,255,0)), Vector3f(149.684998f,43.0950012f,20.6549988f));
  CHECK_CLOSE(convert_rgb_to_ycbcr(Vector3u(0,0,255)), Vector3f(29.0699997f,255.0f,106.845001f));
  CHECK_CLOSE(convert_rgb_to_ycbcr(Vector3u(255,255,0)), Vector3f(225.929993f,0.0f,148.154999f));
  CHECK_CLOSE(convert_rgb_to_ycbcr(Vector3u(255,0,255)), Vector3f(105.314995f,211.904999f,234.345001f));
  CHECK_CLOSE(convert_rgb_to_ycbcr(Vector3u(0,255,255)), Vector3f(178.755005f,170.595001f,0.0f));
  CHECK_CLOSE(convert_rgb_to_ycbcr(Vector3u(255,255,255)), Vector3f(255.0f,127.5f,127.5f));

  #undef CHECK_CLOSE
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
