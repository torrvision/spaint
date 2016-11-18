#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <spaint/geometry/DualNumber.h>
using namespace spaint;

//#################### FIXTURES ####################

/**
 * \brief An instance of this struct provides the context necessary for the tests.
 */
struct Fixture
{
  //#################### PUBLIC VARIABLES ####################

  DualNumberd nd;
  DualNumberf nf;

  //#################### CONSTRUCTORS ####################

  Fixture()
  : nd(2.0, 3.0), nf(5.0f, 4.0f)
  {}
};

//#################### TESTS ####################

BOOST_FIXTURE_TEST_SUITE(test_DualNumber, Fixture)

BOOST_AUTO_TEST_CASE(test_conjugate)
{
  BOOST_CHECK_EQUAL(nd.conjugate(), DualNumberd(2.0, -3.0));
  BOOST_CHECK_EQUAL(nf.conjugate(), DualNumberf(5.0f, -4.0f));
}

BOOST_AUTO_TEST_CASE(test_inverse)
{
  DualNumberd ndInv = nd.inverse();
  BOOST_CHECK_EQUAL(ndInv, DualNumberd(0.5, -0.75));
  BOOST_CHECK_EQUAL(ndInv.inverse(), nd);
  BOOST_CHECK_EQUAL(nd * ndInv, DualNumberd(1.0, 0.0));
  BOOST_CHECK_EQUAL(ndInv * nd, DualNumberd(1.0, 0.0));

  DualNumberf nfInv = nf.inverse();
  BOOST_CHECK_EQUAL(nfInv, DualNumberf(0.2f, -0.16f));
  BOOST_CHECK_EQUAL(nfInv.inverse(), nf);
  BOOST_CHECK_EQUAL(nf * nfInv, DualNumberf(1.0, 0.0));
  BOOST_CHECK_EQUAL(nfInv * nf, DualNumberf(1.0, 0.0));
}

BOOST_AUTO_TEST_CASE(test_sqrt)
{
  DualNumberd ndSqr = nd.sqrt();
  BOOST_CHECK_EQUAL(ndSqr, DualNumberd(1.41421, 1.06066));
  BOOST_CHECK_EQUAL(ndSqr * ndSqr, nd);

  DualNumberf nfSqr = nf.sqrt();
  BOOST_CHECK_EQUAL(nfSqr, DualNumberf(2.23607f, 0.894427f));
  BOOST_CHECK_EQUAL(nfSqr * nfSqr, nf);
}

BOOST_AUTO_TEST_SUITE_END()
