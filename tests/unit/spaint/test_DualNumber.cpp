#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <boost/mpl/list.hpp>

#include <spaint/geometry/DualNumber.h>
using namespace spaint;

//#################### TESTS ####################

typedef boost::mpl::list<double,float> TS;

BOOST_AUTO_TEST_SUITE(test_DualNumber)

BOOST_AUTO_TEST_CASE_TEMPLATE(test_conjugate, T, TS)
{
  DualNumber<T> dn(2,3);
  BOOST_CHECK(DualNumber<T>::close(dn.conjugate(), DualNumber<T>(2,-3)));
}

BOOST_AUTO_TEST_CASE_TEMPLATE(test_inverse, T, TS)
{
  DualNumber<T> dn(2,3);
  DualNumber<T> dnInv = dn.inverse();
  BOOST_CHECK(DualNumber<T>::close(dnInv, DualNumber<T>(0.5f,-0.75f)));
  BOOST_CHECK(DualNumber<T>::close(dnInv.inverse(), dn));
  BOOST_CHECK(DualNumber<T>::close(dn * dnInv, DualNumber<T>(1,0)));
  BOOST_CHECK(DualNumber<T>::close(dnInv * dn, DualNumber<T>(1,0)));
}

BOOST_AUTO_TEST_CASE_TEMPLATE(test_sqrt, T, TS)
{
  DualNumber<T> dn(2,3);
  DualNumber<T> dnSqr = dn.sqrt();
  BOOST_CHECK(DualNumber<T>::close(dnSqr, DualNumber<T>(1.41421f,1.06066f)));
  BOOST_CHECK(DualNumber<T>::close(dnSqr * dnSqr, dn));
}

BOOST_AUTO_TEST_SUITE_END()
