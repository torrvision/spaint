#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <boost/mpl/list.hpp>

#include <spaint/geometry/DualNumber.h>
using namespace spaint;

//#################### FIXTURES ####################

/**
 * \brief An instance of an instantiation of this struct provides the context necessary for the tests.
 */
template <typename T>
struct Fixture
{
  //#################### PUBLIC VARIABLES ####################

  DualNumber<T> dn;

  //#################### CONSTRUCTORS ####################

  Fixture()
  : dn(2,3)
  {}
};

//#################### TESTS ####################

typedef boost::mpl::list<double,float> TS;
#define F Fixture<T>

BOOST_AUTO_TEST_SUITE(test_DualNumber)

BOOST_FIXTURE_TEST_CASE_TEMPLATE(test_conjugate, T, TS, F)
{
  BOOST_CHECK(DualNumber<T>::close(F::dn.conjugate(), DualNumber<T>(2,-3)));
}

BOOST_FIXTURE_TEST_CASE_TEMPLATE(test_inverse, T, TS, F)
{
  DualNumber<T> dnInv = F::dn.inverse();
  BOOST_CHECK(DualNumber<T>::close(dnInv, DualNumber<T>(0.5f,-0.75f)));
  BOOST_CHECK(DualNumber<T>::close(dnInv.inverse(), F::dn));
  BOOST_CHECK(DualNumber<T>::close(F::dn * dnInv, DualNumber<T>(1,0)));
  BOOST_CHECK(DualNumber<T>::close(dnInv * F::dn, DualNumber<T>(1,0)));
}

BOOST_FIXTURE_TEST_CASE_TEMPLATE(test_sqrt, T, TS, F)
{
  DualNumber<T> dnSqr = F::dn.sqrt();
  BOOST_CHECK(DualNumber<T>::close(dnSqr, DualNumber<T>(1.41421f,1.06066f)));
  BOOST_CHECK(DualNumber<T>::close(dnSqr * dnSqr, F::dn));
}

BOOST_AUTO_TEST_SUITE_END()
