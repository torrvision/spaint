#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <sstream>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;

#include <infermous/base/CRFUtil.h>
using namespace infermous;

//#################### HELPERS ####################

enum Colour
{
  RED,
  BLUE
};

template <> struct Eigen::NumTraits<Colour> : NumTraits<int> {};

std::string to_string(const std::vector<Eigen::Vector2i>& v)
{
  std::ostringstream oss;
  for(std::vector<Eigen::Vector2i>::const_iterator it = v.begin(), iend = v.end(); it != iend; ++it)
  {
    oss << it->transpose() << '\n';
  }
  return oss.str();
}

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_CRFUtil)

BOOST_AUTO_TEST_CASE(make_circular_neighbour_offsets_test)
{
  std::vector<Eigen::Vector2i> offsets = CRFUtil::make_circular_neighbour_offsets(3);

  std::vector<Eigen::Vector2i> expectedOffsets = list_of
    (Eigen::Vector2i(0,-3))
    (Eigen::Vector2i(-2,-2))
    (Eigen::Vector2i(-1,-2))
    (Eigen::Vector2i(0,-2))
    (Eigen::Vector2i(1,-2))
    (Eigen::Vector2i(2,-2))
    (Eigen::Vector2i(-2,-1))
    (Eigen::Vector2i(-1,-1))
    (Eigen::Vector2i(0,-1))
    (Eigen::Vector2i(1,-1))
    (Eigen::Vector2i(2,-1))
    (Eigen::Vector2i(-3,0))
    (Eigen::Vector2i(-2,0))
    (Eigen::Vector2i(-1,0))
    (Eigen::Vector2i(1,0))
    (Eigen::Vector2i(2,0))
    (Eigen::Vector2i(3,0))
    (Eigen::Vector2i(-2,1))
    (Eigen::Vector2i(-1,1))
    (Eigen::Vector2i(0,1))
    (Eigen::Vector2i(1,1))
    (Eigen::Vector2i(2,1))
    (Eigen::Vector2i(-2,2))
    (Eigen::Vector2i(-1,2))
    (Eigen::Vector2i(0,2))
    (Eigen::Vector2i(1,2))
    (Eigen::Vector2i(2,2))
    (Eigen::Vector2i(0,3));

  BOOST_CHECK_EQUAL(to_string(offsets), to_string(expectedOffsets));
}

BOOST_AUTO_TEST_CASE(make_square_neighbour_offsets_test)
{
  std::vector<Eigen::Vector2i> offsets = CRFUtil::make_square_neighbour_offsets(2);

  std::vector<Eigen::Vector2i> expectedOffsets = list_of
    (Eigen::Vector2i(-2,-2))
    (Eigen::Vector2i(-1,-2))
    (Eigen::Vector2i(0,-2))
    (Eigen::Vector2i(1,-2))
    (Eigen::Vector2i(2,-2))
    (Eigen::Vector2i(-2,-1))
    (Eigen::Vector2i(-1,-1))
    (Eigen::Vector2i(0,-1))
    (Eigen::Vector2i(1,-1))
    (Eigen::Vector2i(2,-1))
    (Eigen::Vector2i(-2,0))
    (Eigen::Vector2i(-1,0))
    (Eigen::Vector2i(1,0))
    (Eigen::Vector2i(2,0))
    (Eigen::Vector2i(-2,1))
    (Eigen::Vector2i(-1,1))
    (Eigen::Vector2i(0,1))
    (Eigen::Vector2i(1,1))
    (Eigen::Vector2i(2,1))
    (Eigen::Vector2i(-2,2))
    (Eigen::Vector2i(-1,2))
    (Eigen::Vector2i(0,2))
    (Eigen::Vector2i(1,2))
    (Eigen::Vector2i(2,2));

  BOOST_CHECK_EQUAL(to_string(offsets), to_string(expectedOffsets));
}

BOOST_AUTO_TEST_CASE(predict_labels_test)
{
  PotentialsGrid<Colour> potentials(2,2);
  potentials(0,0)[RED] = 0.6f;
  potentials(1,0)[RED] = 0.3f;
  potentials(0,1)[RED] = 0.2f;
  potentials(1,1)[RED] = 0.8f;

  for(size_t y = 0; y < 2; ++y)
  {
    for(size_t x = 0; x < 2; ++x)
    {
      potentials(x,y)[BLUE] = 1.0f - potentials(x,y)[RED];
    }
  }

  Grid<Colour> labels = CRFUtil::predict_labels(potentials);

  Grid<Colour> expectedLabels(2,2);
  expectedLabels(0,0) = expectedLabels(1,1) = RED;
  expectedLabels(1,0) = expectedLabels(0,1) = BLUE;

  BOOST_CHECK_EQUAL(labels, expectedLabels);
}

BOOST_AUTO_TEST_SUITE_END()
