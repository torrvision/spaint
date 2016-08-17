#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <boost/assign/list_of.hpp>
#include <boost/lexical_cast.hpp>
using boost::assign::list_of;
using boost::assign::map_list_of;

#include <evaluation/util/CoordinateDescentParameterOptimiser.h>
using namespace evaluation;

#include <tvgutil/numbers/NumberSequenceGenerator.h>
using namespace tvgutil;

//#################### HELPER FUNCTIONS ####################

float sum_squares_cost_fn(const ParamSet& params)
{
  float cost = 0.0f;
  for(std::map<std::string,std::string>::const_iterator it = params.begin(), iend = params.end(); it != iend; ++it)
  {
    float value = boost::lexical_cast<float>(it->second);
    cost += value * value;
  }
  return cost;
}

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_CoordinateDescentParameterOptimiser)

BOOST_AUTO_TEST_CASE(choose_parameters_test)
{
  // Set up the optimiser.
  const unsigned int seed = 12345;
  const size_t epochCount = 10;
  CoordinateDescentParameterOptimiser optimiser(sum_squares_cost_fn, epochCount, seed);
  optimiser.add_param("Foo", NumberSequenceGenerator::generate_stepped<float>(-5.5f, 1.5f, 5.0f))
           .add_param("Bar", NumberSequenceGenerator::generate_stepped<float>(-1000.0f, 1.0f, 5.0f))
           .add_param("Boo", list_of<float>(-10.0f)(-5.0f)(-2.0f)(0.0f)(5.0f)(15.0f))
           .add_param("Dum", list_of<float>(0.0f));

  // Use the optimiser to choose a set of parameters.
  float cost;
  ParamSet params = optimiser.choose_parameters(&cost);

  // Check that the chosen parameters are as expected.
  ParamSet expectedParams = map_list_of("Foo",boost::lexical_cast<std::string>(0.5f))
                                       ("Bar",boost::lexical_cast<std::string>(0.0f))
                                       ("Boo",boost::lexical_cast<std::string>(0.0f))
                                       ("Dum",boost::lexical_cast<std::string>(0.0f));

  BOOST_CHECK_EQUAL(ParamSetUtil::param_set_to_string(params), ParamSetUtil::param_set_to_string(expectedParams));

  // Check that the cost of the chosen parameters is as expected.
  const float expectedCost = 0.25f;
  const float TOL = 1e-5f;
  BOOST_CHECK_CLOSE(cost, expectedCost, TOL);
}

BOOST_AUTO_TEST_SUITE_END()
