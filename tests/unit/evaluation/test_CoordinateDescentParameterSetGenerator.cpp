#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;
using boost::assign::map_list_of;

// TODO: Is this needed?
#include <boost/bind.hpp>

#include <boost/lexical_cast.hpp>

#include <evaluation/core/ParamSetUtil.h>
#include <evaluation/paramsetgenerators/CoordinateDescentParameterSetGenerator.h>
using namespace evaluation;

#include <tvgutil/numbers/NumberSequenceGenerator.h>
using namespace tvgutil;

//#################### HELPER FUNCTIONS ####################

/**
  * \brief Converts a list of objects to a list of boost::hold_any.
  *
  * \param input The input list.
  * \return      The output list.
  */
template <typename T>
static std::vector<boost::spirit::hold_any> convert_to_hold_any(const std::vector<T>& input)
{
  size_t size = input.size();
  std::vector<boost::spirit::hold_any> result(size);
  for(size_t i = 0; i < size; ++i)
  {
    result[i].assign(input[i]);
  }
  return result;
}

float sum_squares_cost_fn(const ParamSet& params)
{
  float score = 0.0f;
  for(std::map<std::string,std::string>::const_iterator it = params.begin(), iend = params.end(); it != iend; ++it)
  {
    float value = boost::lexical_cast<float>(it->second);
    score += value * value;
  }
  return score;
}

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_CoordinateDescentParameterSetGenerator)

BOOST_AUTO_TEST_CASE(find_best_params)
{
  // TODO: Is this needed?
  boost::function<float(const ParamSet&)> costFunction = boost::bind(sum_squares_cost_fn, _1);

  const unsigned int seed = 12345;
  const size_t epochCount = 10;
  CoordinateDescentParameterSetGenerator paramGenerator(seed, epochCount);

  paramGenerator.add_param("Foo", convert_to_hold_any(NumberSequenceGenerator::generate_stepped<float>(-5.5, 1.5f, 5.0f)))
                .add_param("Bar", convert_to_hold_any(NumberSequenceGenerator::generate_stepped<float>(-1000.0f, 1.0f, 5.0f)))
                .add_param("Boo", list_of<float>(-10.0f)(-5.0f)(-2.0f)(0.0f)(5.0f)(15.0f))
                .add_param("Dum", list_of<float>(0.0f))
                .initialise(sum_squares_cost_fn);

  float bestScore;
  ParamSet bestParams = paramGenerator.calculate_best_parameters(bestScore);

  ParamSet answerParams = map_list_of("Foo",boost::lexical_cast<std::string>(0.5f))
                                     ("Bar",boost::lexical_cast<std::string>(0.0f))
                                     ("Boo",boost::lexical_cast<std::string>(0.0f))
                                     ("Dum",boost::lexical_cast<std::string>(0.0f));

  float answerScore = 0.25f;

  BOOST_CHECK_EQUAL(ParamSetUtil::param_set_to_string(bestParams), ParamSetUtil::param_set_to_string(answerParams));
  const float TOL = 1e-5f;
  BOOST_CHECK_CLOSE(bestScore, answerScore, TOL);
}

BOOST_AUTO_TEST_SUITE_END()
