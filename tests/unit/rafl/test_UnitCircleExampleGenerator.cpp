#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;

#include <rafl/examples/UnitCircleExampleGenerator.h>
using namespace rafl;

typedef int Label;
typedef boost::shared_ptr<const Example<Label> > Example_CPtr;

/**
 * \brief Checks that the specified feature in the example's descriptor has the specified value.
 *
 * \param example       The example.
 * \param featureIndex  The index of the feature in the example's descriptor that we want to check.
 * \param value         The expected value of the feature.
 */
void check_descriptor_value(const Example_CPtr& example, int featureIndex, float value)
{
  BOOST_CHECK_SMALL(fabs((*example->get_descriptor())[featureIndex] - value), 1e-6f);
}

BOOST_AUTO_TEST_SUITE(test_UnitCircleExampleGenerator)

BOOST_AUTO_TEST_CASE(generate_examples_test)
{
  // Generate examples at the four axial points on the unit circle.
  UnitCircleExampleGenerator<Label> generator(list_of(1)(2)(3)(4), 1234, 0.0f, 0.0f);

  // Check that specifying a class label for which the generator is not configured causes a throw.
  BOOST_CHECK_THROW(generator.generate_examples(list_of(1)(5), 5), std::runtime_error);

  // Check that sampling the same class multiple times gives us several copies of the same point (note that we are using standard deviations of 0 for testing purposes).
  std::vector<Example_CPtr> examples = generator.generate_examples(list_of(1), 2);
  BOOST_REQUIRE_EQUAL(examples.size(), 2);
  for(size_t i = 0, size = examples.size(); i < size; ++i)
  {
    check_descriptor_value(examples[i], 0, 1.0f);
    check_descriptor_value(examples[i], 1, 0.0f);
  }

  // Check that sampling from multiple classes works.
  examples = generator.generate_examples(list_of(2)(3), 1);
  BOOST_REQUIRE_EQUAL(examples.size(), 2);
  check_descriptor_value(examples[0], 0, 0.0f);
  check_descriptor_value(examples[0], 1, 1.0f);
  check_descriptor_value(examples[1], 0, -1.0f);
  check_descriptor_value(examples[1], 1, 0.0f);
}

BOOST_AUTO_TEST_SUITE_END()
