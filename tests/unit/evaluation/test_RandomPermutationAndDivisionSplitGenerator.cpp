#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <stdexcept>
#include <vector>

#include <evaluation/splitgenerators/RandomPermutationAndDivisionSplitGenerator.h>
using namespace evaluation;

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_RandomPermutationAndDivisionSplitGenerator)

BOOST_AUTO_TEST_CASE(constructor_test)
{
  const unsigned int seed = 12345;
  const float ratio = 0.5f;

  BOOST_CHECK_THROW(RandomPermutationAndDivisionSplitGenerator(seed, 0, ratio), std::runtime_error);
  BOOST_CHECK_NO_THROW(RandomPermutationAndDivisionSplitGenerator(seed, 1, ratio));

  BOOST_CHECK_THROW(RandomPermutationAndDivisionSplitGenerator(seed, 1, 0.0f), std::runtime_error);
  BOOST_CHECK_THROW(RandomPermutationAndDivisionSplitGenerator(seed, 1, 1.0f), std::runtime_error);
  BOOST_CHECK_NO_THROW(RandomPermutationAndDivisionSplitGenerator(seed, 1, 0.1f));
  BOOST_CHECK_NO_THROW(RandomPermutationAndDivisionSplitGenerator(seed, 1, 0.9f));
}

BOOST_AUTO_TEST_CASE(generate_splits_test)
{
  const size_t exampleCount = 20;
  const size_t splitCount = 5;
  const float ratio = 0.75f;

  RandomPermutationAndDivisionSplitGenerator gen(12345, splitCount, ratio);
  std::vector<SplitGenerator::Split> splits = gen.generate_splits(exampleCount);

  // Check the sizes of the split halves.
  BOOST_REQUIRE_EQUAL(splits.size(), splitCount);
  for(size_t i = 0; i < splitCount; ++i)
  {
    BOOST_CHECK_EQUAL(splits[i].first.size(), exampleCount * ratio);
    BOOST_CHECK_EQUAL(splits[i].second.size(), exampleCount * (1.0f - ratio));
  }
}

BOOST_AUTO_TEST_SUITE_END()
