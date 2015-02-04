#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <iterator>
#include <vector>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;

#include <evaluation/splitgenerators/CrossValidationSplitGenerator.h>
using namespace evaluation;

//#################### TESTS ####################

BOOST_AUTO_TEST_SUITE(test_CrossValidationSplitGenerator)

BOOST_AUTO_TEST_CASE(generate_splits_test)
{
  const size_t exampleCount = 22;
  const size_t foldSizes[] = {5,5,4,4,4};
  const size_t foldCount = sizeof(foldSizes) / sizeof(size_t);

  CrossValidationSplitGenerator gen(12345, foldCount);
  std::vector<SplitGenerator::Split> splits = gen.generate_splits(exampleCount);

  // Check the sizes of the split halves.
  BOOST_REQUIRE_EQUAL(splits.size(), foldCount);
  for(size_t i = 0; i < foldCount; ++i)
  {
    BOOST_CHECK_EQUAL(splits[i].first.size(), exampleCount - foldSizes[i]);
    BOOST_CHECK_EQUAL(splits[i].second.size(), foldSizes[i]);
  }

  // Check that the folds do not share any examples in common (thus, by the pigeonhole principle, all examples appear in exactly one fold).
  for(size_t i = 0; i < foldCount; ++i)
  {
    std::vector<size_t> foldI = splits[i].second;
    std::sort(foldI.begin(), foldI.end());

    for(size_t j = 0; j < foldCount; ++j)
    {
      if(j == i) continue;

      std::vector<size_t> foldJ = splits[j].second;
      std::sort(foldJ.begin(), foldJ.end());

      std::vector<size_t> intersection;
      std::set_intersection(foldI.begin(), foldI.end(), foldJ.begin(), foldJ.end(), std::back_inserter(intersection));
      BOOST_CHECK(intersection.empty());
    }
  }
}

BOOST_AUTO_TEST_SUITE_END()
