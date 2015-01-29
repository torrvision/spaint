#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <tvgutil/colours/PaletteGenerator.h>
using namespace tvgutil;

BOOST_AUTO_TEST_SUITE(test_PaletteGenerator)

BOOST_AUTO_TEST_CASE(generate_random_palette_test)
{
  std::set<int> labels;
  for(int i = 0; i < 3; ++i)
  {
    labels.insert(i);
  }

  std::map<int,RGBAColourUC> palette = PaletteGenerator::generate_random_palette(labels, 12345);
  BOOST_CHECK_EQUAL(palette[0], RGBAColourUC(80,227,237,255));
  BOOST_CHECK_EQUAL(palette[1], RGBAColourUC(10,47,33,255));
  BOOST_CHECK_EQUAL(palette[2], RGBAColourUC(145,211,52,255));
}

BOOST_AUTO_TEST_SUITE_END()
