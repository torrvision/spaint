#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <tvgutil/colours/ColourRGBA.h>
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

  std::map<int,ColourRGBA> palette = PaletteGenerator::generate_random_rgba_palette<int,ColourRGBA>(labels, 12345);
  BOOST_CHECK_EQUAL(palette[0], ColourRGBA::from_ints(237,227,80,255));
  BOOST_CHECK_EQUAL(palette[1], ColourRGBA::from_ints(33,47,10,255));
  BOOST_CHECK_EQUAL(palette[2], ColourRGBA::from_ints(52,211,145,255));
}

BOOST_AUTO_TEST_SUITE_END()
