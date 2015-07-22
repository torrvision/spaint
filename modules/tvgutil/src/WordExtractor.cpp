/**
 * tvgutil: WordExtractor.cpp
 */

#include "WordExtractor.h"

#include <boost/tokenizer.hpp>

namespace tvgutil {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

std::vector<std::vector<std::string> > WordExtractor::extract_words(std::istream& is, const std::string& delimiter)
{
  std::vector<std::vector<std::string> > words;

  std::string line;
  while(std::getline(is, line))
  {
    typedef boost::char_separator<char> sep;
    typedef boost::tokenizer<sep> tokenizer;

    tokenizer tok(line.begin(), line.end(), sep(delimiter.c_str()));
    std::vector<std::string> tokens(tok.begin(), tok.end());
    words.push_back(tokens);
  }

  return words;
}

}
