/**
 * tvgutil: WordExtractor.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "WordExtractor.h"

#include <boost/tokenizer.hpp>

namespace tvgutil {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

std::vector<std::vector<std::string> > WordExtractor::extract_word_lines(std::istream& is, const std::string& delimiters)
{
  std::vector<std::vector<std::string> > wordLines;

  std::string line;
  while(std::getline(is, line))
  {
    typedef boost::char_separator<char> sep;
    typedef boost::tokenizer<sep> tokenizer;

    tokenizer tok(line.begin(), line.end(), sep(delimiters.c_str()));
    std::vector<std::string> words(tok.begin(), tok.end());
    wordLines.push_back(words);
  }

  return wordLines;
}

}
