/**
 * tvgutil: IOUtil.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "IOUtil.h"

#include <stdexcept>

#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>

namespace tvgutil {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

std::vector<std::string> IOUtil::extract_lines(std::istream& is)
{
  std::vector<std::string> lines;

  std::string line;
  while(std::getline(is, line))
  {
    boost::trim(line);
    if(!line.empty()) lines.push_back(line);
  }

  return lines;
}

std::vector<std::vector<std::string> > IOUtil::extract_word_lines(std::istream& is, const std::string& delimiters)
{
  std::vector<std::vector<std::string> > wordLines;

  std::string internalDelimiters = delimiters + '\r';

  std::string line;
  while(std::getline(is, line))
  {
    typedef boost::char_separator<char> sep;
    typedef boost::tokenizer<sep> tokenizer;

    tokenizer tok(line.begin(), line.end(), sep(internalDelimiters.c_str()));
    std::vector<std::string> words(tok.begin(), tok.end());
    wordLines.push_back(words);
  }

  return wordLines;
}

void IOUtil::output_lines(std::ostream& os, const std::vector<std::string>& lines)
{
  if(!os) throw std::runtime_error("Unable to write to the output stream");

  for(size_t i = 0, count = lines.size(); i < count; ++i)
  {
    os << lines[i] << '\n';
  }
}

}
