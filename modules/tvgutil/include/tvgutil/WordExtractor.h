/**
 * tvgutil: WordExtractor.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGUTIL_WORDEXTRACTOR
#define H_TVGUTIL_WORDEXTRACTOR

#include <iosfwd>
#include <string>
#include <vector>

namespace tvgutil {

/**
 * \brief This struct contains utility functions that allow us to extract words from a stream.
 */
struct WordExtractor
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Extracts words from a stream and groups them by line.
   *
   * \param stream    The stream.
   * \param delimiter The delimiter to use to separate the words.
   * \return          The extracted words, grouped by line.
   */
  static std::vector<std::vector<std::string> > extract_words(std::istream& is, const std::string& delimiter);
};

}

#endif
