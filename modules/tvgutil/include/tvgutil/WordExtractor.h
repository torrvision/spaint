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
   * \brief Extracts lines of words from a stream.
   *
   * \param stream      The stream.
   * \param delimiters  The delimiters which can separate words in the stream.
   * \return            The lines of words.
   */
  static std::vector<std::vector<std::string> > extract_word_lines(std::istream& is, const std::string& delimiters);
};

}

#endif
