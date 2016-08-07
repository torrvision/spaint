/**
 * tvgutil: IOUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGUTIL_IOUTIL
#define H_TVGUTIL_IOUTIL

#include <iosfwd>
#include <string>
#include <vector>

namespace tvgutil {

/**
 * \brief This struct contains I/O utility functions.
 */
struct IOUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Extracts lines from a stream.
   *
   * \param is  The stream.
   * \return    The lines.
   */
  static std::vector<std::string> extract_lines(std::istream& is);

  /**
   * \brief Extracts lines of words from a stream.
   *
   * \param stream      The stream.
   * \param delimiters  The delimiters which can separate words in the stream.
   * \return            The lines of words.
   */
  static std::vector<std::vector<std::string> > extract_word_lines(std::istream& is, const std::string& delimiters);

  /**
   * \brief Output lines to a stream.
   *
   * \param os     The stream.
   * \param lines  The lines.
   */
  static void output_lines(std::ostream& os, const std::vector<std::string>& lines);
};

}

#endif
