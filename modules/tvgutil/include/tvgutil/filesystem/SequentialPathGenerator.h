/**
 * tvgutil: SequentialPathGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_TVGUTIL_SEQUENTIALPATHGENERATOR
#define H_TVGUTIL_SEQUENTIALPATHGENERATOR

#include <string>

#include <boost/filesystem.hpp>

namespace tvgutil {

/**
 * \brief An instance of this class can be used to generate numbered paths in a base directory.
 */
class SequentialPathGenerator
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The path to the base directory. */
  boost::filesystem::path m_baseDir;

  /** The current index in the sequence (initially zero). */
  int m_index;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a sequential path generator.
   *
   * \param baseDir The path to the base directory.
   */
  explicit SequentialPathGenerator(const boost::filesystem::path& baseDir);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the path to the base directory.
   *
   * \return  The path to the base directory.
   */
  const boost::filesystem::path& get_base_dir() const;

  /**
   * \brief Gets the current value of the index in the sequence.
   *
   * \return The value of the index in the sequence.
   */
  int get_index() const;

  /**
   * \brief Increments the current index in the sequence.
   */
  void increment_index();

  /**
   * \brief Makes a path in the base directory that is numbered with the current index in the sequence.
   *
   * \param pattern The pattern to use when constructing the path.
   * \return        A path in the base directory that is numbered with the current index in the sequence.
   */
  boost::filesystem::path make_path(const std::string& pattern) const;
};

}

#endif
