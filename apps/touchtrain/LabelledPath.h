/**
 * touchtrain: LabelledPath.h
 */

#ifndef H_TOUCHTRAIN_LABELLEDPATH
#define H_TOUCHTRAIN_LABELLEDPATH

#include <iostream>

/**
 * \brief A struct that represents a labelled path.
 */
template <typename Label>
struct LabelledPath
{
  //#################### PUBLIC VARIABLES ####################

  /** The path. */
  std::string path;

  /** The label associated with the path. */
  Label label;

  //#################### CONSTRUCTORS ####################

  /**
   * \brief Constructs the labelled image path.
   *
   * \param path   The path.
   * \param label  The label associated with the specified path.
   */
  LabelledPath(const std::string& imagePath_, const Label& label_)
  : path(imagePath_), label(label_)
  {}
};

#endif
