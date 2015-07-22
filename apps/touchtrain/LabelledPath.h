/**
 * touchtrain: LabelledPath.h
 */

#ifndef H_TOUCHTRAIN_LABELLEDPATH
#define H_TOUCHTRAIN_LABELLEDPATH

#include <string>

/**
 * \brief An instance of this struct represents a path with an associated label.
 */
template <typename Label>
struct LabelledPath
{
  //#################### PUBLIC VARIABLES ####################

  /** The label associated with the path. */
  const Label label;

  /** The path. */
  const std::string path;

  //#################### CONSTRUCTORS ####################

  /**
   * \brief Constructs a labelled path.
   *
   * \param path  The path.
   * \param label The label associated with the path.
   */
  LabelledPath(const std::string& path_, const Label& label_)
  : label(label_), path(path_)
  {}
};

#endif
