/**
 * spaint: LabelManager.cpp
 */

#ifndef H_SPAINT_LABELMANAGER
#define H_SPAINT_LABELMANAGER

#include <string>

#include <tvgutil/IDAllocator.h>

#include "SpaintVoxel.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to manage the labels that are used for labelling a scene.
 */
class LabelManager
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The ID allocator used to allocate labels. */
  tvgutil::IDAllocator m_labelAllocator;

  /** The maximum number of labels that the manager is allowed to allocate. */
  size_t m_maxLabelCount;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a label manager.
   *
   * \param maxLabelCount The maximum number of labels that the manager is allowed to allocate.
   */
  explicit LabelManager(size_t maxLabelCount);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds a label with the specified name.
   *
   * \param name  The name of the label we want to add.
   */
  void add_label(const std::string& name);

  /**
   * \brief Deletes the label with the specified name.
   *
   * \param name                The name of the label to delete.
   * \throws std::runtime_error If the manager does not contain a label with the specified name.
   */
  void delete_label(const std::string& name);

  /**
   * \brief Gets the label with the specified name.
   *
   * \param name                The name of the label we want to get.
   * \return                    The label with the specified name.
   * \throws std::runtime_error If the manager does not contain a label with the specified name.
   */
  SpaintVoxel::LabelType get_label(const std::string& name) const;

  /**
   * \brief Gets the colour of the label with the specified name.
   *
   * \param name                The name of the label whose colour we want to get.
   * \return                    The colour of the label with the specified name.
   * \throws std::runtime_error If the manager does not contain a label with the specified name.
   */
  void /* Colour */ get_label_colour(const std::string& name) const;

  /**
   * \brief Gets the name of the specified label.
   *
   * \param label               The label whose name we want to get.
   * \return                    The name of the specified label.
   * \throws std::runtime_error If the manager does not contain the specified label.
   */
  std::string get_label_name(SpaintVoxel::LabelType label) const;

  /**
   * \brief Gets whether or not the manager contains a label with the specified name.
   *
   * \param name  The name of the label we want to check.
   * \return      true, if the manager contains a label with the specified name, or false otherwise.
   */
  bool has_label(const std::string& name) const;

  /**
   * \brief Gets the number of labels that are currently allocated.
   *
   * \return  The number of labels that are currently allocated.
   */
  size_t label_count() const;

  /**
   * \brief Gets the maximum number of labels that the manager is allowed to allocate.
   *
   * \return  The maximum number of labels that the manager is allowed to allocate.
   */
  size_t max_label_count() const;
};

}

#endif
